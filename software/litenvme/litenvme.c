/*
 * This file is part of LiteNVMe.
 *
 * Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * LiteNVMe host library (see litenvme.h). The bring-up sequence follows the hardware-validated
 * firmware (bench/firmware/main.c: nvme_bar0_assign, nvme_configure_mps_once, nvme_admin_init,
 * nvme_io_setup, nvme_autoinit), including its read-lag workarounds, and adds: MRRS capped to
 * MPS (the host-memory responder does not split completions), CAP.TO based timeouts, CAP.MPSMIN /
 * Identify Namespace / MDTS checks, and a root-port memory window sized to the discovered BAR.
 */

#include <stddef.h>
#include <string.h>

#include "litenvme_hal.h"
#include <generated/csr.h>
#include <generated/soc.h>

#include "litenvme.h"

#if !defined(CSR_NVME_STREAMER_BASE) || !defined(CSR_NVME_CTRL_BASE) || !defined(NVME_HOSTMEM_BASE)
#error "generated headers must come from a core built with init_mode=external and with_streamer"
#endif

/* Layout (generated soc.h) ---------------------------------------------------------------------*/

#define HM_BASE        ((uint64_t)NVME_HOSTMEM_BASE)
#define ASQ_ADDR       (HM_BASE + NVME_HOSTMEM_ASQ_OFFSET)
#define ACQ_ADDR       (HM_BASE + NVME_HOSTMEM_ACQ_OFFSET)
#define ID_ADDR        (HM_BASE + NVME_HOSTMEM_ID_OFFSET)
#define IOCQ_ADDR      (HM_BASE + NVME_HOSTMEM_IOCQ_OFFSET)
#define IOSQ_ADDR      (HM_BASE + NVME_HOSTMEM_IOSQ_OFFSET)
#define PRP_ADDR       (HM_BASE + NVME_HOSTMEM_PRP_OFFSET)
#define ADMIN_ENTRIES  2
#define IO_ENTRIES     NVME_QSIZE
#define CORE_LBA_BYTES 512u
#define BEAT_BYTES     (NVME_DATA_WIDTH / 8)

/* NVMe controller registers. */
#define REG_CAP  0x00
#define REG_VS   0x08
#define REG_CC   0x14
#define REG_CSTS 0x1c
#define REG_AQA  0x24
#define REG_ASQ  0x28
#define REG_ACQ  0x30

#define LOG(d, ...) do { if ((d)->verbose) lnv_log(__VA_ARGS__); } while (0)

#define ACCESSOR_TIMEOUT_US 100000u

const char *lnv_strerror(int err)
{
	switch (err) {
	case LNV_OK:       return "ok";
	case LNV_ETIMEOUT: return "timeout";
	case LNV_ENODEV:   return "no link/device";
	case LNV_EBAR:     return "BAR0 error";
	case LNV_ECTRL:    return "controller not ready/fatal";
	case LNV_ECMD:     return "admin command failed";
	case LNV_EUNSUPP:  return "unsupported device parameters";
	case LNV_ERANGE:   return "LBA range outside namespace";
	case LNV_EXFER:    return "stream transfer error";
	case LNV_EVERIFY:  return "pattern mismatch";
	case LNV_ESTATE:   return "not initialized";
	default:           return "unknown error";
	}
}

void lnv_dev_defaults(struct lnv_dev *d)
{
	memset(d, 0, sizeof(*d));
	d->bar0 = NVME_BAR0_BASE;
	d->bus  = 0;
	d->dev  = 1;  /* Downstream device BDF used by the validated bring-up (see firmware). */
	d->fn   = 0;
	d->nsid = 1;
}

/* Polling helper -------------------------------------------------------------------------------*/

static int wait_bit(uint32_t (*rd)(void), uint32_t mask, uint32_t timeout_us, uint32_t *last)
{
	uint64_t t0 = lnv_time_us();
	for (;;) {
		uint32_t v = rd();
		if (v & mask) {
			if (last) *last = v;
			return 0;
		}
		if (lnv_time_us() - t0 > timeout_us) {
			if (last) *last = v;
			return LNV_ETIMEOUT;
		}
	}
}

/* PCIe link / PERST# ---------------------------------------------------------------------------*/

uint32_t lnv_link_status(void)
{
	return pcie_phy_link_status_read();
}

int lnv_link_is_up(uint32_t s)
{
	/* status[0]=link up, phy_status[3:2]=3: DL initialization completed. */
	return (s & 1u) && (((s >> 2) & 3u) == 3u);
}

void lnv_link_describe(uint32_t s, char *buf, int len)
{
	static const char *rate[] = {"2.5GT/s", "5GT/s", "8GT/s", "?"};
	unsigned width = 1u << ((s >> 6) & 7u);
	int n = 0;
	const char *r = rate[(s >> 4) & 3u];
	const char *st = lnv_link_is_up(s) ? "up" : "down";
	/* Tiny formatter (no stdio dependency on bare-metal). */
	const char *parts[] = {"link ", st, ", ", r, " x"};
	for (unsigned p = 0; p < sizeof(parts) / sizeof(parts[0]); p++)
		for (const char *c = parts[p]; *c && n < len - 1; c++) buf[n++] = *c;
	if (width >= 10 && n < len - 1) buf[n++] = '0' + width / 10;
	if (n < len - 1) buf[n++] = '0' + width % 10;
	const char *lt = ", ltssm 0x";
	for (const char *c = lt; *c && n < len - 1; c++) buf[n++] = *c;
	unsigned ltssm = (s >> 9) & 0x3fu;
	const char *hex = "0123456789abcdef";
	if (n < len - 1) buf[n++] = hex[ltssm >> 4];
	if (n < len - 1) buf[n++] = hex[ltssm & 0xf];
	buf[n] = 0;
}

int lnv_link_wait(uint32_t timeout_ms, uint32_t *status)
{
	uint64_t t0 = lnv_time_us();
	uint32_t s;
	do {
		s = lnv_link_status();
		if (lnv_link_is_up(s)) {
			if (status) *status = s;
			return LNV_OK;
		}
		lnv_udelay(1000);
	} while (lnv_time_us() - t0 < (uint64_t)timeout_ms * 1000u);
	if (status) *status = s;
	return LNV_ENODEV;
}

void lnv_perst(int assert_reset)
{
	pcie_ctrl_ctrl_write(assert_reset ? 1u : 0u);
}

/* Host memory (queue/PRP window, via the CSR debug port) ---------------------------------------*/

static void hm_wr32(uint64_t addr, uint32_t v)
{
	nvme_hostmem_csr_adr_write((uint32_t)((addr - HM_BASE) >> 2));
	nvme_hostmem_csr_wdata_write(v);
	nvme_hostmem_csr_we_write(1);
	nvme_hostmem_csr_we_write(0);
}

static uint32_t hm_rd32(uint64_t addr)
{
	nvme_hostmem_csr_adr_write((uint32_t)((addr - HM_BASE) >> 2));
	/* The debug read port has latency: discard the first read (firmware hostmem_rd32). */
	(void)nvme_hostmem_csr_rdata_read();
	return nvme_hostmem_csr_rdata_read();
}

static void hm_fill(uint64_t addr, uint32_t bytes, uint32_t v)
{
	for (uint32_t o = 0; o < bytes; o += 4)
		hm_wr32(addr + o, v);
}

/* PCIe config space of the SSD (CFG TLPs) ------------------------------------------------------*/

static int cfg_op(struct lnv_dev *d, uint8_t reg, int we, uint32_t wdata, uint32_t *rdata)
{
	uint32_t bdf = (uint32_t)d->bus | ((uint32_t)(d->dev & 0x1f) << 8) |
	               ((uint32_t)(d->fn & 0x7) << 13) | ((uint32_t)(reg & 0x3f) << 16);
	uint32_t stat;
	nvme_cfg_cfg_bdf_write(bdf);
	if (we)
		nvme_cfg_cfg_wdata_write(wdata);
	nvme_cfg_cfg_ctrl_write(1u | (we ? 2u : 0u));
	nvme_cfg_cfg_ctrl_write(0);
	if (wait_bit(nvme_cfg_cfg_stat_read, 1u, ACCESSOR_TIMEOUT_US, &stat))
		return LNV_ETIMEOUT;
	if (rdata)
		*rdata = nvme_cfg_cfg_rdata_read();
	return (stat & 2u) ? LNV_ENODEV : LNV_OK;
}

static int cfg_rd(struct lnv_dev *d, uint8_t reg, uint32_t *v)
{
	/* Read-lag safe: the accessor can return the previous transaction's data. Discard one read,
	 * then retry until a clean one (firmware cfg_rd32_retry). */
	int err = LNV_ETIMEOUT;
	(void)cfg_op(d, reg, 0, 0, v);
	for (int i = 0; i < 5; i++)
		if ((err = cfg_op(d, reg, 0, 0, v)) == LNV_OK)
			return LNV_OK;
	return err;
}

static int cfg_wr(struct lnv_dev *d, uint8_t reg, uint32_t v)
{
	return cfg_op(d, reg, 1, v, NULL);
}

static uint8_t cfg_express_cap(struct lnv_dev *d)
{
	uint32_t v;
	if (cfg_rd(d, 1, &v) || !((v >> 20) & 1u)) return 0;  /* Status.CapList. */
	if (cfg_rd(d, 0x34 / 4, &v)) return 0;
	uint8_t off = v & 0xfc;
	for (int g = 0; g < 48 && off >= 0x40; g++) {
		if (cfg_rd(d, off / 4, &v)) return 0;
		if ((v & 0xff) == 0x10) return off;
		off = (v >> 8) & 0xfc;
	}
	return 0;
}

/* Root port's own config space (hard-IP cfg_mgmt) ----------------------------------------------*/

static int rootcfg_op(uint16_t byte_off, int we, uint32_t wdata, uint32_t be, uint32_t *rdata)
{
	pcie_rootcfg_addr_write((byte_off / 4) & 0x3ff);
	pcie_rootcfg_func_write(0);
	if (we) {
		pcie_rootcfg_wdata_write(wdata);
		pcie_rootcfg_be_write(be & 0xf);
	}
	pcie_rootcfg_ctrl_write((we ? 2u : 0u) | 1u);
	if (wait_bit(pcie_rootcfg_stat_read, 1u, ACCESSOR_TIMEOUT_US, NULL))
		return LNV_ETIMEOUT;
	if (rdata)
		*rdata = pcie_rootcfg_rdata_read();
	return LNV_OK;
}

static uint16_t rootcfg_express_cap(void)
{
	uint32_t v;
	if (rootcfg_op(0x34, 0, 0, 0xf, &v)) return 0;
	uint16_t off = v & 0xfc;
	for (int g = 0; g < 48 && off >= 0x40; g++) {
		if (rootcfg_op(off, 0, 0, 0xf, &v)) return 0;
		if ((v & 0xff) == 0x10) return off;
		off = (v >> 8) & 0xfc;
	}
	return 0;
}

/* NVMe controller registers (BAR0 MMIO) --------------------------------------------------------*/

static int mmio_op(uint64_t addr, int we, uint32_t wdata, uint32_t *rdata)
{
	uint32_t stat;
	nvme_mmio_mem_adr_l_write((uint32_t)addr);
	nvme_mmio_mem_adr_h_write((uint32_t)(addr >> 32));
	if (we)
		nvme_mmio_mem_wdata_write(wdata);
	nvme_mmio_mem_ctrl_write(1u | (we ? 2u : 0u) | (0xfu << 4) | (1u << 8));
	nvme_mmio_mem_ctrl_write(0);
	if (wait_bit(nvme_mmio_mem_stat_read, 1u, ACCESSOR_TIMEOUT_US, &stat))
		return LNV_ETIMEOUT;
	if (rdata)
		*rdata = nvme_mmio_mem_rdata_read();
	return (stat & 2u) ? LNV_ENODEV : LNV_OK;
}

static int mmio_rd32(struct lnv_dev *d, uint32_t reg, uint32_t *v)
{
	/* Read-lag safe: read until two consecutive reads agree (firmware mmio_rd32). */
	uint32_t prev, cur = 0;
	int err;
	if ((err = mmio_op(d->bar0 + reg, 0, 0, &prev))) return err;
	for (int i = 0; i < 8; i++) {
		if ((err = mmio_op(d->bar0 + reg, 0, 0, &cur))) return err;
		if (cur == prev) break;
		prev = cur;
	}
	*v = cur;
	return LNV_OK;
}

static int mmio_wr32(struct lnv_dev *d, uint32_t reg, uint32_t v)
{
	return mmio_op(d->bar0 + reg, 1, v, NULL);
}

static int mmio_rd64(struct lnv_dev *d, uint32_t reg, uint64_t *v)
{
	uint32_t lo, hi;
	int err;
	if ((err = mmio_rd32(d, reg, &lo)) || (err = mmio_rd32(d, reg + 4, &hi))) return err;
	*v = ((uint64_t)hi << 32) | lo;
	return LNV_OK;
}

static int mmio_wr64(struct lnv_dev *d, uint32_t reg, uint64_t v)
{
	int err;
	if ((err = mmio_wr32(d, reg, (uint32_t)v))) return err;
	return mmio_wr32(d, reg + 4, (uint32_t)(v >> 32));
}

static uint32_t db_reg(struct lnv_dev *d, uint16_t qid, int is_cq)
{
	uint32_t stride = 4u << ((d->cap >> 32) & 0xf);  /* CAP.DSTRD */
	return 0x1000u + (qid * 2u + (is_cq ? 1u : 0u)) * stride;
}

static uint32_t cap_timeout_ms(struct lnv_dev *d)
{
	uint32_t to = (uint32_t)((d->cap >> 24) & 0xff);  /* CAP.TO, 500 ms units. */
	return (to ? to : 1) * 500u + 500u;
}

static int wait_rdy(struct lnv_dev *d, uint32_t want)
{
	uint64_t t0 = lnv_time_us();
	uint32_t csts = 0;
	do {
		if (mmio_rd32(d, REG_CSTS, &csts) == LNV_OK) {
			if (csts & 2u) return LNV_ECTRL;  /* CSTS.CFS */
			if ((csts & 1u) == want) return LNV_OK;
		}
		lnv_udelay(1000);
	} while (lnv_time_us() - t0 < (uint64_t)cap_timeout_ms(d) * 1000u);
	return LNV_ETIMEOUT;
}

/* Bring-up steps -------------------------------------------------------------------------------*/

static int bar0_assign(struct lnv_dev *d)
{
	uint32_t id, bar0_orig, bar0, bar1 = 0;
	int err;
	if ((err = cfg_rd(d, 0, &id))) return err;
	if (id == 0xffffffffu || id == 0)
		return LNV_ENODEV;
	LOG(d, "VID/DID      : 0x%08x\n", (unsigned)id);
	if ((err = cfg_rd(d, 4, &bar0_orig))) return err;
	if (bar0_orig & 1u)
		return LNV_EBAR;  /* I/O BAR. */
	int is64 = ((bar0_orig >> 1) & 3u) == 2u;

	/* Size BAR0 (and BAR1 for a 64-bit BAR) with memory decode and bus mastering off. */
	uint32_t cmd;
	if ((err = cfg_rd(d, 1, &cmd))) return err;
	cfg_wr(d, 1, cmd & ~((1u << 1) | (1u << 2)));
	cfg_wr(d, 4, 0xffffffffu);
	if ((err = cfg_rd(d, 4, &bar0))) return err;
	uint64_t mask = bar0 & 0xfffffff0u;
	if (is64) {
		cfg_wr(d, 5, 0xffffffffu);
		if ((err = cfg_rd(d, 5, &bar1))) return err;
		mask |= (uint64_t)bar1 << 32;
	} else {
		mask |= 0xffffffff00000000ull;
	}
	uint64_t size = ~mask + 1u;
	if (size == 0 || (size & (size - 1)) || (d->bar0 & (size - 1)))
		return LNV_EBAR;
	d->bar0_size = size;

	cfg_wr(d, 4, (uint32_t)d->bar0 | (bar0_orig & 0xfu));
	if (is64)
		cfg_wr(d, 5, (uint32_t)(d->bar0 >> 32));

	/* Command: memory space + bus master, INTx disabled (completions are polled). */
	if ((err = cfg_rd(d, 1, &cmd))) return err;
	cmd = (cmd & 0xffff0000u) | ((cmd & 0xffffu) | (1u << 1) | (1u << 2) | (1u << 10));
	cfg_wr(d, 1, cmd);
	LOG(d, "BAR0         : 0x%llx (size 0x%llx, %s)\n", (unsigned long long)d->bar0,
	    (unsigned long long)size, is64 ? "64-bit" : "32-bit");
	return LNV_OK;
}

static int configure_link(struct lnv_dev *d)
{
	uint32_t rdevcap, rdc, sdevcap, sdc;
	uint16_t rcap = rootcfg_express_cap();
	uint8_t  scap = cfg_express_cap(d);
	if (!rcap || !scap)
		return LNV_ENODEV;
	if (rootcfg_op(rcap + 0x04, 0, 0, 0xf, &rdevcap) || rootcfg_op(rcap + 0x08, 0, 0, 0xf, &rdc))
		return LNV_ETIMEOUT;
	if (cfg_rd(d, (scap + 0x04) / 4, &sdevcap) || cfg_rd(d, (scap + 0x08) / 4, &sdc))
		return LNV_ETIMEOUT;

	uint32_t rsup = rdevcap & 7u, ssup = sdevcap & 7u;
	uint32_t mps  = rsup < ssup ? rsup : ssup;
	/* Root first (so it can receive larger MemWr), then the SSD. The SSD's MRRS is capped at
	 * the MPS: the host-memory responder answers each MemRd with one CplD. */
	uint32_t rnew = (rdc & ~(7u << 5)) | (mps << 5);
	rootcfg_op(rcap + 0x08, 1, (rdc & 0xffff0000u) | (rnew & 0xffffu), 0x3, NULL);
	uint32_t snew = (sdc & ~((7u << 5) | (7u << 12))) | (mps << 5) | (mps << 12);
	cfg_wr(d, (scap + 0x08) / 4, (sdc & 0xffff0000u) | (snew & 0xffffu));
	d->mps_bytes  = 128u << mps;
	d->mrrs_bytes = 128u << mps;

	/* Root-port bridge memory window covering BAR0 (MemBase/MemLimit, 1 MiB granularity), so
	 * the root port forwards our MemRd/MemWr to the SSD. */
	uint64_t lim = d->bar0 + (d->bar0_size < 0x100000u ? 0x100000u : d->bar0_size) - 1;
	uint32_t win = (uint32_t)(((lim >> 16) & 0xfff0u) << 16) | (uint32_t)((d->bar0 >> 16) & 0xfff0u);
	rootcfg_op(0x20, 1, win, 0xf, NULL);
	LOG(d, "MPS/MRRS     : %u/%u B (root sup %u B, SSD sup %u B)\n", (unsigned)d->mps_bytes,
	    (unsigned)d->mrrs_bytes, 128u << rsup, 128u << ssup);
	return LNV_OK;
}

static int admin_submit(struct lnv_dev *d, uint32_t *cmd, uint32_t *cqe)
{
	uint64_t sqe = ASQ_ADDR + (uint64_t)d->asq_tail * 64;
	uint64_t cq  = ACQ_ADDR + (uint64_t)d->acq_head * 16;
	int err;
	cmd[0] = (cmd[0] & 0xffffu) | ((uint32_t)d->acid++ << 16);
	for (int i = 0; i < 16; i++)
		hm_wr32(sqe + 4 * i, cmd[i]);
	/* Clear the CQ slot so a stale phase cannot match. */
	for (int i = 0; i < 3; i++)
		hm_wr32(cq + 4 * i, 0);
	hm_wr32(cq + 12, ((d->acq_phase ^ 1u) & 1u) << 16);

	d->asq_tail = (d->asq_tail + 1) % ADMIN_ENTRIES;
	if ((err = mmio_wr32(d, db_reg(d, 0, 0), d->asq_tail))) return err;

	uint64_t t0 = lnv_time_us();
	for (;;) {
		cqe[3] = hm_rd32(cq + 12);
		if (((cqe[3] >> 16) & 1u) == d->acq_phase)
			break;
		if (lnv_time_us() - t0 > 2000000u)
			return LNV_ETIMEOUT;
	}
	cqe[0] = hm_rd32(cq + 0);
	cqe[1] = hm_rd32(cq + 4);
	cqe[2] = hm_rd32(cq + 8);

	d->acq_head = (d->acq_head + 1) % ADMIN_ENTRIES;
	if (d->acq_head == 0)
		d->acq_phase ^= 1u;
	if ((err = mmio_wr32(d, db_reg(d, 0, 1), d->acq_head))) return err;

	uint32_t sts = cqe[3] >> 17;  /* SC[7:0], SCT[10:8] */
	if (sts & 0x7ffu) {
		LOG(d, "admin op 0x%02x failed: SCT %u SC 0x%02x\n", (unsigned)(cmd[0] & 0xff),
		    (unsigned)((sts >> 8) & 7u), (unsigned)(sts & 0xffu));
		return LNV_ECMD;
	}
	return LNV_OK;
}

static int admin_init(struct lnv_dev *d)
{
	uint32_t cc = 0, vs = 0;
	int err;
	for (int tries = 0; tries < 20; tries++) {
		if (mmio_rd64(d, REG_CAP, &d->cap) == LNV_OK && d->cap != 0 && d->cap != ~0ull)
			break;
		d->cap = 0;
		lnv_udelay(50000);
	}
	if (d->cap == 0)
		return LNV_ENODEV;
	mmio_rd32(d, REG_VS, &vs);
	LOG(d, "CAP/VS       : 0x%016llx / %u.%u\n", (unsigned long long)d->cap,
	    (unsigned)(vs >> 16), (unsigned)((vs >> 8) & 0xff));
	if (((d->cap >> 48) & 0xf) != 0)
		return LNV_EUNSUPP;  /* CAP.MPSMIN > 4 KiB: the core builds 4 KiB PRPs. */
	if (!((d->cap >> 37) & 1u))
		return LNV_EUNSUPP;  /* CAP.CSS: NVM command set not supported. */
	if ((((d->cap & 0xffff) + 1) < IO_ENTRIES))
		return LNV_EUNSUPP;  /* CAP.MQES below the core's IO queue size. */

	if ((err = mmio_rd32(d, REG_CC, &cc))) return err;
	if (cc & 1u) {
		mmio_wr32(d, REG_CC, cc & ~1u);
		if ((err = wait_rdy(d, 0))) return err;
	}
	d->asq_tail = d->acq_head = 0;
	d->acq_phase = 1;
	d->acid = 1;
	hm_fill(ASQ_ADDR, ADMIN_ENTRIES * 64, 0);
	hm_fill(ACQ_ADDR, ADMIN_ENTRIES * 16, 0);

	uint32_t aqa = (ADMIN_ENTRIES - 1) | ((ADMIN_ENTRIES - 1) << 16);
	if ((err = mmio_wr32(d, REG_AQA, aqa))) return err;
	if ((err = mmio_wr64(d, REG_ASQ, ASQ_ADDR))) return err;
	if ((err = mmio_wr64(d, REG_ACQ, ACQ_ADDR))) return err;
	/* CC: EN, CSS=NVM, MPS=4 KiB, IOSQES=64 B, IOCQES=16 B. */
	if ((err = mmio_wr32(d, REG_CC, 1u | (6u << 16) | (4u << 20)))) return err;
	return wait_rdy(d, 1);
}

static void id_string(uint64_t addr, char *dst, int n)
{
	for (int i = 0; i < n; i += 4) {
		uint32_t v = hm_rd32(addr + i);
		for (int b = 0; b < 4 && i + b < n; b++)
			dst[i + b] = (char)((v >> (8 * b)) & 0xff);
	}
	dst[n] = 0;
	for (int i = n - 1; i >= 0 && (dst[i] == ' ' || dst[i] == 0); i--)
		dst[i] = 0;
}

static int identify(struct lnv_dev *d)
{
	uint32_t cmd[16], cqe[4];
	int err;
	struct lnv_identify *id = &d->id;

	/* Identify Controller (CNS=1). */
	memset(cmd, 0, sizeof(cmd));
	cmd[0]  = 0x06;
	cmd[6]  = (uint32_t)ID_ADDR;
	cmd[7]  = (uint32_t)(ID_ADDR >> 32);
	cmd[10] = 1;
	hm_fill(ID_ADDR, 0x210, 0);
	if ((err = admin_submit(d, cmd, cqe))) return err;
	id->vid  = (uint16_t)hm_rd32(ID_ADDR);
	id_string(ID_ADDR + 0x04, id->sn, 20);
	id_string(ID_ADDR + 0x18, id->mn, 40);
	id_string(ID_ADDR + 0x40, id->fr, 8);
	id->mdts = (uint8_t)(hm_rd32(ID_ADDR + 0x4c) >> 8);
	id->ver  = hm_rd32(ID_ADDR + 0x50);
	id->nn   = hm_rd32(ID_ADDR + 0x204);

	/* Identify Namespace (CNS=0). */
	memset(cmd, 0, sizeof(cmd));
	cmd[0]  = 0x06;
	cmd[1]  = d->nsid;
	cmd[6]  = (uint32_t)ID_ADDR;
	cmd[7]  = (uint32_t)(ID_ADDR >> 32);
	cmd[10] = 0;
	hm_fill(ID_ADDR, 0x100, 0);
	if ((err = admin_submit(d, cmd, cqe))) return err;
	id->nsze = hm_rd32(ID_ADDR + 0x00) | ((uint64_t)hm_rd32(ID_ADDR + 0x04) << 32);
	id->ncap = hm_rd32(ID_ADDR + 0x08) | ((uint64_t)hm_rd32(ID_ADDR + 0x0c) << 32);
	uint32_t flbas = (hm_rd32(ID_ADDR + 0x18) >> 16) & 0xf;  /* FLBAS @ byte 26 */
	uint32_t lbaf  = hm_rd32(ID_ADDR + 0x80 + 4 * flbas);
	id->lba_ms    = (uint16_t)(lbaf & 0xffff);
	id->lba_bytes = 1u << ((lbaf >> 16) & 0xff);

	LOG(d, "Model/SN/FW  : %s / %s / %s\n", id->mn, id->sn, id->fr);
	LOG(d, "Namespace %u  : %llu x %u B (MDTS %u)\n", (unsigned)d->nsid,
	    (unsigned long long)id->nsze, (unsigned)id->lba_bytes, (unsigned)id->mdts);

	if (id->nsze == 0)
		return LNV_ENODEV;
	if (id->lba_bytes != CORE_LBA_BYTES || id->lba_ms != 0)
		return LNV_EUNSUPP;  /* The core is built for 512 B LBAs without metadata. */
#ifdef NVME_STREAMER_SLOT_BYTES
	if (id->mdts && ((4096ull << id->mdts) < NVME_STREAMER_SLOT_BYTES))
		return LNV_EUNSUPP;  /* A slot-sized command exceeds the drive's max transfer. */
#endif
	return LNV_OK;
}

static int io_queues(struct lnv_dev *d)
{
	uint32_t cmd[16], cqe[4];
	int err;

	/* Set Features: Number of Queues (1 SQ / 1 CQ). */
	memset(cmd, 0, sizeof(cmd));
	cmd[0]  = 0x09;
	cmd[10] = 0x07;
	if ((err = admin_submit(d, cmd, cqe))) return err;

	hm_fill(IOCQ_ADDR, IO_ENTRIES * 16, 0);
	hm_fill(IOSQ_ADDR, IO_ENTRIES * 64, 0);

	/* Create IO CQ: physically contiguous, interrupts disabled (the engine polls). */
	memset(cmd, 0, sizeof(cmd));
	cmd[0]  = 0x05;
	cmd[6]  = (uint32_t)IOCQ_ADDR;
	cmd[7]  = (uint32_t)(IOCQ_ADDR >> 32);
	cmd[10] = NVME_QID | ((IO_ENTRIES - 1u) << 16);
	cmd[11] = 1;
	if ((err = admin_submit(d, cmd, cqe))) return err;

	/* Create IO SQ bound to that CQ. */
	memset(cmd, 0, sizeof(cmd));
	cmd[0]  = 0x01;
	cmd[6]  = (uint32_t)IOSQ_ADDR;
	cmd[7]  = (uint32_t)(IOSQ_ADDR >> 32);
	cmd[10] = NVME_QID | ((IO_ENTRIES - 1u) << 16);
	cmd[11] = ((uint32_t)NVME_QID << 16) | 1u;
	return admin_submit(d, cmd, cqe);
}

static void engine_enable(struct lnv_dev *d)
{
	uint64_t sq_db = d->bar0 + db_reg(d, NVME_QID, 0);
	uint64_t cq_db = d->bar0 + db_reg(d, NVME_QID, 1);
	nvme_engine_enable_write(0);
	nvme_engine_sq_base_lo_write((uint32_t)IOSQ_ADDR);
	nvme_engine_sq_base_hi_write((uint32_t)(IOSQ_ADDR >> 32));
	nvme_engine_cq_base_lo_write((uint32_t)IOCQ_ADDR);
	nvme_engine_cq_base_hi_write((uint32_t)(IOCQ_ADDR >> 32));
	nvme_engine_sq_db_lo_write((uint32_t)sq_db);
	nvme_engine_sq_db_hi_write((uint32_t)(sq_db >> 32));
	nvme_engine_cq_db_lo_write((uint32_t)cq_db);
	nvme_engine_cq_db_hi_write((uint32_t)(cq_db >> 32));
	nvme_engine_prp_list_lo_write((uint32_t)PRP_ADDR);
	nvme_engine_prp_list_hi_write((uint32_t)(PRP_ADDR >> 32));
	nvme_engine_enable_write(1);
}

int lnv_init(struct lnv_dev *d)
{
	int err;
	d->ready = 0;
	nvme_ctrl_init_done_write(0);
	nvme_ctrl_init_error_write(0);
	nvme_engine_enable_write(0);

	if ((err = lnv_link_wait(1000, &d->link_status))) goto fail;
	if ((err = bar0_assign(d)))    goto fail;
	if ((err = configure_link(d))) goto fail;
	if ((err = admin_init(d)))     goto fail;
	if ((err = identify(d)))       goto fail;
	if ((err = io_queues(d)))      goto fail;
	engine_enable(d);
	nvme_ctrl_init_done_write(1);
	d->ready = 1;
	return LNV_OK;
fail:
	nvme_ctrl_init_error_write(1);
	return err;
}

/* Stream ring ----------------------------------------------------------------------------------*/

static void read_stats(struct lnv_xfer_stats *st)
{
	uint32_t e = nvme_streamer_errors_read();
	st->cycles       = nvme_streamer_cycles_read();
	st->stall_cycles = nvme_streamer_stall_cycles_read();
	st->beats        = nvme_streamer_data_beats_read();
	st->submitted    = nvme_streamer_submitted_read();
	st->completed    = nvme_streamer_completed_read();
	st->err_count    = e & 0xffffu;
	st->err_status   = e >> 16;
	st->stale        = nvme_streamer_stale_read();
	st->slots_hwm    = nvme_streamer_slots_hwm_read();
	st->error        = (nvme_streamer_status_read() >> 2) & 1u;
}

static int range_ok(struct lnv_dev *d, uint64_t lba, uint32_t count)
{
	return count != 0 && lba < d->id.nsze && count <= d->id.nsze - lba;
}

int lnv_stream(struct lnv_dev *d, int write, uint64_t lba, uint32_t count, uint32_t timeout_ms,
               struct lnv_xfer_stats *st)
{
	if (!d->ready || !nvme_ctrl_init_done_read() || !(nvme_streamer_status_read() & 1u))
		return LNV_ESTATE;  /* Not initialized, or a previous transfer is still running. */
	if (!range_ok(d, lba, count))
		return LNV_ERANGE;
	nvme_streamer_lba_lo_write((uint32_t)lba);
	nvme_streamer_lba_hi_write((uint32_t)(lba >> 32));
	nvme_streamer_count_write(count);
	nvme_streamer_nsid_write(d->nsid);
	nvme_streamer_ctrl_write(write ? 2u : 0u);         /* Set direction, then pulse start. */
	nvme_streamer_ctrl_write((write ? 2u : 0u) | 1u);

	uint64_t t0 = lnv_time_us();
	while (!(nvme_streamer_status_read() & 1u)) {        /* status.done */
		if (lnv_time_us() - t0 > (uint64_t)timeout_ms * 1000u) {
			if (st) read_stats(st);
			return LNV_ETIMEOUT;
		}
	}
	struct lnv_xfer_stats local;
	if (!st) st = &local;
	read_stats(st);
	if (st->submitted == 0)
		return LNV_ESTATE;  /* Start was not taken. */
	return st->error ? LNV_EXFER : LNV_OK;
}

static void pattern_setup(int check, uint64_t lba, uint32_t count, uint32_t seed)
{
	if (check) {
		nvme_pattern_check_ctrl_lba_lo_write((uint32_t)lba);
		nvme_pattern_check_ctrl_lba_hi_write((uint32_t)(lba >> 32));
		nvme_pattern_check_ctrl_count_write(count);
		nvme_pattern_check_ctrl_seed_write(seed);
		nvme_pattern_check_ctrl_ctrl_write(1);
	} else {
		nvme_pattern_gen_ctrl_lba_lo_write((uint32_t)lba);
		nvme_pattern_gen_ctrl_lba_hi_write((uint32_t)(lba >> 32));
		nvme_pattern_gen_ctrl_count_write(count);
		nvme_pattern_gen_ctrl_seed_write(seed);
		nvme_pattern_gen_ctrl_ctrl_write(1);
	}
}

int lnv_pattern_write(struct lnv_dev *d, uint64_t lba, uint32_t count, uint32_t seed,
                      uint32_t timeout_ms, struct lnv_xfer_stats *st)
{
	if (!d->ready)
		return LNV_ESTATE;
	if (!range_ok(d, lba, count))
		return LNV_ERANGE;
	nvme_stream_mux_sel_write(1);
	pattern_setup(0, lba, count, seed);
	int err = lnv_stream(d, 1, lba, count, timeout_ms, st);
	if (err == LNV_OK && !(nvme_pattern_gen_status_read() & 1u))
		err = LNV_EXFER;  /* Generator not drained: stream/length mismatch. */
	return err;
}

int lnv_pattern_verify(struct lnv_dev *d, uint64_t lba, uint32_t count, uint32_t seed,
                       uint32_t timeout_ms, struct lnv_pattern_result *res)
{
	if (!d->ready)
		return LNV_ESTATE;
	if (!range_ok(d, lba, count))
		return LNV_ERANGE;
	nvme_stream_mux_sel_write(1);
	pattern_setup(1, lba, count, seed);
	int err = lnv_stream(d, 0, lba, count, timeout_ms, &res->rd);
	res->beats_checked  = nvme_pattern_check_beats_read();
	res->bad_beats      = nvme_pattern_check_errors_read();
	res->first_bad_beat = nvme_pattern_check_first_err_read();
	res->last_error     = (nvme_pattern_check_status_read() >> 2) & 1u;
	if (err)
		return err;
	if (res->bad_beats || res->last_error ||
	    res->beats_checked != count * (CORE_LBA_BYTES / BEAT_BYTES))
		return LNV_EVERIFY;
	return LNV_OK;
}

double lnv_xfer_bytes_per_s(const struct lnv_xfer_stats *st, uint32_t count)
{
	if (!st->cycles)
		return 0.0;
	return (double)count * CORE_LBA_BYTES * (double)CONFIG_CLOCK_FREQUENCY / (double)st->cycles;
}
