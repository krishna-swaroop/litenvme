/*
 * This file is part of LiteNVMe.
 *
 * Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * litenvme_test: bring up a generated LiteNVMe core from Linux on the Zynq PS and run the
 * built-in pattern round trip (PL generator -> SSD -> PL checker) with throughput reporting.
 *
 *   litenvme_test -a 0xa0000000 status
 *   litenvme_test -a 0xa0000000 -v init
 *   litenvme_test -a 0xa0000000 --serial <SSD SN> init pattern 0x100000 1G
 *
 * Commands run in order within one invocation (the admin queue state lives in this process).
 * Anything that writes to the SSD requires --serial matching Identify Controller, and only
 * touches the LBA range given on the command line.
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "litenvme_hal.h"
#include <generated/csr.h>
#include <generated/soc.h>

#include "hal_linux.h"
#include "litenvme.h"

#define APERTURE_BYTES 0x10000

static void usage(const char *argv0)
{
	fprintf(stderr,
		"usage: %s -a <aperture phys addr> [-d /dev/mem|/dev/uioN] [-v] [--serial SN] cmd...\n"
		"commands:\n"
		"  status                      link, init state, engine/streamer counters\n"
		"  perst                       pulse PERST# (SSD + root port reset), wait for link\n"
		"  init                        NVMe bring-up (BAR0, MPS, admin, Identify, IO queues)\n"
		"  pattern LBA COUNT [SEED]    write pattern then read back and verify   (writes!)\n"
		"  verify  LBA COUNT SEED      read back and verify a previously written pattern\n"
		"  bench   LBA COUNT N [SEED]  N x (pattern write + verify), with throughput   (writes!)\n"
		"COUNT is in 512 B blocks, or a size with K/M/G suffix (binary, bytes).\n",
		argv0);
}

static int parse_u64(const char *s, uint64_t *v)
{
	char *end;
	*v = strtoull(s, &end, 0);
	return end != s && *end == 0 ? 0 : -1;
}

static int parse_count(const char *s, uint32_t *count)
{
	char *end;
	uint64_t v = strtoull(s, &end, 0), mul = 0;
	if (end == s) return -1;
	if (*end == 'K' || *end == 'k') mul = 1ull << 10;
	if (*end == 'M' || *end == 'm') mul = 1ull << 20;
	if (*end == 'G' || *end == 'g') mul = 1ull << 30;
	if (mul) {
		v *= mul;
		if (v % 512 || end[1] != 0) return -1;
		v /= 512;
	} else if (*end != 0) {
		return -1;
	}
	if (v == 0 || v > 0xffffffffu) return -1;
	*count = (uint32_t)v;
	return 0;
}

static uint32_t xfer_timeout_ms(uint32_t count)
{
	/* Generous: 10 s + time at 50 MB/s. */
	return 10000u + (uint32_t)(((uint64_t)count * 512u) / 50000u);
}

static void print_xfer(const char *what, const struct lnv_xfer_stats *st, uint32_t count)
{
	double bytes = (double)count * 512.0;
	double secs  = (double)st->cycles / (double)CONFIG_CLOCK_FREQUENCY;
	double rate  = lnv_xfer_bytes_per_s(st, count);
	printf("%-6s %9.1f MiB in %8.3f ms = %8.1f MB/s | cmds %u/%u, slot hwm %u, "
	       "user stalls %.1f%%, errors %u (status 0x%04x), stale %u\n",
	       what, bytes / (1 << 20), secs * 1e3, rate / 1e6, st->completed, st->submitted,
	       st->slots_hwm, st->cycles ? 100.0 * (double)st->stall_cycles / (double)st->cycles : 0.0,
	       st->err_count, st->err_status, st->stale);
}

static void cmd_status(void)
{
	char buf[96];
	uint32_t s = lnv_link_status();
	lnv_link_describe(s, buf, sizeof(buf));
	printf("pcie     : %s (raw 0x%08x), MPS %u B, MRRS %u B\n", buf, s,
	       (unsigned)pcie_phy_max_payload_size_read(), (unsigned)pcie_phy_max_request_size_read());
	printf("init     : done %u, error %u\n", (unsigned)nvme_ctrl_init_done_read(),
	       (unsigned)nvme_ctrl_init_error_read());
	uint32_t es = nvme_engine_status_read();
	printf("engine   : enable %u, busy %u, inflight %u, submitted %u, completed %u\n",
	       (unsigned)nvme_engine_enable_read(), es & 1u, (es >> 8) & 0xffu,
	       (unsigned)nvme_engine_submitted_read(), (unsigned)nvme_engine_completed_read());
	uint32_t ss = nvme_streamer_status_read();
	printf("streamer : done %u, busy %u, error %u, slots %u x %u B\n", ss & 1u, (ss >> 1) & 1u,
	       (ss >> 2) & 1u, (unsigned)NVME_STREAMER_NSLOTS, (unsigned)NVME_STREAMER_SLOT_BYTES);
}

static int check_write_allowed(struct lnv_dev *d, const char *serial)
{
	if (!d->ready) {
		fprintf(stderr, "error: run 'init' first (in the same invocation)\n");
		return -1;
	}
	if (!serial || strcmp(serial, d->id.sn) != 0) {
		fprintf(stderr, "error: refusing to write: pass --serial '%s' to confirm this drive\n",
		        d->id.sn);
		return -1;
	}
	return 0;
}

int main(int argc, char **argv)
{
	const char *dev = "/dev/mem", *serial = NULL;
	uint64_t aperture = 0;
	int verbose = 0, i = 1, rc = 0;

	for (; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-a") && i + 1 < argc) {
			if (parse_u64(argv[++i], &aperture)) { usage(argv[0]); return 2; }
		} else if (!strcmp(argv[i], "-d") && i + 1 < argc) {
			dev = argv[++i];
		} else if (!strcmp(argv[i], "--serial") && i + 1 < argc) {
			serial = argv[++i];
		} else if (!strcmp(argv[i], "-v")) {
			verbose = 1;
		} else {
			usage(argv[0]);
			return 2;
		}
	}
	if (!aperture || i >= argc) {
		usage(argv[0]);
		return 2;
	}
	if (lnv_linux_map(dev, aperture, APERTURE_BYTES))
		return 1;

	struct lnv_dev d;
	lnv_dev_defaults(&d);
	d.verbose = verbose;

	while (i < argc && rc == 0) {
		const char *cmd = argv[i++];
		if (!strcmp(cmd, "status")) {
			cmd_status();
		} else if (!strcmp(cmd, "perst")) {
			lnv_perst(1);
			lnv_udelay(10000);
			lnv_perst(0);
			uint32_t s;
			char buf[96];
			int err = lnv_link_wait(2000, &s);
			lnv_link_describe(s, buf, sizeof(buf));
			printf("perst    : %s\n", buf);
			d.ready = 0;
			if (err) rc = 1;
		} else if (!strcmp(cmd, "init")) {
			int err = lnv_init(&d);
			if (err) {
				fprintf(stderr, "init failed: %s\n", lnv_strerror(err));
				rc = 1;
				break;
			}
			char buf[96];
			lnv_link_describe(d.link_status, buf, sizeof(buf));
			printf("init     : OK, %s, MPS/MRRS %u/%u B\n", buf, d.mps_bytes, d.mrrs_bytes);
			printf("ssd      : %s  SN %s  FW %s  VID 0x%04x  NVMe %u.%u\n", d.id.mn, d.id.sn,
			       d.id.fr, d.id.vid, (unsigned)(d.id.ver >> 16), (unsigned)((d.id.ver >> 8) & 0xff));
			printf("ns %u     : %" PRIu64 " blocks x %u B (%.1f GB), MDTS %u\n", d.nsid, d.id.nsze,
			       d.id.lba_bytes, (double)d.id.nsze * d.id.lba_bytes / 1e9, d.id.mdts);
		} else if (!strcmp(cmd, "pattern") || !strcmp(cmd, "verify") || !strcmp(cmd, "bench")) {
			int write = strcmp(cmd, "verify") != 0;
			int bench = !strcmp(cmd, "bench");
			uint64_t lba, iters = 1, seed64 = 0;
			uint32_t count;
			if (i + 1 >= argc || parse_u64(argv[i], &lba) || parse_count(argv[i + 1], &count)) {
				usage(argv[0]);
				rc = 2;
				break;
			}
			i += 2;
			if (bench) {
				if (i >= argc || parse_u64(argv[i++], &iters) || iters == 0) { usage(argv[0]); rc = 2; break; }
			}
			if (i < argc && parse_u64(argv[i], &seed64) == 0) {
				i++;
			} else if (!write) {
				fprintf(stderr, "verify needs the SEED used to write\n");
				rc = 2;
				break;
			} else {
				seed64 = (uint64_t)time(NULL);
			}
			if (write && check_write_allowed(&d, serial)) { rc = 1; break; }
			for (uint64_t k = 0; k < iters && rc == 0; k++) {
				uint32_t seed = (uint32_t)seed64 + (uint32_t)k;
				struct lnv_xfer_stats wst;
				struct lnv_pattern_result res;
				memset(&res, 0, sizeof(res));
				if (write) {
					int err = lnv_pattern_write(&d, lba, count, seed, xfer_timeout_ms(count), &wst);
					print_xfer("write", &wst, count);
					if (err) { fprintf(stderr, "write failed: %s\n", lnv_strerror(err)); rc = 1; break; }
				}
				int err = lnv_pattern_verify(&d, lba, count, seed, xfer_timeout_ms(count), &res);
				print_xfer("read", &res.rd, count);
				printf("verify : seed 0x%08x, %u beats checked, %u bad (first at beat %u)%s -> %s\n",
				       seed, res.beats_checked, res.bad_beats, res.first_bad_beat,
				       res.last_error ? ", TLAST misplaced" : "", err ? lnv_strerror(err) : "PASS");
				if (err) rc = 1;
			}
		} else {
			fprintf(stderr, "unknown command '%s'\n", cmd);
			usage(argv[0]);
			rc = 2;
		}
	}

	lnv_linux_unmap();
	return rc;
}
