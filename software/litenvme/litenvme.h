/*
 * This file is part of LiteNVMe.
 *
 * Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * LiteNVMe host library: one-time NVMe bring-up and stream-ring control of a generated core
 * (init_mode="external", with_streamer) from an external processor over AXI-Lite. The PL owns
 * the steady-state data path; this code only configures it and starts/monitors transfers.
 */

#ifndef LITENVME_H
#define LITENVME_H

#include <stdint.h>

enum {
	LNV_OK          =  0,
	LNV_ETIMEOUT    = -1,  /* Accessor, controller or transfer timeout.            */
	LNV_ENODEV      = -2,  /* No link / no device at the configured BDF.           */
	LNV_EBAR        = -3,  /* BAR0 missing, I/O, bad size or misaligned.           */
	LNV_ECTRL       = -4,  /* Controller fatal status / did not become ready.      */
	LNV_ECMD        = -5,  /* Admin command completed with an error status.        */
	LNV_EUNSUPP     = -6,  /* Device parameters this core does not support.        */
	LNV_ERANGE      = -7,  /* LBA range outside the namespace.                     */
	LNV_EXFER       = -8,  /* Stream transfer reported an error completion.        */
	LNV_EVERIFY     = -9,  /* Pattern check mismatch.                              */
	LNV_ESTATE      = -10, /* Called in the wrong state (e.g. before init).        */
};

struct lnv_identify {
	char     mn[41], sn[21], fr[9];
	uint16_t vid;
	uint32_t ver;
	uint8_t  mdts;          /* Max transfer = 2^mdts * min page (0 = no limit). */
	uint32_t nn;
	uint64_t nsze, ncap;    /* Namespace 1, in logical blocks. */
	uint32_t lba_bytes;
	uint16_t lba_ms;        /* Metadata bytes per LBA (must be 0). */
};

struct lnv_dev {
	/* Configuration (defaults from generated soc.h; override before lnv_init). */
	uint64_t bar0;
	uint8_t  bus, dev, fn;  /* SSD config-space BDF as seen by the accessor. */
	uint32_t nsid;
	int      verbose;

	/* Discovered state. */
	uint64_t cap;
	uint64_t bar0_size;
	uint32_t mps_bytes, mrrs_bytes;
	uint32_t link_status;
	struct lnv_identify id;
	int      ready;

	/* Admin queue ring state. */
	uint16_t asq_tail, acq_head, acq_phase, acid;
};

struct lnv_xfer_stats {
	uint64_t cycles, stall_cycles;
	uint32_t beats, submitted, completed;
	uint32_t err_count, err_status, stale, slots_hwm;
	int      error;
};

struct lnv_pattern_result {
	struct lnv_xfer_stats wr, rd;
	uint32_t beats_checked, bad_beats, first_bad_beat;
	int      last_error;
};

void        lnv_dev_defaults(struct lnv_dev *d);
const char *lnv_strerror(int err);

/* PCIe link. */
uint32_t    lnv_link_status(void);
int         lnv_link_is_up(uint32_t status);
int         lnv_link_wait(uint32_t timeout_ms, uint32_t *status);
void        lnv_link_describe(uint32_t status, char *buf, int len);
void        lnv_perst(int assert_reset);

/* Bring-up: BAR0, MPS/MRRS, controller enable, Identify, IO queues, engine, init_done. */
int         lnv_init(struct lnv_dev *d);

/* Stream ring transfer of `count` logical blocks at `lba` (blocking, polled). */
int         lnv_stream(struct lnv_dev *d, int write, uint64_t lba, uint32_t count,
                       uint32_t timeout_ms, struct lnv_xfer_stats *st);

/* Built-in pattern generator/checker round trip (writes the LBA range!). */
int         lnv_pattern_write(struct lnv_dev *d, uint64_t lba, uint32_t count, uint32_t seed,
                              uint32_t timeout_ms, struct lnv_xfer_stats *st);
int         lnv_pattern_verify(struct lnv_dev *d, uint64_t lba, uint32_t count, uint32_t seed,
                               uint32_t timeout_ms, struct lnv_pattern_result *res);

/* Throughput helper: bytes/s from a transfer's cycle count at the core clock. */
double      lnv_xfer_bytes_per_s(const struct lnv_xfer_stats *st, uint32_t count);

#endif /* LITENVME_H */
