/*
 * This file is part of LiteNVMe.
 *
 * Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Simulation entry point: run the unmodified host library against the simulated core, the same
 * way litenvme_test does on hardware (init, pattern write, verify, negative verify, re-init),
 * and report results through a flat array the Python testbench decodes (see LNV_SIM_* below).
 */

#include <string.h>

#include "litenvme_hal.h"
#include "litenvme.h"

enum {
	LNV_SIM_RC_INIT = 0,
	LNV_SIM_RC_WRITE,
	LNV_SIM_RC_VERIFY,
	LNV_SIM_RC_VERIFY_BADSEED,
	LNV_SIM_RC_REINIT,
	LNV_SIM_RC_VERIFY_AFTER_REINIT,
	LNV_SIM_MPS,
	LNV_SIM_MRRS,
	LNV_SIM_NSZE,
	LNV_SIM_LBA_BYTES,
	LNV_SIM_WR_CYCLES,
	LNV_SIM_WR_SUBMITTED,
	LNV_SIM_WR_COMPLETED,
	LNV_SIM_WR_HWM,
	LNV_SIM_RD_CYCLES,
	LNV_SIM_RD_SUBMITTED,
	LNV_SIM_RD_COMPLETED,
	LNV_SIM_CHECKED,
	LNV_SIM_BAD,
	LNV_SIM_BAD_BADSEED,
	LNV_SIM_RC_RANGE,
	LNV_SIM_N
};

int lnv_sim_main(uint64_t lba, uint32_t count, uint32_t seed, uint64_t *out, char *sn, char *mn)
{
	struct lnv_dev d;
	struct lnv_xfer_stats wst;
	struct lnv_pattern_result res, bad;

	memset(out, 0, sizeof(uint64_t) * LNV_SIM_N);
	lnv_dev_defaults(&d);
	d.verbose = 1;

	int rc = lnv_init(&d);
	out[LNV_SIM_RC_INIT] = (uint64_t)(int64_t)rc;
	if (rc)
		return rc;
	memcpy(sn, d.id.sn, sizeof(d.id.sn));
	memcpy(mn, d.id.mn, sizeof(d.id.mn));
	out[LNV_SIM_MPS]       = d.mps_bytes;
	out[LNV_SIM_MRRS]      = d.mrrs_bytes;
	out[LNV_SIM_NSZE]      = d.id.nsze;
	out[LNV_SIM_LBA_BYTES] = d.id.lba_bytes;

	rc = lnv_pattern_write(&d, lba, count, seed, 60000, &wst);
	out[LNV_SIM_RC_WRITE]     = (uint64_t)(int64_t)rc;
	out[LNV_SIM_WR_CYCLES]    = wst.cycles;
	out[LNV_SIM_WR_SUBMITTED] = wst.submitted;
	out[LNV_SIM_WR_COMPLETED] = wst.completed;
	out[LNV_SIM_WR_HWM]       = wst.slots_hwm;
	if (rc)
		return rc;

	memset(&res, 0, sizeof(res));
	rc = lnv_pattern_verify(&d, lba, count, seed, 60000, &res);
	out[LNV_SIM_RC_VERIFY]    = (uint64_t)(int64_t)rc;
	out[LNV_SIM_RD_CYCLES]    = res.rd.cycles;
	out[LNV_SIM_RD_SUBMITTED] = res.rd.submitted;
	out[LNV_SIM_RD_COMPLETED] = res.rd.completed;
	out[LNV_SIM_CHECKED]      = res.beats_checked;
	out[LNV_SIM_BAD]          = res.bad_beats;

	/* Negative check: the wrong seed must be reported as a mismatch on every beat. */
	memset(&bad, 0, sizeof(bad));
	out[LNV_SIM_RC_VERIFY_BADSEED] = (uint64_t)(int64_t)lnv_pattern_verify(&d, lba, count, seed + 1, 60000, &bad);
	out[LNV_SIM_BAD_BADSEED]       = bad.bad_beats;

	/* Range check is enforced before touching the drive. */
	out[LNV_SIM_RC_RANGE] = (uint64_t)(int64_t)lnv_stream(&d, 0, d.id.nsze - 1, 2, 1000, NULL);

	/* Re-run bring-up (controller reset, queues recreated at index 0) and read again. */
	rc = lnv_init(&d);
	out[LNV_SIM_RC_REINIT] = (uint64_t)(int64_t)rc;
	if (rc)
		return rc;
	memset(&res, 0, sizeof(res));
	out[LNV_SIM_RC_VERIFY_AFTER_REINIT] = (uint64_t)(int64_t)lnv_pattern_verify(&d, lba, count, seed, 60000, &res);
	return 0;
}
