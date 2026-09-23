/*
 * This file is part of LiteNVMe.
 *
 * Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Simulation platform backend: every CSR access, time query, delay and log line is forwarded to
 * callbacks installed by the Python testbench (test/test_system.py), which performs the access
 * as an AXI-Lite transaction on the simulated core. Built with -DLNV_HAL_EXTERN_ACCESSORS.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "litenvme_hal.h"
#include "litenvme.h"

typedef uint32_t (*lnv_sim_rd_fn)(uint32_t offset);
typedef void     (*lnv_sim_wr_fn)(uint32_t offset, uint32_t value);
typedef uint64_t (*lnv_sim_time_fn)(void);
typedef void     (*lnv_sim_delay_fn)(uint32_t us);
typedef void     (*lnv_sim_log_fn)(const char *line);

static lnv_sim_rd_fn    sim_rd;
static lnv_sim_wr_fn    sim_wr;
static lnv_sim_time_fn  sim_time;
static lnv_sim_delay_fn sim_delay;
static lnv_sim_log_fn   sim_log;

void lnv_sim_set_callbacks(lnv_sim_rd_fn rd, lnv_sim_wr_fn wr, lnv_sim_time_fn t,
                           lnv_sim_delay_fn delay, lnv_sim_log_fn log)
{
	sim_rd = rd; sim_wr = wr; sim_time = t; sim_delay = delay; sim_log = log;
}

uint32_t lnv_hal_rd32(unsigned long offset)          { return sim_rd((uint32_t)offset); }
void     lnv_hal_wr32(unsigned long offset, uint32_t v) { sim_wr((uint32_t)offset, v); }
uint64_t lnv_time_us(void)                           { return sim_time(); }
void     lnv_udelay(uint32_t us)                     { sim_delay(us); }

void lnv_log(const char *fmt, ...)
{
	char buf[512];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (sim_log)
		sim_log(buf);
}
