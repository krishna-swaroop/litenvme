/*
 * This file is part of LiteNVMe.
 *
 * Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Platform layer for driving a generated LiteNVMe core (init_mode="external") from an external
 * processor: Zynq PS (Linux userspace or bare-metal), MicroBlaze, ...
 *
 * Include this header BEFORE <generated/csr.h>: it provides the CSR accessors the generated
 * header uses (CSR_ACCESSORS_DEFINED) and makes CSR_BASE the runtime base of the core's
 * AXI-Lite aperture, so every generated nvme_*_read()/write() works unchanged.
 */

#ifndef LITENVME_HAL_H
#define LITENVME_HAL_H

#include <stdint.h>

#define CSR_ACCESSORS_DEFINED

#ifdef LNV_HAL_EXTERN_ACCESSORS
/* Accesses go through platform functions taking the aperture offset (simulation, bridges). */
uint32_t lnv_hal_rd32(unsigned long offset);
void     lnv_hal_wr32(unsigned long offset, uint32_t v);

#define CSR_BASE 0UL

static inline void csr_write_simple(unsigned long v, unsigned long a)
{
	lnv_hal_wr32(a, (uint32_t)v);
}

static inline unsigned long csr_read_simple(unsigned long a)
{
	return lnv_hal_rd32(a);
}
#else
/* Virtual (or physical, on bare-metal) address of the core's csr_axil aperture. */
extern volatile uint8_t *lnv_csr_base;

#define CSR_BASE ((unsigned long)lnv_csr_base)

static inline void csr_write_simple(unsigned long v, unsigned long a)
{
	*(volatile uint32_t *)a = (uint32_t)v;
}

static inline unsigned long csr_read_simple(unsigned long a)
{
	return *(volatile uint32_t *)a;
}
#endif

/* Provided by the platform backend (hal_linux.c, or your bare-metal port). */
uint64_t lnv_time_us(void);          /* Monotonic microseconds. */
void     lnv_udelay(uint32_t us);
void     lnv_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif /* LITENVME_HAL_H */
