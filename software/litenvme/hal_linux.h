/*
 * This file is part of LiteNVMe.
 *
 * Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef LITENVME_HAL_LINUX_H
#define LITENVME_HAL_LINUX_H

#include <stddef.h>
#include <stdint.h>

/* Map `len` bytes of the core's AXI-Lite aperture at physical address `phys` via `dev`
 * ("/dev/mem" or "/dev/uioN"). Returns 0 on success. */
int  lnv_linux_map(const char *dev, uint64_t phys, size_t len);
void lnv_linux_unmap(void);

#endif /* LITENVME_HAL_LINUX_H */
