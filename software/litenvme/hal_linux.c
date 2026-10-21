/*
 * This file is part of LiteNVMe.
 *
 * Copyright (c) 2026 Krishna Swaroop <swaroopdhulipalla99@gmail.com>
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Linux userspace platform backend: maps the core's AXI-Lite aperture through /dev/mem (or a
 * UIO device node) and provides time/delay/log. For bare-metal (A53/R5/MicroBlaze), point
 * lnv_csr_base at the physical aperture and implement the same three functions on a timer.
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "litenvme_hal.h"
#include "hal_linux.h"

volatile uint8_t *lnv_csr_base;

static void  *map_base;
static size_t map_len;
static int    map_fd = -1;

int lnv_linux_map(const char *dev, uint64_t phys, size_t len)
{
	long page = sysconf(_SC_PAGESIZE);
	if (phys % (uint64_t)page) {
		fprintf(stderr, "litenvme: aperture 0x%llx not page aligned\n", (unsigned long long)phys);
		return -1;
	}
	map_fd = open(dev, O_RDWR | O_SYNC);
	if (map_fd < 0) {
		fprintf(stderr, "litenvme: open %s: %s\n", dev, strerror(errno));
		return -1;
	}
	/* /dev/mem: offset is the physical address; UIO: map index 0 is at offset 0. */
	off_t off = strncmp(dev, "/dev/uio", 8) == 0 ? 0 : (off_t)phys;
	map_base = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, map_fd, off);
	if (map_base == MAP_FAILED) {
		fprintf(stderr, "litenvme: mmap 0x%llx+0x%zx: %s\n", (unsigned long long)phys, len,
		        strerror(errno));
		close(map_fd);
		map_fd = -1;
		return -1;
	}
	map_len      = len;
	lnv_csr_base = (volatile uint8_t *)map_base;
	return 0;
}

void lnv_linux_unmap(void)
{
	if (map_base && map_base != MAP_FAILED)
		munmap(map_base, map_len);
	if (map_fd >= 0)
		close(map_fd);
	map_base = NULL;
	map_fd   = -1;
}

uint64_t lnv_time_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

void lnv_udelay(uint32_t us)
{
	usleep(us);
}

void lnv_log(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}
