# LiteNVMe host library + litenvme_test for Linux on the Zynq PS (or any Linux host with the
# core mapped). GEN_DIR is the generator's software/include directory for YOUR core build:
#
#   make GEN_DIR=../../build/zu17eg_ssd0/software/include CROSS_COMPILE=aarch64-linux-gnu-
#
# For bare-metal, build litenvme.c with your toolchain plus a platform file providing
# lnv_csr_base, lnv_time_us(), lnv_udelay() and lnv_log() (see litenvme_hal.h).

CROSS_COMPILE ?=
CC            := $(CROSS_COMPILE)gcc
GEN_DIR       ?= ../../build/zu17eg_ssd0/software/include
CFLAGS        ?= -O2
CFLAGS        += -std=gnu11 -Wall -Wextra -Werror -Iinclude -I$(GEN_DIR) -I.

all: litenvme_test

litenvme_test: litenvme_test.c litenvme.c hal_linux.c litenvme.h litenvme_hal.h hal_linux.h \
               $(GEN_DIR)/generated/csr.h $(GEN_DIR)/generated/soc.h
	$(CC) $(CFLAGS) -o $@ litenvme_test.c litenvme.c hal_linux.c

clean:
	rm -f litenvme_test

.PHONY: all clean
