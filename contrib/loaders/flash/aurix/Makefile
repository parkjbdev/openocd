# SPDX-License-Identifier: GPL-2.0-or-later

BIN2C = ../../../../src/helper/bin2char.sh

TRICORE_CROSS_COMPILE ?= tricore-elf-
TRICORE_CC      ?= $(TRICORE_CROSS_COMPILE)gcc
TRICORE_OBJCOPY ?= $(TRICORE_CROSS_COMPILE)objcopy

# The helpers use TriCore 1.6.2 instructions.  They are linked flat at
# address zero because OpenOCD copies them into a work area at runtime.
TRICORE_CFLAGS  = -mtc162 -O3 -ffreestanding -nostdlib -fomit-frame-pointer \
		  -Wall -Wextra -fno-unroll-loops -nostartfiles -T tricore-flat.ld

all: tc2x-pflash-program.inc tc2x-dflash-program.inc \
	tc3x-pflash-program.inc tc3x-dflash-program.inc \
	tc4x-pflash-program.inc tc4x-dflash-program.inc \
	tc4x-pflashcs-program.inc tc4x-dflashcs-program.inc \
	tc4x-rram-pnvm-program.inc tc4x-rram-dnvm-program.inc \
	tc3x-wdt-disable.inc

.PHONY: all clean

# TC2x eFlash helper: same C source, built once per bank kind. TC2x has a
# single host command interface, so no CS (checker-core) variants exist.
tc2x-pflash-program.elf: aurix-flash-program.c tc2x-flash.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC2X_FLASH_PFLASH $< -o $@

tc2x-dflash-program.elf: aurix-flash-program.c tc2x-flash.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC2X_FLASH_DFLASH $< -o $@

# TC3x eFlash helper: same C source, built once per bank kind. TC3x has a
# single host command interface, so no CS (checker-core) variants exist.
tc3x-pflash-program.elf: aurix-flash-program.c tc3x-flash.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC3X_FLASH_PFLASH $< -o $@

tc3x-dflash-program.elf: aurix-flash-program.c tc3x-flash.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC3X_FLASH_DFLASH $< -o $@

# TC4x eFlash helper: same C source, built once per bank kind and command
# interface. The plain pflash/dflash variants use the host interface
# (module 0); the "cs" variants use the second, CSRM/checker-core
# interface (module 1).
tc4x-pflash-program.elf: aurix-flash-program.c tc4x-flash.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC4X_FLASH_PFLASH $< -o $@

tc4x-dflash-program.elf: aurix-flash-program.c tc4x-flash.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC4X_FLASH_DFLASH $< -o $@

tc4x-pflashcs-program.elf: aurix-flash-program.c tc4x-flash.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC4X_FLASH_PFLASHCS $< -o $@

tc4x-dflashcs-program.elf: aurix-flash-program.c tc4x-flash.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC4X_FLASH_DFLASHCS $< -o $@

# TC4x RRAM (PMUR/DMUR) helper: same C source, built once per instance since
# page size and error bits differ between program and data RRAM.
tc4x-rram-pnvm-program.elf: aurix-rram-program.c tc4x-rram.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC4X_RRAM_PNVM $< -o $@

tc4x-rram-dnvm-program.elf: aurix-rram-program.c tc4x-rram.h errors.h tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) -DTC4X_RRAM_DNVM $< -o $@

# Per-CPU watchdog Disabled Mode helper: plain C like the other loaders
# above, but note it still has to run as the target CPU's own code rather
# than through plain host-side register writes (see tc3x-wdt-disable.c for
# why). Only TC3x is wired up on the host side so far; see aurix_eflash.c.
tc3x-wdt-disable.elf: tc3x-wdt-disable.c tricore-mmio.h tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_CFLAGS) $< -o $@

%.bin: %.elf
	$(TRICORE_OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.bin
