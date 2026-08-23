# SPDX-License-Identifier: GPL-2.0-or-later

BIN2C = ../../../src/helper/bin2char.sh

ARM_CROSS_COMPILE ?= arm-none-eabi-
ARM_AS      ?= $(ARM_CROSS_COMPILE)as
ARM_OBJCOPY ?= $(ARM_CROSS_COMPILE)objcopy

ARM_AFLAGS = -EL

RISCV_CROSS_COMPILE ?= riscv64-unknown-elf-
RISCV_CC      ?= $(RISCV_CROSS_COMPILE)gcc
RISCV_OBJCOPY ?= $(RISCV_CROSS_COMPILE)objcopy
RISCV32_CFLAGS = -march=rv32e -mabi=ilp32e -nostdlib -nostartfiles -Os -fPIC
RISCV64_CFLAGS = -march=rv64i -mabi=lp64 -nostdlib -nostartfiles -Os -fPIC

TRICORE_CROSS_COMPILE ?= tricore-elf-
TRICORE_CC      ?= $(TRICORE_CROSS_COMPILE)gcc
TRICORE_OBJCOPY ?= $(TRICORE_CROSS_COMPILE)objcopy
# CRC32.B and CRC32L.W are TriCore 1.6.2 instructions.  The helper is linked
# flat at address zero because OpenOCD copies it into a work area at runtime.
TRICORE_AFLAGS = -mtc162 -nostdlib -nostartfiles -T tricore-flat.ld

.PHONY: all arm riscv tricore clean

all:	arm riscv tricore

arm: armv4_5_crc.inc armv7m_crc.inc

riscv:	riscv32_crc.inc riscv64_crc.inc

tricore: tricore.inc

armv4_5_%.elf: armv4_5_%.s
	$(ARM_AS) $(ARM_AFLAGS) $< -o $@

armv4_5_%.bin: armv4_5_%.elf
	$(ARM_OBJCOPY) -Obinary $< $@

armv7m_%.elf: armv7m_%.s
	$(ARM_AS) $(ARM_AFLAGS) $< -o $@

armv7m_%.bin: armv7m_%.elf
	$(ARM_OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

riscv32_%.elf:	riscv_%.c
	$(RISCV_CC) $(RISCV32_CFLAGS) $< -o $@

riscv64_%.elf:	riscv_%.c
	$(RISCV_CC) $(RISCV64_CFLAGS) $< -o $@

riscv%.bin:	riscv%.elf
	$(RISCV_OBJCOPY) -Obinary $< $@

tricore.elf:	tricore.s tricore-flat.ld
	$(TRICORE_CC) $(TRICORE_AFLAGS) $< -o $@

tricore.bin:	tricore.elf
	$(TRICORE_OBJCOPY) -Obinary $< $@

clean:
	-rm -f *.elf *.bin *.inc
