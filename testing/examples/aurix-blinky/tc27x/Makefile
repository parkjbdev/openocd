# SPDX-License-Identifier: GPL-2.0-or-later

# Cross toolchain prefix. Any TriCore ELF GCC will do, for example the
# HighTec / Infineon AURIX GCC or a self-built tricore-elf-gcc.
CROSS_COMPILE ?= tricore-elf-

CC      = $(CROSS_COMPILE)gcc
SIZE    = $(CROSS_COMPILE)size
OBJDUMP = $(CROSS_COMPILE)objdump

# When CROSS_COMPILE carries a directory, GCC still looks for its assembler
# and linker on PATH and picks up the host ones, which fail with
# "as: unrecognized option '-mtc161'". Point it at the same directory.
ifneq ($(dir $(CROSS_COMPILE)),./)
GCC_PREFIX = -B$(dir $(CROSS_COMPILE))
endif

TARGET  = tc27x-blinky
STAGE1  = tc27x-stage1

CFLAGS  = -mcpu=tc27xx -O1 -g -Wall -Wextra \
	  -ffunction-sections -fdata-sections -nostartfiles $(GCC_PREFIX)
LDFLAGS = -T tc27x.ld -nostartfiles -nostdlib \
	  -Wl,--gc-sections -Wl,-Map=$(TARGET).map

SRCS    = bmhd.S startup.S main.c

all: $(TARGET).elf
	$(SIZE) $<

# Generated Boot Mode Header, pulled in by bmhd.S's ".incbin". Without
# this, the BootROM only runs the flashed image while a debugger is
# attached; see gen-bmhd.py's header comment and tc27x.ld's comment.
bmhd0.bin: gen-bmhd.py
	python3 $< $@

$(TARGET).elf: $(SRCS) bmhd0.bin tc27x.ld
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -o $@

# Stage 1: no GPIO, no watchdog code, just a free-running counter in RAM.
# Used to empirically confirm (with zero risk) whether this family needs
# any watchdog disabled before an application can run, and to pin down
# the correct entry address. See startup.S / tc27x.ld's comments for
# what it found on TC275.
$(STAGE1).elf: startup-stage1.S main-stage1.c tc27x-stage1.ld
	$(CC) $(CFLAGS) -T tc27x-stage1.ld -nostartfiles -nostdlib \
	  -Wl,--gc-sections -Wl,-Map=$(STAGE1).map \
	  startup-stage1.S main-stage1.c -o $@

stage1: $(STAGE1).elf
	$(SIZE) $<

disasm: $(TARGET).elf
	$(OBJDUMP) -d -S $<

# Erase, program and verify with OpenOCD, then leave the board running.
flash: $(TARGET).elf
	openocd -f board/infineon/kit_a1g_tc275_lite.cfg \
		-c "program $(TARGET).elf verify reset exit"

clean:
	rm -f $(TARGET).elf $(TARGET).map $(STAGE1).elf $(STAGE1).map bmhd0.bin

.PHONY: all clean disasm flash stage1
