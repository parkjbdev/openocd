# SPDX-License-Identifier: GPL-2.0-or-later

# Cross toolchain prefix. Any TriCore ELF GCC will do, for example the
# HighTec / Infineon AURIX GCC or a self-built tricore-elf-gcc.
CROSS_COMPILE ?= tricore-elf-

CC      = $(CROSS_COMPILE)gcc
SIZE    = $(CROSS_COMPILE)size
OBJDUMP = $(CROSS_COMPILE)objdump

# When CROSS_COMPILE carries a directory, GCC still looks for its assembler
# and linker on PATH and picks up the host ones, which fail with
# "as: unrecognized option '-mtc162'". Point it at the same directory.
ifneq ($(dir $(CROSS_COMPILE)),./)
GCC_PREFIX = -B$(dir $(CROSS_COMPILE))
endif

TARGET  = tc4d7-blinky
STAGE1  = tc4d7-stage1

CFLAGS  = -mcpu=tc4DAx -O1 -g -Wall -Wextra \
	  -ffunction-sections -fdata-sections -nostartfiles $(GCC_PREFIX)
LDFLAGS = -T tc4d7.ld -nostartfiles -nostdlib \
	  -Wl,--gc-sections -Wl,-Map=$(TARGET).map

SRCS    = startup.S main.c

all: $(TARGET).elf
	$(SIZE) $<

$(TARGET).elf: $(SRCS) tc4d7.ld
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -o $@

# Stage 1: no GPIO, no watchdog code, just a free-running counter in RAM.
# Used to empirically confirm (with zero risk) whether this family needs
# any watchdog disabled before an application can run. See startup.S's
# comment for what it found on TC4D7.
$(STAGE1).elf: startup-stage1.S main-stage1.c tc4d7-stage1.ld
	$(CC) $(CFLAGS) -T tc4d7-stage1.ld -nostartfiles -nostdlib \
	  -Wl,--gc-sections -Wl,-Map=$(STAGE1).map \
	  startup-stage1.S main-stage1.c -o $@

stage1: $(STAGE1).elf
	$(SIZE) $<

disasm: $(TARGET).elf
	$(OBJDUMP) -d -S $<

# Erase, program and verify with OpenOCD, then leave the board running.
flash: $(TARGET).elf
	openocd -f board/infineon/kit_a3g_tc4d7_lite.cfg \
		-c "program $(TARGET).elf verify reset exit"

clean:
	rm -f $(TARGET).elf $(TARGET).map $(STAGE1).elf $(STAGE1).map

.PHONY: all clean disasm flash stage1
