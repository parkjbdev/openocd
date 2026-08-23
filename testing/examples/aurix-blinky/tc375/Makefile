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

TARGET  = tc375-blinky

CFLAGS  = -mcpu=tc38xx -O1 -g -Wall -Wextra \
	  -ffunction-sections -fdata-sections -nostartfiles $(GCC_PREFIX)
LDFLAGS = -T tc375.ld -nostartfiles -nostdlib \
	  -Wl,--gc-sections -Wl,-Map=$(TARGET).map

SRCS    = startup.S main.c

all: $(TARGET).elf
	$(SIZE) $<

$(TARGET).elf: $(SRCS) tc375.ld
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -o $@

disasm: $(TARGET).elf
	$(OBJDUMP) -d -S $<

# Erase, program and verify with OpenOCD, then leave the board running.
flash: $(TARGET).elf
	openocd -f board/infineon/kit_a2g_tc375_lite.cfg \
		-c "program $(TARGET).elf verify reset exit"

clean:
	rm -f $(TARGET).elf $(TARGET).map

.PHONY: all clean disasm flash
