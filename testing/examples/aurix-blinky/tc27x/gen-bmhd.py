#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Generate a TC2x Boot Mode Header 0 (BMHD0), the 32-byte structure the
# BootROM reads from 0xA0000000 before it will run any flashed code
# standalone (no debugger attached).
#
# Why this exists: confirmed on real TC275 hardware (Lite Kit, serial
# LK7U8NLP) that a flashed image with NO BMHD only ever runs while a
# debugger is attached and requests halt-after-reset (which forces the
# BootROM's "Internal Start" fallback, entry 0xA000'0020H). With no
# debugger, the official TC21x/TC22x/TC23x User's Manual, chapter 5.1.6.3
# "Select and Prepare Startup Modes", documents that the BootROM instead
# falls back to Generic Bootstrap Loader mode, which does not execute the
# flashed image at all -- this is why the board looked "dead" once the
# debugger was disconnected, even though the exact same flashed bytes ran
# fine while connected. A valid BMHD makes the header-evaluation procedure
# (manual chapter 5.1.4, Table 5-1/Figure 5-2) succeed, which selects the
# boot mode from BMI directly and skips that fallback, so the chip boots
# standalone every time, debugger or not.
#
# Layout (manual Table 5-1, byte offsets 00H..1FH):
#   00H STADABM   (4B) User Code Start Address
#   04H BMI       (2B) Boot Mode Index
#   06H BMHDID    (2B) must be B359H
#   08H ChkStart  (4B) ABM memory range check -- start address
#   0CH ChkEnd    (4B) ABM memory range check -- end address
#   10H CRCrange  (4B) CRC32 of [ChkStart..ChkEnd], only used if ABM selected
#   14H CRCrange~ (4B) bitwise inverse of the above
#   18H CRChead   (4B) CRC32 of bytes 00H..17H (the 6 words above)
#   1CH CRChead~  (4B) bitwise inverse of the above
#
# CRC algorithm (manual footnote under Table 5-1): "CRC calculation is
# based on IEEE 802.3, the CRC32 ethernet polynomial used is 04C11DB71H"
# -- i.e. the standard reflected Ethernet/zlib CRC32 (same one computed
# by Python's binascii.crc32/zlib.crc32), fed the header words as
# big-endian byte streams. This exact combination was found by brute
# force against a community-sourced worked BMHD example and then
# independently re-derived and cross-checked here against the real
# CRChead this script reproduces (0x6512F9D1) -- see the worked example
# below.
#
# This script's output was written to real TC275 PFLASH at 0xA0000000
# and hardware-verified three ways: (1) byte-exact readback after
# "flash write_image erase", (2) a live register trace showing the
# blink loop still executing correctly (P00_OUT toggling) across a
# "reset run", and (3) a full debugger "shutdown" followed by the
# board owner visually confirming LED1/LED2 blinking with NO debugger
# attached -- the actual acceptance test for this whole mechanism.
#
# Usage: gen-bmhd.py [output.bin]   (default: bmhd0.bin)

import struct
import sys
import zlib

# User Code Start Address. Unused in practice: this BMHD selects
# "Internal start from Flash" via BMI/HWCFG (see below), and that mode's
# entry address is unconditionally fixed at 0xA000'0020H by the BootROM
# regardless of STADABM (manual chapter 5.1.5.1). Set to the real entry
# address anyway for clarity/documentation; it is never read by hardware
# in this configuration.
STADABM = 0xA0000020

# Boot Mode Index. Bit layout (manual chapter 5.1.5.2, "BMI structure"):
#   bit 3      PINDIS      1 = ignore HWCFG pins, use BMI-selected mode
#                          unconditionally (we want deterministic boot,
#                          not pin-dependent)
#   bits 7:4   HWCFG[3:0]  111x = Internal start from Flash (lockstep
#                          bit HWCFG[0] = 0 = lockstep off, matching the
#                          existing non-lockstep blinky)
#   bits 9:8   lockstep    arbitrary / excluded from the header's own
#                          BMI validity check (manual Table 5-1 note)
# 0x0078 = 0b0000_0000_0111_1000: PINDIS=1, HWCFG[3:0]=1111 (reread as
# 0111 above the PINDIS bit -- verified against hardware: this exact
# value was flashed and the chip booted correctly standalone).
BMI = 0x0078

# Fixed confirmation code (manual Table 5-1): must be exactly this value
# or the whole header is rejected as absent/invalid.
BMHDID = 0xB359

# Alternate Boot Mode (ABM) range check is not used here (this BMHD
# selects a non-ABM internal-start mode at step 5 of the evaluation
# procedure, manual chapter 5.1.4), so these are inert/zero.
CHKSTART = 0x00000000
CHKEND = 0x00000000
CRCRANGE = 0x00000000


def crc32_be_words(words):
    """CRC32 (reflected Ethernet/zlib, poly 0x04C11DB7) over a list of
    32-bit words, each word fed big-endian -- the exact combination the
    BootROM uses for CRChead/CRCrange (see module docstring)."""
    data = b"".join(struct.pack(">I", w & 0xFFFFFFFF) for w in words)
    return zlib.crc32(data) & 0xFFFFFFFF


def build_bmhd():
    crcrange_inv = (~CRCRANGE) & 0xFFFFFFFF

    # CRChead covers offsets 00H..17H: STADABM, (BMHDID<<16 | BMI) as one
    # word, ChkStart, ChkEnd, CRCrange, CRCrange~ -- exactly the 6 words
    # Table 5-1 lists before the CRChead field itself.
    header_words = [
        STADABM,
        (BMHDID << 16) | BMI,
        CHKSTART,
        CHKEND,
        CRCRANGE,
        crcrange_inv,
    ]
    crchead = crc32_be_words(header_words)
    crchead_inv = (~crchead) & 0xFFFFFFFF

    return struct.pack(
        "<IHHIIIIII",
        STADABM,
        BMI,
        BMHDID,
        CHKSTART,
        CHKEND,
        CRCRANGE,
        crcrange_inv,
        crchead,
        crchead_inv,
    )


def self_test():
    """Worked-example regression check: this exact STADABM/BMI/BMHDID
    combination must always reproduce the CRChead value that was
    hardware-verified on real TC275 silicon (see module docstring)."""
    blob = build_bmhd()
    crchead = struct.unpack_from("<I", blob, 0x18)[0]
    assert crchead == 0x6512F9D1, (
        "BMHD generator regression: CRChead changed from the "
        "hardware-verified 0x6512F9D1 to 0x%08X -- did STADABM/BMI/"
        "BMHDID change? If intentional, re-verify on real hardware "
        "before trusting the new value." % crchead
    )


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "bmhd0.bin"
    self_test()
    blob = build_bmhd()
    assert len(blob) == 32, "BMHD0 must be exactly 32 bytes"
    with open(out_path, "wb") as f:
        f.write(blob)
    print(
        "%s: %d bytes, STADABM=0x%08X BMI=0x%04X BMHDID=0x%04X "
        "CRChead=0x%08X"
        % (
            out_path,
            len(blob),
            STADABM,
            BMI,
            BMHDID,
            struct.unpack_from("<I", blob, 0x18)[0],
        )
    )


if __name__ == "__main__":
    main()
