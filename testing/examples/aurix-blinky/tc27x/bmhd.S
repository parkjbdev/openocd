/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Wrap the generated Boot Mode Header (bmhd0.bin, see gen-bmhd.py) in its
 * own output section so the linker script can place it at the fixed
 * BMHD0 slot, 0xA0000000 (manual Table 5-2), immediately before the
 * .startup code at 0xA0000020.
 *
 * Without this header, the BootROM only runs the flashed image while a
 * debugger is attached and requests halt-after-reset; standalone it
 * falls back to Generic Bootstrap Loader mode instead, which does not
 * execute the application at all. See gen-bmhd.py's header comment for
 * the full manual citation and hardware verification.
 */

    .section .bmhd, "a", @progbits
    .incbin "bmhd0.bin"
