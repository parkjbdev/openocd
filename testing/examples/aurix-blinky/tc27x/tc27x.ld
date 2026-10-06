/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Link the image at the TC27x "Internal Start" entry address, preceded
 * by a Boot Mode Header (BMHD) at the fixed BMHD0 slot.
 *
 * Confirmed on real TC275 hardware: unlike TC375/TC4D7 (which land
 * "reset halt" at exactly 0xa0000000, see tc375.ld/tc4d7.ld), TC2x's
 * stage 1 probe (see tc27x-stage1.ld / startup-stage1.S in this same
 * directory) only ran correctly once linked at 0xa0000020.
 *
 * Root cause, confirmed against the official Infineon TC21x/TC22x/TC23x
 * User's Manual (same BootROM generation as TC27x), chapter 5.1.6.3
 * "Select and Prepare Startup Modes": the BootROM's "Internal Start"
 * mode -- used either as a fallback with no valid BMHD, or (as
 * configured here, see gen-bmhd.py) selected directly by a valid BMHD's
 * BMI field -- says verbatim: "The User Start Address is set to
 * A000'0020H." This is a fixed, hardwired entry point for this mode,
 * architecturally different from TC3x/TC4x, which store their start
 * address in a separate UCB sector decoupled from the PFLASH code
 * region and so boot directly at 0xa0000000.
 *
 * This was empirically proven on real hardware: a stage 1 probe linked
 * at 0xa0000000 left D0 stuck at 0 through the whole CSA-init sequence
 * (i.e. none of the instructions before 0xa0000020 ever actually ran),
 * causing a NULL-pointer store trap. Re-linking the identical code at
 * 0xa0000020 fixed it immediately: the free-running counter in DSPR0
 * was observed incrementing across two reads 300ms apart.
 *
 * IMPORTANT, found the hard way: with no BMHD, this all still only ran
 * because a debugger was attached and requesting halt-after-reset
 * (which forces Internal Start regardless of header state). With the
 * debugger fully disconnected, the chip instead fell back to Generic
 * Bootstrap Loader mode -- which never executes the flashed image at
 * all -- so the board looked "dead" standalone despite the identical
 * bytes running correctly moments earlier while connected. The .bmhd
 * section below (see gen-bmhd.py / bmhd.S) fixes this: a valid BMHD
 * makes the header-evaluation procedure succeed and select Internal
 * Start directly, independent of debugger state. Hardware-verified:
 * byte-exact flash readback, a live register trace showing the blink
 * loop still executing, and -- the real test -- the board owner
 * visually confirming LED1/LED2 blinking after a full debugger
 * "shutdown" with nothing attached.
 */

ENTRY(_START)

MEMORY
{
    PFLASH_NC (rx)  : ORIGIN = 0xA0000000, LENGTH = 2M
    DSPR0     (rwx) : ORIGIN = 0x70000000, LENGTH = 32K
}

__CSA_SIZE   = 64 * 64;
__STACK_SIZE = 4096;

PHDRS
{
    flash_code PT_LOAD FLAGS(5);   /* R + X */
    ram_data   PT_LOAD FLAGS(6);   /* R + W */
}

SECTIONS
{
    .bmhd 0xA0000000 :
    {
        KEEP(*(.bmhd))
    } > PFLASH_NC :flash_code

    .startup 0xA0000020 :
    {
        KEEP(*(.startup))
    } > PFLASH_NC :flash_code

    .text :
    {
        *(.text*)
        *(.rodata*)
    } > PFLASH_NC :flash_code

    .data :
    {
        __data_start = .;
        *(.data*)
        __data_end = .;
    } > DSPR0 AT > PFLASH_NC :ram_data

    __data_load = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    } > DSPR0 :NONE

    .csa (NOLOAD) :
    {
        . = ALIGN(64);
        __CSA_BEGIN = .;
        . += __CSA_SIZE;
        __CSA_END = .;
    } > DSPR0 :NONE

    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        __STACK_BOTTOM = .;
        . += __STACK_SIZE;
        __STACK_TOP = .;
    } > DSPR0 :NONE
}
