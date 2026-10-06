/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Stage 1 bring-up link for the Infineon AURIX TC4D7 Lite Kit.
 *
 * Linked directly at 0x80000000 (the cached, definitely-mapped base of
 * flash bank pfls00), not at the 0xa0000000 non-cached alias TC375 uses.
 * The alias has been confirmed to mirror pfls00 byte for byte on a real
 * TC4D7 (see tc4dx.cfg's pfls00_nc comment), but whether the hardware
 * actually boots through it -- and whether any watchdog needs disabling
 * before .bss runs -- are still open questions this stage is meant to
 * answer empirically, so it deliberately does not depend on either: no
 * watchdog-disable call, plain increment-a-counter loop, no GPIO.
 */

ENTRY(_START)

MEMORY
{
    PFLASH (rx)  : ORIGIN = 0x80000000, LENGTH = 2M
    DSPR0  (rwx) : ORIGIN = 0x70000000, LENGTH = 32K
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
    .startup 0x80000000 :
    {
        KEEP(*(.startup))
    } > PFLASH :flash_code

    .text :
    {
        *(.text*)
        *(.rodata*)
    } > PFLASH :flash_code

    .data :
    {
        __data_start = .;
        *(.data*)
        __data_end = .;
    } > DSPR0 AT > PFLASH :ram_data

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
