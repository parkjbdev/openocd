/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Link the image at the TC375 application reset vector.
 *
 * The boot mode header in the UCB selects the start address. The factory
 * default is 0xa0000000, the base of the non-cached alias of program
 * flash bank 0, so the reset handler is placed there.
 *
 * The explicit PHDRS block keeps the ELF file headers out of the first
 * PT_LOAD segment. Without it the segment starts below the reset vector
 * and OpenOCD is asked to program bytes outside the flash bank.
 */

ENTRY(_START)

MEMORY
{
    PFLASH_NC (rx)  : ORIGIN = 0xA0000000, LENGTH = 4M
    DSPR0     (rwx) : ORIGIN = 0x70000000, LENGTH = 112K
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
    .startup 0xA0000000 :
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
