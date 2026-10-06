/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Link the image at the TC4D7 application reset vector.
 *
 * Confirmed on real TC4D7 hardware: a "reset halt" after flashing this
 * family's stage 1 probe (see startup-stage1.S)
 * lands at pc = 0xa0000000, exactly like TC375 -- TC4x boots through the
 * same kind of non-cached PFLASH alias. 0xa0000000 and 0x80000000 have
 * also been confirmed to read back byte-identical content, so this is a
 * real alias of pfls00, not a different physical bank; see tc4dx.cfg's
 * pfls00_nc comment. Flashing targets that alias through the pfls00_nc
 * virtual bank, the same pattern tc37x.cfg uses for pfls0_nc.
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
