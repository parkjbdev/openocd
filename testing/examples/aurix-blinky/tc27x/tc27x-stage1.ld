/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Stage 1 bring-up link for the Infineon AURIX TC275 Lite Kit.
 *
 * TC2x's BootROM (SSW) is architecturally different from TC3x/TC4x here,
 * confirmed against the official Infineon TC21x/TC22x/TC23x User's Manual
 * (same BootROM generation as TC27x), chapter "5.1.6.3 Select and Prepare
 * Startup Modes":
 *
 *   "Internal Start - This is the basic TC23x type of operation in which
 *   the user code is started out of the Internal Flash Memory. The User
 *   Start Address is set to A000'0020H."
 *
 * This is the fallback mode the SSW takes when no valid Boot Mode Header
 * (BMHD) is found at 0xA0000000/0xA0020000/0xA000FFE0/0xA001FFE0 (all
 * erased/0xFF here, since none has ever been written) -- unlike TC3x/TC4x,
 * where the boot mode header lives in a separate UCB sector and can point
 * directly at 0xA0000000. Confirmed on real hardware: "reset halt" on this
 * TC275 consistently lands PC at exactly 0xa0000020 with D0 reading stale
 * (never executed the movh/addi that would have set it) -- i.e. the first
 * 0x20 bytes of this image are simply never fetched as instructions by the
 * SSW's Internal Start path. Linking the entry point directly at 0xA0000020
 * (instead of 0xA0000000) matches this fixed hardware behaviour.
 *
 * Whether any watchdog needs disabling before .bss runs is still an
 * open question this stage is meant to answer empirically, so it
 * deliberately does not depend on it: no watchdog-disable call, plain
 * increment-a-counter loop, no GPIO.
 */

ENTRY(_START)

MEMORY
{
    PFLASH_NC (rx)  : ORIGIN = 0xA0000020, LENGTH = 2M - 0x20
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
