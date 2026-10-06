/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Minimal bare-metal reset handler for the Infineon AURIX TC4D7, stage 1.
 *
 * Identical TriCore boot sequence to TC3x (same core architecture, just a
 * newer core version: v1.8 here vs v1.6.2 on TC375):
 *  1. Reset vector fetched from flash
 *  2. Clear PCXI
 *  3. Initialise CSA (Context Save Area) free list in DSPR
 *  4. Set stack pointer (A10)
 *  5. Clear .bss
 *  6. Call main()
 *
 * Deliberately NO watchdog-disable call here (contrast with tc375-blinky's
 * startup.S, which calls early_safety_init() at this point): TC4x has no
 * sourceable watchdog SFR in the public iLLD headers (see
 * tc4-work-2026/WATCHDOG.md), and blindly guessing at safety-critical
 * register addresses is exactly the kind of mistake that can brick a board
 * (TC3x's own main.c warns a wrong SMU write triggers a reset). This stage
 * exists to answer empirically, with zero risk, whether TC4D7's Lite Kit
 * even needs a watchdog disabled at all: main() just free-runs a counter.
 */

    .section .startup, "ax", @progbits
    .globl  _START
    .type   _START, @function
    .align  8

_START:
    disable
    isync

    /* ---------------------------------------------------------- */
    /* 1.  Clear PCXI                                             */
    /* ---------------------------------------------------------- */
    movh    %d0, 0
    mtcr    $pcxi, %d0
    isync

    /* ---------------------------------------------------------- */
    /* 2.  Initialise CSA free list (verbatim from crt0-tc2x.S)  */
    /* ---------------------------------------------------------- */
    movh    %d0, hi:__CSA_BEGIN
    addi    %d0, %d0, lo:__CSA_BEGIN
    addi    %d0, %d0, 63            /* force 64-byte alignment */
    andn    %d0, %d0, 63
    movh    %d2, hi:__CSA_END
    addi    %d2, %d2, lo:__CSA_END
    andn    %d2, %d2, 63
    sub     %d2, %d2, %d0
    sh      %d2, %d2, -6            /* number of CSAs */
    mov.a   %a3, %d0                /* %a3 = first CSA address */
    extr.u  %d0, %d0, 28, 4        /* segment bits [31:28] */
    sh      %d0, %d0, 16
    lea     %a4, 0                  /* previous = null */
    st.a    [%a3], %a4              /* store null in first CSA */
    mov.aa  %a4, %a3
    lea     %a3, [%a3] 64
    mov.d   %d1, %a3
    extr.u  %d1, %d1, 6, 16
    or      %d1, %d1, %d0
    mtcr    $lcx, %d1               /* LCX = second CSA */
    add     %d2, %d2, -2
    mov.a   %a5, %d2
csa_loop:
    mov.d   %d1, %a4
    extr.u  %d1, %d1, 6, 16
    or      %d1, %d1, %d0
    st.w    [%a3], %d1
    mov.aa  %a4, %a3
    lea     %a3, [%a3] 64
    loop    %a5, csa_loop
    mov.d   %d1, %a4
    extr.u  %d1, %d1, 6, 16
    or      %d1, %d1, %d0
    mtcr    $fcx, %d1               /* FCX = last filled CSA */
    isync

    /* ---------------------------------------------------------- */
    /* 3.  Stack pointer (A10 = SP on TriCore)                   */
    /* ---------------------------------------------------------- */
    movh.a  %sp, hi:__STACK_TOP
    lea     %sp, [%sp] lo:__STACK_TOP

    /* ---------------------------------------------------------- */
    /* 4.  Clear .bss                                            */
    /* ---------------------------------------------------------- */
    movh    %d0, hi:__bss_start
    addi    %d0, %d0, lo:__bss_start
    movh    %d1, hi:__bss_end
    addi    %d1, %d1, lo:__bss_end
    mov.a   %a2, %d0
    mov.a   %a3, %d1
    mov     %d0, 0
bss_clear:
    lt.a    %d2, %a2, %a3
    jz      %d2, bss_done
    st.w    [%a2+] 4, %d0
    j       bss_clear
bss_done:

    /* ---------------------------------------------------------- */
    /* 5.  Call main                                             */
    /* ---------------------------------------------------------- */
    call    main

spin:
    j spin

    .size _START, .-_START
