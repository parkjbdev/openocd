/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Minimal bare-metal reset handler for the Infineon AURIX TC275, stage 1.
 *
 * Same classic-AURIX TriCore boot sequence as TC375 (TC2x and TC3x share
 * the v1.6.2 core generation; TC4x is the one that differs, with v1.8):
 *  1. Reset vector fetched from 0xa0000000 (NC mirror of 0x80000000)
 *  2. Clear PCXI
 *  3. Initialise CSA (Context Save Area) free list in DSPR
 *  4. Set stack pointer (A10)
 *  5. Clear .bss
 *  6. Call main()
 *
 * Deliberately NO watchdog-disable call here (contrast with
 * tc375-blinky's startup.S, which calls early_safety_init() at this
 * point): the only TC2x watchdog register confirmed so far is the
 * safety watchdog's WDTS_CON0 at 0xf00360f0 (used by aurix_eflash.c's
 * ENDINIT toggle during flash erase/program, verified on silicon), and
 * that evidence is about brief, debugger-attached algorithm runs, not
 * about whether a free-running application also needs a per-CPU
 * watchdog serviced. Guessing at that would risk the same kind of
 * mistake TC375's own main.c warns about (a wrong SMU/watchdog write
 * can force a reset). This stage exists to answer empirically, with
 * zero risk, whether the TC275 Lite Kit needs any watchdog disabled at
 * all before a free-running loop can survive: main() just free-runs a
 * counter.
 */

    .section .startup, "ax", @progbits
    .globl  _START
    .type   _START, @function
    .align  2       /* 4-byte code alignment only: this must land exactly
                      * at 0xA0000020, the fixed TC2x "Internal Start"
                      * entry address (see tc27x-stage1.ld) -- a larger
                      * power-of-two alignment (e.g. .align 8 = 256 bytes)
                      * would push _START past that fixed address. */

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
