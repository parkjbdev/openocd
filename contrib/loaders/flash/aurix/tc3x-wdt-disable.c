// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2026 Infineon Technologies AG
 */

/*
 * On-target per-CPU watchdog Disabled Mode helper for Infineon AURIX TC3x.
 *
 * A per-CPU watchdog's WDTxCON0/WDTxCON1 registers can only be written by
 * code executing on that specific CPU: writes issued externally over JTAG
 * as plain memory accesses - the way tc3x_eflash_disable_one_wdt() in
 * aurix_eflash.c handles the Safety watchdog's identical registers from
 * the host side - are silently ignored by the hardware. Confirmed
 * experimentally on a TC375 Lite Kit: writing the exact same Password-then-
 * Modify Access sequence to WDTCPU0CON0 (0xF003624C) or WDTCPU1CON0
 * (0xF0036258) over JTAG never changes the register (every readback stays
 * at its entry value), while the identical sequence to WDTSCON0 (Safety,
 * 0xF00362A8) takes effect immediately.
 *
 * This helper performs the same five-step Disabled Mode sequence as
 * tc3x_eflash_disable_one_wdt(), but running as the target CPU's own code,
 * so it actually lands in Disabled Mode - confirmed by reading
 * WDTxSR.DS/TO back from the host afterwards (not WDTxCON0 - see that
 * function's comment for why WDTxCON0 alone is not a reliable indicator).
 *
 * Entry state, as set up by tc3x_eflash_disable_cpu_wdt_algo():
 *   %a4  WDTxCON0 address (e.g. the target CPU's own WDTCPUyCON0)
 *   %a5  WDTxCON1 address (matching WDTxCON1)
 * Exit state:
 *   %d2  always zero (reserved for future error reporting)
 */

#include <stdint.h>

#include "tricore-mmio.h"

/* Mirrors TC3X_ENDINIT_PW_STEP1/STEP2/SET and TC3X_WDT_DR in
 * src/flash/nor/aurix_eflash.c, which this sequence matches exactly. */
#define WDT_PW_STEP1	0xF1u
#define WDT_PW_STEP2	0xF2u
#define WDT_ENDINIT_SET	0x03u
#define WDT_DR		0x08u

int __attribute__((noreturn))
main(uint32_t *con0_addr, uint32_t *con1_addr)
{
	/* The low byte of WDTxCON0 holds the password/mode field driven
	 * below; the rest of the register (the running counter's compare
	 * values, etc.) must be preserved across every write. */
	const uint32_t pw = mmio_read_u32((uintptr_t)con0_addr) & 0xFFFFFF00u;

	/* Step 1: Password Access (unlock). Switches the watchdog to
	 * Time-Out Mode, starting the countdown to an SMU alarm if nothing
	 * below completes correctly. */
	mmio_write_u32((uintptr_t)con0_addr, pw | WDT_PW_STEP1);

	/* Step 2: Modify Access, ENDINIT=0. Required before WDTxCON1 becomes
	 * writable. */
	mmio_write_u32((uintptr_t)con0_addr, pw | WDT_PW_STEP2);

	/* Step 3: Request Disabled Mode. WDTxCON1 is gated solely by the
	 * corresponding WDTxCON0.ENDINIT being de-asserted, which step 2
	 * just did. */
	mmio_write_u32((uintptr_t)con1_addr, WDT_DR);

	/* Step 4/5: re-assert ENDINIT via a fresh Password+Modify Access.
	 * This is what commits WDTxCON1.DR=1 into WDTxSR.DS=1, i.e. what
	 * actually reaches Disabled Mode instead of leaving the counter
	 * running. */
	mmio_write_u32((uintptr_t)con0_addr, pw | WDT_PW_STEP1);
	mmio_write_u32((uintptr_t)con0_addr, pw | WDT_ENDINIT_SET | WDT_PW_STEP2);

	{
		register uint32_t result __asm__("d2") = 0;
		while (1)
			__asm__ volatile("debug" ::"d"(result));
	}

	__builtin_unreachable();
}
