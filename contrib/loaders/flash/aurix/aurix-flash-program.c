// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2026 Infineon Technologies AG
 */

/*
 * eFlash programming helper for Infineon AURIX TC2x/TC3x/TC4x program and
 * data flash (PFLASH/DFLASH, including UCB, which is programmed through
 * the DFLASH command sequence).
 *
 * The host cannot write eFlash directly: it feeds the destination address
 * and page-sized chunks of data through a command/status register
 * interface (HOST_CMD_ADDR and friends, see
 * tc2x-flash.h/tc3x-flash.h/tc4x-flash.h), one "enter page mode, stream
 * data, issue write" cycle per page. TC2x and TC3x each have a single such
 * interface; TC4x has two (host and CSRM/checker-core), and separate
 * pflash/dflash command codes and status bits besides.
 *
 * Entry state, as set up by aurix_eflash_write_algo():
 *   %a4  source buffer start (work area, ring buffer with an 8 byte header)
 *   %d4  source buffer size in bytes (including the header)
 *   %d5  flash destination address
 *   %d6  number of bytes to program
 * Exit state:
 *   %d2  zero on success, error status otherwise
 *
 * The helper runs as an async flash algorithm and polls the work area's
 * write pointer for new data, so the host can stream a buffer much larger
 * than the work area while programming is in progress.
 *
 * Built for eight variants selected at compile time: TC2x PFLASH/DFLASH,
 * TC3x PFLASH/DFLASH, and TC4x PFLASH/DFLASH/PFLASHCS/DFLASHCS (the "CS"
 * pair uses the second, CSRM-side command interface). See tc2x-flash.h,
 * tc3x-flash.h, tc4x-flash.h and the Makefile in this directory.
 *
 * TC2x and TC3x PFLASH writes are additionally protected by Safety ENDINIT
 * (a safety-critical register write-protection mechanism); tc2x_safety_
 * endinit()/tc3x_safety_endinit() below briefly lift it for the write
 * command sequence and restore it immediately after. DFLASH is not
 * ENDINIT-protected on either family, so it does not call either helper -
 * this is a genuine hardware asymmetry between the two, not an omission.
 * TC4x has no equivalent protection on either flash type.
 *
 * Restyled from the reference implementation published by Christoph Seitz
 * in tricore-oss-openocd (contrib/loaders/flash/aurix/), which replaced
 * this project's earlier tc3x-program.S/tc4x-program.S - hand written
 * assembly reconstructed from a pre-built loader blob shipped without
 * source (this project never had a TC2x loader of its own, hand written
 * or otherwise, so there was nothing to replace for that family - this is
 * simply new support). The command/status register offsets below were
 * independently confirmed against this project's own register-level
 * "fallback" write path in src/flash/nor/aurix_eflash.c, which predates
 * this file and was derived from Infineon's public register description
 * rather than from either loader; TC2x is not yet wired into that driver,
 * so only the TC3x/TC4x offsets could be cross-checked this way.
 */

#include <stdbool.h>
#include <stdint.h>

#include "errors.h"

#if defined(TC2X_FLASH_PFLASH) || defined(TC2X_FLASH_DFLASH)
#include "tc2x-flash.h"
#elif defined(TC3X_FLASH_PFLASH) || defined(TC3X_FLASH_DFLASH)
#include "tc3x-flash.h"
#elif defined(TC4X_FLASH_PFLASH) || defined(TC4X_FLASH_DFLASH) || \
	defined(TC4X_FLASH_PFLASHCS) || defined(TC4X_FLASH_DFLASHCS)
#include "tc4x-flash.h"
#else
#error "no flash bank selected, see tc2x-flash.h / tc3x-flash.h / tc4x-flash.h"
#endif

#if defined(TC2X_FLASH_PFLASH)
/*
 * Briefly lift (endinit=0) or restore (endinit=1) ENDINIT write protection
 * around the page write command sequence, using the documented two-step
 * password toggle on TC2X_WDTS_CON0. Returns 0 on success, or a synthetic
 * BIT(28) error code if the register never reflects the requested state.
 */
static inline uint32_t __attribute__((always_inline))
tc2x_safety_endinit(uint32_t endinit)
{
	const uint32_t value = mmio_read_u32(TC2X_WDTS_CON0_ADDR);

	mmio_write_u32(TC2X_WDTS_CON0_ADDR, (value & 0xFFFFFF00u) | (~value & 0xFCu) | 0x01u);
	mmio_write_u32(TC2X_WDTS_CON0_ADDR, (value & 0xFFFFFF00u) | (~value & 0xFCu) | 0x02u | endinit);

	for (uint32_t i = 0; i < 100000u; i++) {
		if ((mmio_read_u32(TC2X_WDTS_CON0_ADDR) & 1u) == endinit)
			return 0;
	}

	return BIT(28);
}
#endif

#if defined(TC3X_FLASH_PFLASH)
/*
 * Briefly lift (endinit=0) or restore (endinit=1) Safety ENDINIT write
 * protection around the page write command sequence. Same purpose as
 * tc2x_safety_endinit() above, but using the Password+Modify Access step
 * values (0xF1/0xF2) confirmed for TC3x's WDTSCON0 rather than TC2x's
 * (0x01/0x02) - see TC3X_ENDINIT_PW_STEP1/STEP2/SET in
 * src/flash/nor/aurix_eflash.c, which this mirrors. Required per
 * AURIX_TC3xx_Part1_UserManual.pdf section 6.3.1, "Safety Endinit
 * protection": "All command sequences that can modify PFLASH content are
 * protected by Safety Endinit. A PROER is generated if there is an access
 * to the PFLASH without removal of Safety Endinit protection." - a
 * requirement independent of, and in addition to, any watchdog
 * disable/service state. TC3x DFLASH is not ENDINIT-protected, same
 * asymmetry as TC2x.
 */
static inline uint32_t __attribute__((always_inline))
tc3x_safety_endinit(uint32_t endinit)
{
	const uint32_t pw = mmio_read_u32(TC3X_WDTS_CON0_ADDR) & 0xFFFFFF00u;

	mmio_write_u32(TC3X_WDTS_CON0_ADDR, pw | 0xF1u);
	mmio_write_u32(TC3X_WDTS_CON0_ADDR, pw | (endinit ? 0x03u : 0x00u) | 0xF2u);

	for (uint32_t i = 0; i < 100000u; i++) {
		if ((mmio_read_u32(TC3X_WDTS_CON0_ADDR) & 1u) == endinit)
			return 0;
	}

	return BIT(28);
}
#endif

static inline void __attribute__((always_inline))
clear_status(void)
{
	mmio_write_u32(HOST_CMD_ADDR + 0x5554u, 0xFAu);
	mmio_barrier();
}

static inline uint32_t __attribute__((always_inline))
enter_page_mode(void)
{
	mmio_write_u32(HOST_CMD_ADDR + 0x5554u, ENTER_PAGE_CMD);
	mmio_barrier();

	uint32_t timeout = 0;
	do {
		if (FLASH_PAGE_READY())
			return 0;

		uint32_t ret = FLASH_ERRSR();
		if (ret)
			return ret;
	} while (++timeout < 1000u);

	return ERROR_PAGE;
}

static inline uint32_t __attribute__((always_inline))
wait_write_complete(void)
{
	uint64_t start = flash_read_timebase();
	const uint64_t stop = start + WRITE_TIMEOUT_TICKS;

	while (flash_read_timebase() < stop) {
		uint32_t ret = FLASH_ERRSR();
		if (ret)
			return ret;

		if (FLASH_IDLE())
			return 0;
	}

	return ERROR_TIMEOUT;
}

int __attribute__((noreturn))
main(void *buffer_start, uint32_t buffer_size, uintptr_t addr, uint32_t size)
{
	uint32_t ret = 0;
	volatile uintptr_t *wptr = (volatile uintptr_t *)buffer_start;
	volatile uintptr_t *rptr = (volatile uintptr_t *)((uintptr_t)buffer_start + 4u);
	const uintptr_t fifo_end = (uintptr_t)buffer_start + buffer_size;

	*rptr = (uintptr_t)buffer_start + 8u;

	/* Disable interrupts */
	__asm__ volatile("disable");
	clear_status();
	if (FLASH_BUSY()) {
		ret = ERROR_BUSY;
		goto out_noclear;
	}
	FLASH_SET_PRMODE();

	while (size) {
		uintptr_t cur_rptr = *rptr;
		const bool burst_mode = (addr % BURST_SIZE == 0u) && (size >= BURST_SIZE);
		const uintptr_t copy_size = burst_mode ? BURST_SIZE : PAGE_SIZE;
		uint32_t i;

		/* Clear status before writing data */
		clear_status();

		/* Enter page mode */
		ret = enter_page_mode();
		if (ret)
			goto out;

#if defined(TC4X_FLASH_PFLASH) || defined(TC4X_FLASH_DFLASH) || \
	defined(TC4X_FLASH_PFLASHCS) || defined(TC4X_FLASH_DFLASHCS)
		/* Clear done bit before writing data */
		clear_status();
#endif

		for (i = 0; i < copy_size; i += 8u) {
			/* Wait until there is space in the FIFO */
			while (*wptr == cur_rptr)
				; /* spin */

			mmio_write_u64(HOST_CMD_ADDR + 0x55F0u, *(uint64_t *)cur_rptr);
			mmio_barrier();
			/* Check for sequence error */
			ret = FLASH_ERRSR();
			if (ret)
				goto out;

			cur_rptr += 8u;
			if (cur_rptr >= fifo_end)
				cur_rptr = (uintptr_t)buffer_start + 8u;
			*rptr = cur_rptr;
		}

#if defined(TC4X_FLASH_PFLASH) || defined(TC4X_FLASH_DFLASH) || \
	defined(TC4X_FLASH_PFLASHCS) || defined(TC4X_FLASH_DFLASHCS)
		/* Clear status before issuing write command */
		clear_status();
#endif

#if defined(TC2X_FLASH_PFLASH)
		/* PFLASH write commands are ENDINIT-protected on TC2x; DFLASH is not. */
		ret = tc2x_safety_endinit(0u);
		if (ret)
			goto out;
#elif defined(TC3X_FLASH_PFLASH)
		/* Same ENDINIT protection on TC3x PFLASH; DFLASH is not protected
		 * either. See tc3x_safety_endinit()'s comment. */
		ret = tc3x_safety_endinit(0u);
		if (ret)
			goto out;
#endif

		/* Issue page write command */
		mmio_write_u32(HOST_CMD_ADDR + 0xAA50u, (uint32_t)addr);
		mmio_write_u32(HOST_CMD_ADDR + 0xAA58u, 0u);
		mmio_write_u32(HOST_CMD_ADDR + 0xAAA8u, 0xA0u);
		mmio_write_u32(HOST_CMD_ADDR + 0xAAA8u, FLASH_WRITE_CMD(burst_mode));
		mmio_barrier();

#if defined(TC2X_FLASH_PFLASH)
		ret = tc2x_safety_endinit(1u);
		if (ret)
			goto out;
#elif defined(TC3X_FLASH_PFLASH)
		ret = tc3x_safety_endinit(1u);
		if (ret)
			goto out;
#endif

		if (size < copy_size) {
			ret = ERROR_PAGE;
			goto out;
		}
		size -= (uint32_t)copy_size;
		addr += copy_size;

		flash_post_write_delay();

		/* Wait for the write to complete */
		ret = wait_write_complete();
		if (ret)
			goto out;
	}

	goto out_noerr;

out:
	mmio_write_u32(HOST_CMD_ADDR + 0x5554u, 0xF0u);
	clear_status();
out_noclear:
	*rptr = 0u;
out_noerr:
	/* Report error code and notify debugger */
	{
		register uint32_t result __asm__("d2") = ret;
		while (1)
			__asm__ volatile("debug" ::"d"(result));
	}

	__builtin_unreachable();
}
