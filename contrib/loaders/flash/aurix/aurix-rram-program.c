// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2026 Infineon Technologies AG
 */

/*
 * RRAM programming helper for Infineon AURIX TC4X program and data RRAM.
 *
 * TC4X parts replace the eFlash program/data flash of earlier AURIX
 * families with resistive RAM (RRAM). Unlike eFlash, an RRAM write cannot
 * be driven by simple memory-mapped store instructions: the host has to
 * hand pages to the RRAM controller (PMUR/DMUR) through an INITWR/STWR
 * request handshake, feeding one word at a time into the WDATA FIFO.  The
 * register layout below (RRAM_UR_*) is undocumented and was recovered by
 * observing controller behaviour, not from a public register reference.
 *
 * Entry state, as set up by tc4x_rram.c:
 *   %a4  source buffer start (work area, ring buffer with an 8 byte header)
 *   %d4  source buffer size in bytes (including the header)
 *   %d5  RRAM destination address
 *   %d6  number of bytes to program
 *   %a5  RRAM controller (PMUR or DMUR) base address
 * Exit state:
 *   %d2  zero on success, error status otherwise
 *
 * The helper runs as an async flash algorithm and polls the work area's
 * write pointer for new data, so the host can stream a buffer much larger
 * than the work area while programming is in progress.  The host aborts
 * a transfer early by zeroing the write pointer; the helper detects this
 * and stops.  It stops permanently at the DEBUG instruction at the end of
 * the file.
 *
 * Built for two variants selected at compile time, matching the two RRAM
 * instances on TC4X: PMUR (program RRAM, 32 byte pages) and DMUR (data
 * RRAM, 8 byte pages, with two extra error classes only DMUR reports).
 * See tc4x-rram.h and the Makefile in this directory.
 */

#include <stdint.h>

#include "errors.h"
#include "tc4x-rram.h"
#include "tricore-mmio.h"

#define WRITE_TIMEOUT_TICKS (550ull * 500ull)

static inline uint32_t __attribute__((always_inline))
wait_done(uintptr_t rram)
{
	uint64_t start = mmio_read_u64(CPU0_STM_ABS_ADDR);
	const uint64_t stop = start + WRITE_TIMEOUT_TICKS;

	while (mmio_read_u64(CPU0_STM_ABS_ADDR) < stop) {
		uint32_t reqstat = mmio_read_u32(rram + RRAM_UR_REQSTAT);
		if ((reqstat & RRAM_REQSTAT_BUSY) == 0u && (reqstat & RRAM_REQSTAT_DONE))
			return reqstat & RRAM_REQSTAT_ERROR_MASK;
	}

	return ERROR_TIMEOUT;
}

static inline void __attribute__((always_inline))
clear_status(uintptr_t rram)
{
	mmio_write_u32(rram + RRAM_UR_CLRSTAT, RRAM_CLRSTAT_MASK);
	mmio_barrier();
}

/* rram_base must stay a pointer type so it is passed in an address register. */
int __attribute__((noreturn))
main(void *buffer_start, uint32_t buffer_size, uintptr_t addr, uint32_t size,
	 volatile void *const rram_base)
{
	register uint32_t ret __asm__("d2") = 0;
	const uintptr_t rram = (uintptr_t)rram_base;
	volatile uint32_t *wptr = (volatile uint32_t *)buffer_start;
	volatile uint32_t *rptr = (volatile uint32_t *)((uintptr_t)buffer_start + 4u);
	const uint32_t fifo_start = (uint32_t)((uintptr_t)buffer_start + 8u);
	const uint32_t fifo_end = (uint32_t)((uintptr_t)buffer_start + buffer_size);
	const uint32_t end = addr + size;

	uint32_t read_ptr = fifo_start;
	*rptr = read_ptr;

	while (addr < end) {
		clear_status(rram);
		mmio_write_u32(rram + RRAM_UR_REQUEST, RRAM_REQUEST_INITWR);
		mmio_barrier();
		mmio_write_u32(rram + RRAM_UR_NVMADDR, addr);
		mmio_barrier();

		uint32_t i;
		for (i = 0; i < PAGE_SIZE / 4u; i++) {
			/* Wait for data, the host clears the write pointer to abort */
			while (*wptr == read_ptr) {
				if (*wptr == 0u)
					goto out;
			}

			mmio_write_u32(rram + RRAM_UR_WDATA + i * 4u, mmio_read_u32(read_ptr));
			mmio_barrier();

			read_ptr += 4u;
			if (read_ptr >= fifo_end)
				read_ptr = fifo_start;
			*rptr = read_ptr;
		}

		mmio_write_u32(rram + RRAM_UR_REQUEST, RRAM_REQUEST_STWR);
		mmio_barrier();

		ret = wait_done(rram);
		if (ret) {
			*rptr = 0u;
			goto out;
		}

		addr += PAGE_SIZE;
	}

out:
	while (1)
		__asm__ volatile("debug" ::"d"(ret));

	__builtin_unreachable();
}
