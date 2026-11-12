/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   TriCore Target Definitions for Infineon AURIX                         *
 *   Copyright (C) 2026 Infineon Technologies AG                           *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_TRICORE_TRICORE_H
#define OPENOCD_TARGET_TRICORE_TRICORE_H

#include "target/breakpoints.h"
#include "target/register.h"
#include "target/target.h"

#include "ocmts.h"

enum tricore_version {
	TRICORE_1_6,
	TRICORE_1_6_2,
	TRICORE_1_8,
};

enum tricore_fpu { TRICORE_FPU_NONE, TRICORE_FPU_SINGLE, TRICORE_FPU_DOUBLE };

enum tricore_event_type { TRICORE_EVENT_ADDRESS, TRICORE_EVENT_PC };
enum tricore_event_compare { TRICORE_EVENT_EQUALITY, TRICORE_EVENT_RANGE };
enum tricore_event_access { TRICORE_EVENT_STORE = 0x1, TRICORE_EVENT_LOAD = 0x2 };

struct tricore_event {
	bool enable;
	bool bbm;
	enum tricore_event_type type;
	enum tricore_event_compare compare;
	enum tricore_event_access access;

	uint32_t address;
};

enum tricore_debug_counter {
	TRICORE_DEBUG_COUNTER_NO_CHANGE,
	TRICORE_DEBUG_COUNTER_START,
	TRICORE_DEBUG_COUNTER_STOP,
	TRICORE_DEBUG_COUNTER_TOGGLE,
};

enum tricore_event_source {
	TRICORE_EVENT_EXTERNAL = 0,
	TRICORE_EVENT_CORE_REGISTER,
	TRICORE_EVENT_SOFTWARE,
	TRICORE_EVENT_TRIGGER0 = 16,
	TRICORE_EVENT_TRIGGER1,
	TRICORE_EVENT_TRIGGER2,
	TRICORE_EVENT_TRIGGER3,
	TRICORE_EVENT_TRIGGER4,
	TRICORE_EVENT_TRIGGER5,
	TRICORE_EVENT_TRIGGER6,
	TRICORE_EVENT_TRIGGER7,
};

#define TRICORE_NUM_EVENTS 8

/*
 * Idle encoding written to TRxEVT to disarm a trigger entry.
 *
 * Bits [2:0] (EVTA, the event action) are zero, which disables the trigger.
 * Bit 13 (RNG) is set to match the encoding the Infineon tools leave behind,
 * observed in DAP captures of an official debug session.
 */
#define TRICORE_EVENT_DISABLED 0x2000u

/* Milliseconds to wait for the core to halt after issuing a single step. */
#define TRICORE_STEP_TIMEOUT_MS 200

/* DBGSR.HALT - set while the core is halted (halted = 0x...03, running = 0x...01). */
#define TRICORE_DBGSR_HALT 0x2u
/* DBGSR.SUSP - core suspended (still running as far as OpenOCD is concerned). */
#define TRICORE_DBGSR_SUSP 0x10u

/* DBGSR write encodings: bit 2 enables the update of HALT in bit 1. */
#define TRICORE_DBGSR_HALT_SET   0x6u
#define TRICORE_DBGSR_HALT_CLEAR 0x4u

enum {
	TRICORE_D0 = 0,
	TRICORE_D1,
	TRICORE_D2,
	TRICORE_D3,
	TRICORE_D4,
	TRICORE_D5,
	TRICORE_D6,
	TRICORE_D7,
	TRICORE_D8,
	TRICORE_D9,
	TRICORE_D10,
	TRICORE_D11,
	TRICORE_D12,
	TRICORE_D13,
	TRICORE_D14,
	TRICORE_D15,
	TRICORE_A0,
	TRICORE_A1,
	TRICORE_A2,
	TRICORE_A3,
	TRICORE_A4,
	TRICORE_A5,
	TRICORE_A6,
	TRICORE_A7,
	TRICORE_A8,
	TRICORE_A9,
	TRICORE_A10,
	TRICORE_A11,
	TRICORE_A12,
	TRICORE_A13,
	TRICORE_A14,
	TRICORE_A15,
	TRICORE_PCXI,
	TRICORE_PSW,
	TRICORE_PC,
	TRICORE_ICR
};

struct tricore_info {
	struct ocmts *ocmts;

	enum tricore_version version;
	enum tricore_fpu fpu;
	struct tricore_event events[8];

	uint32_t dbgsr;
	uint32_t icr;
	void *regs;

	struct reg *pc;

	uint8_t active_vm;
	enum tricore_event_source active_event;

	bool has_dcache;
	bool has_icache;

	bool virt_enabled;
	bool suspended;
	bool halted;

	/*
	 * Boot halt (BOOTCON.BHALT on TC1.8, SYSCON.BHALT before): the core has
	 * not been started by CPU0's startup code yet. Such a core reads as
	 * halted until a debugger resumes it; from then on it reads as running
	 * (it will run as soon as it is started) until it really halts, or until
	 * a halt request takes it back.
	 */
	bool bhalt;
	bool bhalt_released;

	uint32_t *algorithm_context;
	unsigned int algorithm_context_regs;
	enum target_debug_reason algorithm_debug_reason;
};

struct tricore_reg {
	struct target *target;
	uint16_t addr;
	uint32_t value;
};

static inline struct tricore_info *target_to_tricore(struct target *target)
{
	return (struct tricore_info *)target->arch_info;
}

#endif /* OPENOCD_TARGET_TRICORE_TRICORE_H */
