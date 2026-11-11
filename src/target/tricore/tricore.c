// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   TriCore Target Support for Infineon AURIX                             *
 *   Copyright (C) 2026 Infineon Technologies AG                           *
 ***************************************************************************/

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "helper/binarybuffer.h"
#include "helper/time_support.h"
#include "helper/command.h"
#include "helper/log.h"
#include "helper/types.h"
#include "jtag/interface.h"
#include "jtag/jtag.h"
#include "target/algorithm.h"
#include "target/breakpoints.h"
#include "target/register.h"
#include "target/target.h"
#include "target/target_type.h"

#include "aurix.h"
#include "ocmts.h"
#include "tricore.h"
#include "tricore_register.h"

struct tricore_private_config {
	struct ocmts *ocmts;
};

/* Forward declarations */
static int tricore_event_set(struct target *target, uint8_t event_id);
int tricore_set_breakpoint(struct target *target, struct breakpoint *breakpoint, bool restore);
int tricore_unset_breakpoint(struct target *target, struct breakpoint *breakpoint);

static const struct {
	const char *const name;
	uint16_t addr;
} tricore_core_regs[] = {
	{.name = "d0", .addr = 0xFF00},
	{.name = "d1", .addr = 0xFF04},
	{.name = "d2", .addr = 0xFF08},
	{.name = "d3", .addr = 0xFF0C},
	{.name = "d4", .addr = 0xFF10},
	{.name = "d5", .addr = 0xFF14},
	{.name = "d6", .addr = 0xFF18},
	{.name = "d7", .addr = 0xFF1C},
	{.name = "d8", .addr = 0xFF20},
	{.name = "d9", .addr = 0xFF24},
	{.name = "d10", .addr = 0xFF28},
	{.name = "d11", .addr = 0xFF2C},
	{.name = "d12", .addr = 0xFF30},
	{.name = "d13", .addr = 0xFF34},
	{.name = "d14", .addr = 0xFF38},
	{.name = "d15", .addr = 0xFF3C},
	{.name = "a0", .addr = 0xFF80},
	{.name = "a1", .addr = 0xFF84},
	{.name = "a2", .addr = 0xFF88},
	{.name = "a3", .addr = 0xFF8C},
	{.name = "a4", .addr = 0xFF90},
	{.name = "a5", .addr = 0xFF94},
	{.name = "a6", .addr = 0xFF98},
	{.name = "a7", .addr = 0xFF9C},
	{.name = "a8", .addr = 0xFFA0},
	{.name = "a9", .addr = 0xFFA4},
	{.name = "a10", .addr = 0xFFA8},
	{.name = "a11", .addr = 0xFFAC},
	{.name = "a12", .addr = 0xFFB0},
	{.name = "a13", .addr = 0xFFB4},
	{.name = "a14", .addr = 0xFFB8},
	{.name = "a15", .addr = 0xFFBC},
	/* LCX/FCX must precede pcx/psw/pc/icr here so that "pc" lands at raw
	 * register index 36 (32 D/A regs + LCX + FCX + pcx + psw = indices
	 * 32-35, pc = 36), matching the fixed raw-register numbering GDB's
	 * TriCore target description expects (GDB requests reg_num=36 for
	 * PC regardless of the XML feature's register order/names - this
	 * was previously missing LCX/FCX, shifting PC down to index 34 and
	 * causing "gdb requested a non-existing register (reg_num=36)" and
	 * an immediate dropped connection on every gdb attach). Order/addr
	 * cross-checked against the known-working DAS-based aurix-openocd
	 * driver's register table, which is src/target/tricore/aurix.c in the
	 * external tricore-oss tree at [1], not in this one. */
	{.name = "lcx", .addr = 0xFE3C},
	{.name = "fcx", .addr = 0xFE38},
	{.name = "pcx", .addr = 0xFE00},
	{.name = "psw", .addr = 0xFE04},
	{.name = "pc", .addr = 0xFE08},
	{.name = "icr", .addr = 0xFE2C},
};

static struct reg_feature tricore_core_feature = {
	.name = "org.gnu.gdb.tricore.core",
};

static int tricore_build_reg_cache(struct target *target, const struct reg_arch_type *type)
{
	const int num_regs = ARRAY_SIZE(tricore_core_regs);
	struct tricore_info *tricore = target_to_tricore(target);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct tricore_reg *reg_arch_info = calloc(num_regs, sizeof(struct tricore_reg));
	int i;

	if (!cache || !reg_list || !reg_arch_info) {
		free(cache);
		free(reg_list);
		free(reg_arch_info);
		target->reg_cache = NULL;
		return ERROR_FAIL;
	}
	target->reg_cache = cache;

	cache->name = "TriCore registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = 0;

	for (i = 0; i < num_regs; i++) {
		reg_arch_info[i].addr = tricore_core_regs[i].addr;
		reg_arch_info[i].target = target;

		reg_list[i].name = tricore_core_regs[i].name;
		reg_list[i].number = i;
		reg_list[i].size = 32;
		reg_list[i].value = (uint8_t *)&reg_arch_info[i].value;
		reg_list[i].type = type;
		reg_list[i].arch_info = &reg_arch_info[i];
		reg_list[i].exist = true;

		/* Registers data type, as used by GDB target description */
		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (!reg_list[i].reg_data_type) {
			for (int j = 0; j < i; j++)
				free(reg_list[j].reg_data_type);
			free(reg_arch_info);
			free(reg_list);
			free(cache);
			target->reg_cache = NULL;
			return ERROR_FAIL;
		}
		if (i < 16)
			reg_list[i].reg_data_type->type = REG_TYPE_INT;
		else if (i < 32)
			reg_list[i].reg_data_type->type = REG_TYPE_DATA_PTR;
		else if (i == 36)
			reg_list[i].reg_data_type->type = REG_TYPE_CODE_PTR;
		else
			reg_list[i].reg_data_type->type = REG_TYPE_UINT32;

		reg_list[i].feature = &tricore_core_feature;
		reg_list[i].group = "general";

		cache->num_regs++;
	}

	tricore->pc = &reg_list[36];

	return ERROR_OK;
}

static void tricore_free_reg_cache(struct target *target)
{
	/* the cache is only built in init_target(), which does not run if
	 * OpenOCD shuts down before reaching the run stage */
	if (!target->reg_cache)
		return;

	struct reg *reg_list = target->reg_cache->reg_list;

	for (unsigned int i = 0; i < target->reg_cache->num_regs; i++)
		free(reg_list[i].reg_data_type);

	free(reg_list->arch_info);
	free(reg_list);
	free(target->reg_cache);
	target->reg_cache = NULL;
}

static inline uint32_t tricore_get_reg_addr(struct target *target, uint16_t addr)
{
	struct tricore_info *tricore = target_to_tricore(target);
	if (tricore->version == TRICORE_1_8) {
		if ((addr & 0xFF00) == 0xFE00 && (addr < TRICORE_BIV || addr > TRICORE_LCX) && addr != TRICORE_PPRS) {
			/* Register are not replicated. Read HRA*/
			return target->dbgbase + 0x10000 + addr;
		}
		if ((addr & 0xFF00) == 0xFD00 || (addr & 0xFF00) == 0xF000) {
			/* Register are not replicated. Read HRA*/
			return target->dbgbase + 0x10000 + addr;
		}
		if (addr >= TRICORE_VCON0 && addr <= TRICORE_BHV)
			return target->dbgbase + 0x30000 + addr;
		if (tricore->virt_enabled) {
			switch (tricore->active_vm) {
			case 0:
				return target->dbgbase + 0x30000 + addr;
			case 1:
				return target->dbgbase + 0x10000 + addr;
			default:
				return target->dbgbase + 0x20000 + addr;
			}
		}
		return target->dbgbase + 0x10000 + addr;
	}

	return target->dbgbase + 0x10000 + addr;
}

static int tricore_restore_reg_cache(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret;
	unsigned int i;

	for (i = 0; i < target->reg_cache->num_regs; i++) {
		struct reg *reg = &target->reg_cache->reg_list[i];
		struct tricore_reg *arch_reg = (struct tricore_reg *)reg->arch_info;

		if (reg->dirty) {
			ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, arch_reg->addr), arch_reg->value);
			if (ret) {
				LOG_TARGET_ERROR(target, "Failed to write core reg 0x%04x", arch_reg->addr);
				return ret;
			}
			reg->dirty = false;
		}
	}

	return ERROR_OK;
}

int tricore_reg_get(struct reg *reg)
{
	struct tricore_reg *arch_reg = (struct tricore_reg *)reg->arch_info;
	struct target *target = arch_reg->target;
	struct tricore_info *tricore = target_to_tricore(target);
	int ret = 0;

	if (reg->valid)
		return ERROR_OK;

	ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, arch_reg->addr), &arch_reg->value);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to read core reg 0x%04x", arch_reg->addr);
		return ret;
	}

	/* If target is unhalted all register reads should be uncached. */
	if (target->state == TARGET_HALTED)
		reg->valid = true;
	else
		reg->valid = false;

	reg->dirty = false;

	return ERROR_OK;
}

int tricore_reg_set(struct reg *reg, uint8_t *buf)
{
	struct tricore_reg *arch_reg = (struct tricore_reg *)reg->arch_info;
	struct target *target = arch_reg->target;
	uint32_t value = target_buffer_get_u32(target, buf);

	target_buffer_set_u32(target, reg->value, value);

	reg->valid = true;
	reg->dirty = true;

	return ERROR_OK;
}

static const struct reg_arch_type
	__attribute__((used)) tricore_reg_type = {.get = tricore_reg_get, .set = tricore_reg_set};

static inline void tricore_examin_debug_reason(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	tricore->halted = (tricore->dbgsr & TRICORE_DBGSR_HALT) != 0;
	/*
	 * EVTSRC is five bits wide [12:8]. Masking with 0xF truncated the trigger
	 * sources (TRICORE_EVENT_TRIGGER0 == 16) down into the external/core-register
	 * range, so every hardware breakpoint hit was reported to GDB as DBG_REASON_DBGRQ
	 * (SIGINT) instead of DBG_REASON_BREAKPOINT (SIGTRAP).
	 */
	tricore->active_event = (tricore->dbgsr >> 8) & 0x1F;
	tricore->suspended = (tricore->dbgsr & TRICORE_DBGSR_SUSP) != 0;

	LOG_TARGET_DEBUG(target, "DBGSR halt: %d suspended: %d event: %d", tricore->halted, tricore->suspended,
					 tricore->active_event);
	if (target->debug_reason == DBG_REASON_SINGLESTEP)
		return;

	/* Suspended in halt counts as an external debug request. */
	if (tricore->suspended)
		target->debug_reason = DBG_REASON_DBGRQ;

	switch (tricore->active_event) {
	case TRICORE_EVENT_EXTERNAL:
	case TRICORE_EVENT_CORE_REGISTER:
		target->debug_reason = DBG_REASON_DBGRQ;
		break;
	case TRICORE_EVENT_SOFTWARE:
		target->debug_reason = DBG_REASON_BREAKPOINT;
		break;
	case TRICORE_EVENT_TRIGGER0:
	case TRICORE_EVENT_TRIGGER1:
	case TRICORE_EVENT_TRIGGER2:
	case TRICORE_EVENT_TRIGGER3:
	case TRICORE_EVENT_TRIGGER4:
	case TRICORE_EVENT_TRIGGER5:
	case TRICORE_EVENT_TRIGGER6:
	case TRICORE_EVENT_TRIGGER7:
		if (tricore->events[tricore->active_event - TRICORE_EVENT_TRIGGER0].type == TRICORE_EVENT_ADDRESS)
			target->debug_reason = DBG_REASON_WATCHPOINT;
		else
			target->debug_reason = DBG_REASON_BREAKPOINT;
		break;
	}
}

static int tricore_debug_entry(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	uint32_t vcon0 = 0, vcon1 = 0;
	int ret = 0;

	/*
	 * Virtualization can be switched off while the core runs: the AURIX
	 * startup code clears VCON0.VEN right after reset. VCON0/VCON1 then stop
	 * answering, so trusting the virt_enabled seen at examine time turns the
	 * first halt after that into a failed status read. TCCON bit 3 tracks
	 * VEN and is always readable through the HRA aperture.
	 */
	if (tricore->version == TRICORE_1_8) {
		uint32_t tccon;

		ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TCCON), &tccon);
		if (ret == ERROR_OK)
			tricore->virt_enabled = !!(tccon & 0x8);
	}

	if (ret == ERROR_OK)
		ret = ocmts_queue_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_DBGSR), &tricore->dbgsr);
	if (ret == ERROR_OK)
		ret = ocmts_queue_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_ICR), &tricore->icr);
	if (ret == ERROR_OK && tricore->version == TRICORE_1_8 && tricore->virt_enabled) {
		ret = ocmts_queue_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_VCON0), &vcon0);
		if (ret == ERROR_OK)
			ret = ocmts_queue_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_VCON1), &vcon1);
	}
	if (ret == ERROR_OK)
		ret = ocmts_run(tricore->ocmts);
	if (ret) {
		LOG_TARGET_ERROR(target, "failed to read status registers");
		return ERROR_FAIL;
	}

	tricore_examin_debug_reason(target);

	if (tricore->version == TRICORE_1_8) {
		/* VCON0/VCON1 are only read while virtualization is known to be
		 * active; they default to 0 otherwise, which selects the
		 * non-virtualized defaults below. */
		if (vcon0 & 0x1) {
			tricore->virt_enabled = true;
			tricore->active_vm = vcon1 & 0xF;
		} else {
			tricore->virt_enabled = false;
			/* Virt 1 (HRA) is the default*/
			tricore->active_vm = 1;
		}
	}

	return ERROR_OK;
}

static int tricore_init_debug_access(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret = 0;

	if (tricore->version == TRICORE_1_8) {
		uint32_t tccon;
		ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TCCON), &tccon);
		if (ret)
			return ret;
		tricore->virt_enabled = !!(tccon & 0x8);

		/* Enable core debug */
		uint32_t virt_dbg_en = tricore->virt_enabled ? 0xFF0000 : 0;
		ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_DBGCFG), virt_dbg_en | 0x8003);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to enable debug");
			return ret;
		}
	}

	/* Set debug instruction event trigger */
	ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_SWEVT),
							 tricore->version == TRICORE_1_8 ? 0x9 : 0xA);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to set SW event");
		return ret;
	}

	/*
	 * Disarm every hardware trigger entry.
	 *
	 * The OCDS implements "halt after reset" by arming trigger 0 on the boot
	 * address (TR0EVT = TYP_PC | BBM | EVTA_HALT, TR0ADR = BOOT_ADDR). That
	 * trigger survives into the debug session, so any attempt to resume makes
	 * the core halt again the instant it fetches from the reset vector - which
	 * looks exactly like "resume does nothing".
	 *
	 * OpenOCD tracks breakpoints in tricore->events[], all of which start
	 * disabled, so clearing the hardware here also resynchronises the two.
	 * Entries are cleared to the same idle encoding the Infineon tools use
	 * (EVT = TRICORE_EVENT_DISABLED, ADR = 0xFFFFFFFF); block writes are not
	 * used because the CSFR aperture only accepts single-word accesses.
	 */
	for (uint8_t i = 0; i < TRICORE_NUM_EVENTS; i++) {
		ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXEVT(i)),
								 TRICORE_EVENT_DISABLED);
		if (ret == ERROR_OK)
			ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXADR(i)),
									 0xFFFFFFFF);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to disarm hardware trigger %u", i);
			return ret;
		}
		tricore->events[i].enable = false;
	}

	return ERROR_OK;
}

static int tricore_poll(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	enum target_state prev_target_state = target->state;
	int ret = ERROR_OK;
	bool bhalt = false;

	if (tricore->version == TRICORE_1_8) {
		uint32_t bootcon;
		ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_BOOTCON), &bootcon);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to read BOOTCON register");
			return ret;
		}
		bhalt = (bootcon & 0x1) != 0;
	} else {
		uint32_t syscon;
		ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_SYSCON), &syscon);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to read SYSCON register");
			return ret;
		}
		bhalt = (syscon & 0x1000000) != 0;
	}

	ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_DBGSR), &tricore->dbgsr);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to read DBGSR register");
		return ret;
	}

	/*
	 * DBGSR.HALT (bit 1) is the halted indicator: an Infineon-tool reference
	 * session reads 0x1003 while halted and 0x1001 while running. Bit 4 (SUSP)
	 * must NOT be treated as "halted" - a suspended core is still running from
	 * OpenOCD's point of view, and testing it made every resume look like it
	 * had failed.
	 */
	if ((tricore->dbgsr & TRICORE_DBGSR_HALT) || bhalt) {
		LOG_TARGET_DEBUG(target, "Target is halted DGBSR: 0x%08x BHALT: %d", tricore->dbgsr, bhalt);
		target->state = TARGET_HALTED;

		if (prev_target_state != TARGET_HALTED) {
			ret = tricore_debug_entry(target);
			if (ret) {
				LOG_TARGET_ERROR(target, "Failed to read debug status");
				return ret;
			}

			if (prev_target_state == TARGET_DEBUG_RUNNING)
				ret = target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			else
				ret = target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			if (ret)
				return ret;
		}
	} else {
		target->state = TARGET_RUNNING;
	}

	return ret;
}

/* Invoked only from target_arch_state().
 * Issue USER() w/architecture specific status.  */
int tricore_arch_state(struct target *target)
{
	return ERROR_FAIL;
}

/* target request support */
int tricore_target_request_data(struct target *target, uint32_t size, uint8_t *buffer)
{
	return ERROR_FAIL;
}

int tricore_halt(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret = 0;

	if (target->state == TARGET_HALTED) {
		LOG_TARGET_DEBUG(target, "target already halted");
		return ERROR_OK;
	}

	/*
	 * Halt by writing DBGSR.HALT. The adapter must not intercept this: an
	 * earlier miniWiggler workaround implemented "resume" as a hardware reset,
	 * which silently destroyed target state on every continue.
	 */
	ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_DBGSR),
							 TRICORE_DBGSR_HALT_SET);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to halt target");
		return ret;
	}

	target->debug_reason = DBG_REASON_DBGRQ;

	return ret;
}

int tricore_resume(struct target *target, bool current, target_addr_t address, bool handle_breakpoints,
				   bool debug_execution)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret = 0;

	LOG_TARGET_DEBUG(target, "%s, state=%d", __func__, target->state);

	if (target->state == TARGET_RUNNING) {
		LOG_TARGET_DEBUG(target, "Target already running");
		return ERROR_OK;
	}

	if (current == 0) {
		uint8_t pc_buf[4];

		buf_set_u32(pc_buf, 0, 32, (uint32_t)address);
		ret = tricore_reg_set(tricore->pc, pc_buf);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to set PC before continue");
			return ret;
		}
	}

	ret = tricore_restore_reg_cache(target);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to restore events");
		return ret;
	}

	/*
	 * Resume by clearing DBGSR.HALT.
	 *
	 * Note that the halt-after-reset trigger the SSW arms on the boot address
	 * is disarmed in tricore_init_debug_access(); without that, this write
	 * appears to do nothing because the core halts again on the very next
	 * instruction fetch.
	 */
	ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_DBGSR),
							 TRICORE_DBGSR_HALT_CLEAR);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to continue target");
		return ret;
	}

	/* registers are now invalid */
	register_cache_invalidate(target->reg_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
		ret = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		if (ret)
			return ret;
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target->debug_reason = DBG_REASON_NOTHALTED;
		ret = target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		if (ret)
			return ret;
	}

	return ERROR_OK;
}

int tricore_step(struct target *target, bool current, target_addr_t address, bool handle_breakpoints)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (current == 0) {
		uint8_t pc_buf[4];

		buf_set_u32(pc_buf, 0, 32, (uint32_t)address);
		ret = tricore_reg_set(tricore->pc, pc_buf);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to set PC before step");
			return ret;
		}
	} else {
		ret = tricore_reg_get(tricore->pc);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to get PC before step");
			return ret;
		}
		address = buf_get_u32(tricore->pc->value, 0, 32);
	}

	struct breakpoint *breakpoint = NULL;
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, address);
		if (breakpoint) {
			ret = tricore_unset_breakpoint(target, breakpoint);
			if (ret != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed to temporarily remove breakpoint at 0x%08" TARGET_PRIxADDR,
						 breakpoint->address);
				goto restore_breakpoint;
			}
		}
	}

	ret = tricore_restore_reg_cache(target);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to restore events");
		goto restore_breakpoint;
	}

	/*
	 * Single-step using a PC *range* trigger covering the whole address space,
	 * with break-after-make semantics: the core executes exactly one
	 * instruction and then halts.
	 *
	 * The previous implementation armed an equality trigger on the *current* PC
	 * and resumed. Because the core is already at that address, it either
	 * re-triggered without making progress or never matched again, which is why
	 * stepping always ended in "Target did not halt after step".
	 */
	struct breakpoint step_breakpoint = {
		.type = BKPT_HARD,
		.address = 0,
		.length = 0xFFFFFFFF,
	};
	ret = tricore_set_breakpoint(target, &step_breakpoint, false);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to set step breakpoint");
		goto restore_breakpoint;
	}
	ret = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	if (ret != ERROR_OK)
		goto restore_breakpoint;

	ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_DBGSR),
							 TRICORE_DBGSR_HALT_CLEAR);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to start single stepping");
		goto restore_breakpoint;
	}

	/* registers are now invalid */
	register_cache_invalidate(target->reg_cache);

	/* Wait for the step to complete rather than assuming a fixed delay. */
	int64_t deadline = timeval_ms() + TRICORE_STEP_TIMEOUT_MS;
	do {
		ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_DBGSR), &tricore->dbgsr);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to read DBGSR after step");
			if (tricore_unset_breakpoint(target, &step_breakpoint) != ERROR_OK)
				LOG_TARGET_ERROR(target, "Failed to remove step breakpoint after read error");
			goto restore_breakpoint;
		}
		if (tricore->dbgsr & TRICORE_DBGSR_HALT)
			break;
		usleep(100);
	} while (timeval_ms() < deadline);

	ret = tricore_unset_breakpoint(target, &step_breakpoint);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to remove step breakpoint");
		goto restore_breakpoint;
	}

restore_breakpoint:
	/* Every path out of here goes through this, so a step that fails or times
	 * out can never leave the user's breakpoint disabled for the rest of the
	 * session. */
	if (breakpoint) {
		int restore_ret = tricore_set_breakpoint(target, breakpoint, true);
		if (restore_ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to restore breakpoint at 0x%08" TARGET_PRIxADDR,
					 breakpoint->address);
			if (ret == ERROR_OK)
				ret = restore_ret;
		}
	}
	if (ret != ERROR_OK)
		return ret;

	if (tricore->dbgsr & TRICORE_DBGSR_HALT) {
		target->debug_reason = DBG_REASON_SINGLESTEP;
		ret = tricore_debug_entry(target);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to read debug status after step");
			return ret;
		}
		ret = target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		if (ret != ERROR_OK)
			return ret;
	} else {
		LOG_TARGET_ERROR(target, "Target did not halt after step (DBGSR: 0x%08x)", tricore->dbgsr);
		return ERROR_FAIL;
	}

	return ret;
}

int tricore_assert_reset(struct target *target)
{
	enum reset_types reset_config = jtag_get_reset_config();

	LOG_TARGET_DEBUG(target, "Reset asserted");

	/* Issue some kind of warm reset. */
	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
	} else if (reset_config & RESET_HAS_SRST) {
		int retval = adapter_assert_reset();
		if (retval != ERROR_OK)
			return retval;
		if (reset_config & RESET_SRST_PULLS_TRST) {
			LOG_ERROR("TRST with PORST will put chip in test mode");
			return ERROR_FAIL;
		}
		if (reset_config & RESET_CNCT_UNDER_SRST) {
			LOG_ERROR("Connect under SRST not possible");
			return ERROR_FAIL;
		}
	} else {
		LOG_ERROR("%s: how to reset?", target_name(target));
		return ERROR_FAIL;
	}

	/* registers are now invalid */
	register_cache_invalidate(target->reg_cache);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

int tricore_deassert_reset(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret;

	if (target->coreid == 0 && target->reset_halt) {
		/* Set HAAR bit before releasing the reset. */
		ret = tricore->ocmts->ops->queue_io_set_ojconf(tricore->ocmts, 0x3);
		if (ret != ERROR_OK)
			return ret;
	}

	/* be certain SRST is off */
	ret = adapter_deassert_reset();
	if (ret != ERROR_OK)
		return ret;

	if (target->coreid == 0) {
		int err = ocmts_init(tricore->ocmts);
		if (err) {
			LOG_TARGET_ERROR(target, "Failed to re-enable debug after reset");
			return err;
		}
		alive_sleep(100);
	}

	/*
	 * The halt-after-reset trigger the SSW arms on the boot address is cleared
	 * by tricore_init_debug_access() below, for both reset_halt and reset_run.
	 * It used to be cleared here with a block write, which the CSFR aperture
	 * silently drops.
	 */

	if (!target_was_examined(target))
		return ERROR_OK;

	ret = tricore_init_debug_access(target);
	if (ret != ERROR_OK)
		return ret;

	ret = tricore_poll(target);
	if (ret != ERROR_OK)
		return ret;

	if (target->reset_halt) {
		if (target->state != TARGET_HALTED) {
			LOG_TARGET_WARNING(target, "ran after reset and before halt ...");
			if (target_was_examined(target)) {
				ret = tricore_halt(target);
				if (ret != ERROR_OK)
					return ret;

				/*
				 * tricore_halt() only requests the halt by writing DBGSR; it
				 * never updates target->state, refreshes the register cache,
				 * or fires TARGET_EVENT_HALTED the way the generic "halt"
				 * command does by following up with target_wait_state(). Do
				 * the same here: poll until tricore_poll() observes DBGSR.HALT
				 * and brings target->state to TARGET_HALTED itself. Without
				 * this, target->state is left at whatever tricore_poll() set
				 * it to a few lines up (typically TARGET_RUNNING), so the
				 * very next tricore_resume() call sees "already running" and
				 * returns immediately without touching the PC - the core
				 * stays wherever DBGSR.HALT caught it instead of moving to
				 * the address the caller asked for.
				 */
				int64_t start = timeval_ms();
				while (target->state != TARGET_HALTED) {
					ret = tricore_poll(target);
					if (ret != ERROR_OK)
						return ret;
					if (target->state == TARGET_HALTED)
						break;
					if (timeval_ms() - start > 2000) {
						LOG_TARGET_ERROR(target,
							"target did not reach halted state after reset recovery");
						break;
					}
					alive_sleep(10);
				}
			} else {
				target->state = TARGET_UNKNOWN;
			}
		}
	} else if (target->state == TARGET_HALTED) {
		/*
		 * "reset run" must leave the core executing. The SSW halts at the boot
		 * vector regardless of what we ask for, so an explicit resume is needed
		 * once the triggers have been disarmed - otherwise the target sits at
		 * the reset vector and the firmware never starts.
		 */
		ret = tricore_resume(target, true, 0, false, false);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to resume after reset");
			return ret;
		}
	}

	return ERROR_OK;
}
int tricore_soft_reset_halt(struct target *target)
{
	return ERROR_FAIL;
}

const char *tricore_get_gdb_arch(const struct target *target)
{
	return "tricore";
}

int tricore_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size,
							 enum target_register_class reg_class)
{
	switch (reg_class) {
	case REG_CLASS_ALL:
	case REG_CLASS_GENERAL:
		*reg_list_size = target->reg_cache->num_regs;
		*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));
		if (!*reg_list) {
			LOG_TARGET_ERROR(target, "Out of memory");
			return ERROR_FAIL;
		}

		int i;
		for (i = 0; i < *reg_list_size; i++)
			(*reg_list)[i] = &target->reg_cache->reg_list[i];
		return ERROR_OK;
	default:
		LOG_ERROR("not a valid register class type in query.");
		return ERROR_FAIL;
	}
}

int tricore_read_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct tricore_info *tricore = target_to_tricore(target);
	struct ocmts *ocmts = tricore->ocmts;
	int ret;

	/* OCMTS cannot service unaligned half-word or word accesses. A read
	 * fails with a confusing zero-count error, so reject it up front. */
	if ((size == 2 || size == 4) && (address & (size - 1))) {
		LOG_TARGET_ERROR(target, "Unaligned %" PRIu32 "-byte read at 0x%08" PRIx64, size, address);
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	switch (size) {
	case 1:
	case 2:
		while (count > 0) {
			ret = ocmts_queue_io_set_address(ocmts, address);
			if (ret)
				goto err;
			if (size == 1)
				ret = ocmts_queue_io_read_byte(ocmts, buffer);
			else
				ret = ocmts_queue_io_read_hword(ocmts, (void *)buffer);
			if (ret)
				goto err;
			count--;
			address += size;
			buffer += size;
		}
		ret = ocmts_run(ocmts);
		if (ret)
			goto err;
		break;
	case 4:
		/* Use individual 32-bit reads for all counts.
		 * Block reads (cmd=10) have complex response framing; individual
		 * reads (cmd=8) are reliable and confirmed working. */
		while (count > 0) {
			uint32_t value;
			ret = ocmts_io_read_u32(ocmts, address, &value);
			if (ret)
				goto err;
			buf_set_u32(buffer, 0, 32, value);
			count--;
			address += 4;
			buffer += 4;
		}
		break;
	default:
		LOG_ERROR("Unsupported read size %" PRIu32, size);
		return ERROR_FAIL;
	}
	return ERROR_OK;
err:
	LOG_TARGET_ERROR(target, "Failed to read memory at 0x%08" PRIx64 "[count=%u, size=%u]", address, count, size);
	return ret;
}

int tricore_write_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count,
						 const uint8_t *buffer)
{
	struct tricore_info *tricore = target_to_tricore(target);
	struct ocmts *ocmts = tricore->ocmts;
	int ret;

	/* aurix-mac-dap: GDB "load", load_image and mww write here, not through a
	 * flash driver. An ACS image carries its BMHD at UCB0 addresses, so a plain
	 * load would aim at the UCB; refuse instead of finding out what the NVM
	 * does with it. See aurix_range_ucb(). */
	const char *ucb = aurix_range_ucb(address, (uint64_t)size * count);
	if (ucb) {
		LOG_TARGET_ERROR(target, "Refusing to write %" PRIu32 " x %" PRIu32 " bytes at 0x%08" PRIx64
				 ": it touches %s, which this build never modifies", count, size, address, ucb);
		return ERROR_FAIL;
	}

	/* An unaligned half-word or word write is discarded by OCMTS while still
	 * reporting success, which silently loses data. Reject it up front. */
	if ((size == 2 || size == 4) && (address & (size - 1))) {
		LOG_TARGET_ERROR(target, "Unaligned %" PRIu32 "-byte write at 0x%08" PRIx64, size, address);
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	switch (size) {
	case 1:
	case 2:
		while (count) {
			ret = ocmts_queue_io_set_address(ocmts, address);
			if (ret)
				return ret;
			if (size == 1)
				ret = ocmts_queue_io_write_byte(ocmts, buffer);
			else
				ret = ocmts_queue_io_write_hword(ocmts, buffer);
			if (ret)
				return ret;
			count--;
			address += size;
			buffer += size;
		}
		ret = ocmts_run(ocmts);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to write memory at 0x%08" PRIx64 "[count=%u, size=%u]", address,
							 count, size);
			return ret;
		}
		break;
	case 4:
		/* Use individual 32-bit writes for all counts.
		 * Block writes (cmd=9 writemem) have complex payload framing;
		 * individual cmd=8 writes are reliable and confirmed working. */
		while (count) {
			uint32_t value = buf_get_u32(buffer, 0, 32);
			ret = ocmts_io_write_u32(ocmts, address, value);
			if (ret)
				return ret;
			count--;
			address += 4;
			buffer += 4;
		}
		return ERROR_OK;
	default:
		LOG_ERROR("Unsupported write size %u", size);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const uint8_t tricore_crc_program[] = {
#include "contrib/loaders/checksum/tricore.inc"
};

int tricore_checksum_memory(struct target *target, target_addr_t address, uint32_t count, uint32_t *checksum)
{
	int ret;
	const size_t crc_code_size = ARRAY_SIZE(tricore_crc_program);

	if (count < crc_code_size * 4) {
		/* Don't use the algorithm for relatively small buffers. It's faster
		 * just to read the memory.  target_checksum_memory() will take care of
		 * that if we fail. */
		return ERROR_FAIL;
	}

	if (0x70100000 + crc_code_size > address && 0x70100000 < address + count) {
		LOG_TARGET_ERROR(target, "Memory range overlaps with CRC program");
		return ERROR_FAIL;
	}

	ret = target_write_buffer(target, 0x70100000, crc_code_size, tricore_crc_program);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to write CRC program to target");
		return ret;
	}

	/*
	 * reg_param values are byte buffers in target byte order: they are
	 * consumed by tricore_reg_set()/target_buffer_get_u32(). Marshal them
	 * explicitly so the code is correct on a big-endian host too.
	 */
	uint8_t addr_buf[4], count_buf[4], checksum_buf[4];

	buf_set_u32(addr_buf, 0, 32, (uint32_t)address);
	buf_set_u32(count_buf, 0, 32, count);

	struct reg_param reg_params[] = {
		{.reg_name = "a4", .size = 32, .value = addr_buf, .direction = PARAM_OUT},
		{.reg_name = "d4", .size = 32, .value = count_buf, .direction = PARAM_OUT},
		{.reg_name = "d2", .size = 32, .value = checksum_buf, .direction = PARAM_IN},
	};

	/* The CRC is computed one bit at a time on the target, so the run time
	 * grows with the buffer. Allow for a pessimistic 250 KiB/s on top of a
	 * fixed margin for setting the algorithm up. */
	unsigned int timeout_ms = 100 + count / 256;

	ret = target_run_algorithm(target, 0, NULL, 3, reg_params, 0x70100000, 0, timeout_ms, NULL);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to run CRC program on target");
		return ret;
	}

	*checksum = buf_get_u32(checksum_buf, 0, 32);

	return ERROR_OK;
}

/*
 * Helper function to configure a hardware breakpoint event.
 */
static int tricore_event_set(struct target *target, uint8_t event_id)
{
	struct tricore_info *tricore = target_to_tricore(target);
	struct tricore_event *event;
	struct tricore_event *event2;
	int ret = ERROR_OK;
	uint32_t trxevt = 0;

	if (event_id >= TRICORE_NUM_EVENTS)
		return ERROR_FAIL;

	event = &tricore->events[event_id];
	event2 = NULL;

	/*
	 * A range trigger is programmed as a consecutive even/odd register pair,
	 * so it has to start on an even slot and its partner has to exist.
	 * event2 stays NULL for a single trigger, which keeps the pair-only
	 * accesses below provably safe.
	 */
	if (event->compare == TRICORE_EVENT_RANGE) {
		if ((event_id % 2) != 0 || event_id + 1 >= TRICORE_NUM_EVENTS)
			return ERROR_FAIL;
		event2 = &tricore->events[event_id + 1];
	}

	if (event->enable) {
		if (event2 && !event2->enable)
			return ERROR_FAIL;
		trxevt |= (event->bbm << 3) | (event->type << 12) | (event->compare << 13) | (event->access << 27);
		if (tricore->version == TRICORE_1_8)
			trxevt |= 1;
		else
			trxevt |= 2;

		/* Program address first and then event */
		if (event2) {
			ret = ocmts_queue_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXADR(event_id + 1)),
										&event2->address);
		}
		if (ret == ERROR_OK)
			ret = ocmts_queue_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXADR(event_id)),
										&event->address);
		if (ret == ERROR_OK)
			ret = ocmts_queue_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXEVT(event_id)),
										&trxevt);
		if (ret == ERROR_OK)
			ret = ocmts_run(tricore->ocmts);
		if (ret)
			return ret;
	} else {
		/*
		 * Disarm the entry (and its range partner). Single-word writes are used
		 * because the CSFR aperture does not service block accesses - a block
		 * write here silently does nothing and leaves the trigger armed.
		 */
		uint8_t last = (event->compare == TRICORE_EVENT_RANGE) ? event_id + 1 : event_id;

		for (uint8_t i = event_id; i <= last && i < TRICORE_NUM_EVENTS; i++) {
			ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXEVT(i)),
									 TRICORE_EVENT_DISABLED);
			if (ret == ERROR_OK)
				ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXADR(i)),
										 0xFFFFFFFF);
			if (ret)
				return ret;
		}
	}

	return ERROR_OK;
}

/*
 * Set a breakpoint by finding an available trigger event and configuring it.
 */
int tricore_set_breakpoint(struct target *target, struct breakpoint *breakpoint, bool bbm)
{
	struct tricore_info *tricore = target_to_tricore(target);
	uint8_t event_id = 0;
	const bool range = breakpoint->length > 4;
	int ret;

	/*
	 * Find an available event slot. A range breakpoint occupies a
	 * consecutive even/odd pair, so both halves of the pair must be free;
	 * checking only the even slot would silently overwrite an already
	 * configured odd slot.
	 */
	for (event_id = 0; event_id < TRICORE_NUM_EVENTS; event_id++) {
		if (tricore->events[event_id].enable)
			continue;
		if (!range)
			break;
		if ((event_id % 2) != 0 || event_id + 1 >= TRICORE_NUM_EVENTS)
			continue;
		if (!tricore->events[event_id + 1].enable)
			break;
	}

	if (event_id >= TRICORE_NUM_EVENTS) {
		LOG_TARGET_ERROR(target, "No hardware event available for breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Configure the event */
	tricore->events[event_id] = (struct tricore_event){
		.type = TRICORE_EVENT_PC,
		.bbm = bbm,
		.access = 0, /* Unused for PC breakpoints */
		.address = breakpoint->address,
		.compare = (breakpoint->length > 4) ? TRICORE_EVENT_RANGE : TRICORE_EVENT_EQUALITY,
		.enable = true,
	};
	breakpoint->number = event_id;

	/* For range breakpoints, configure second event */
	if (breakpoint->length > 4) {
		tricore->events[event_id + 1] = (struct tricore_event){
			.type = TRICORE_EVENT_PC,
			.address = breakpoint->address + breakpoint->length,
			.enable = true,
		};
		breakpoint->linked_brp = event_id + 1;
	} else {
		breakpoint->linked_brp = TRICORE_NUM_EVENTS; /* Invalid index */
	}

	ret = tricore_event_set(target, event_id);
	if (ret) {
		tricore->events[event_id].enable = false;
		if (breakpoint->length > 4)
			tricore->events[event_id + 1].enable = false;
		LOG_TARGET_ERROR(target, "Failed to set breakpoint at 0x%" PRIx64, breakpoint->address);
		return ret;
	}

	breakpoint->is_set = true;
	LOG_TARGET_DEBUG(target, "Breakpoint set at 0x%" PRIx64 " using event %d", breakpoint->address, event_id);

	return ERROR_OK;
}

/*
 * Remove a breakpoint by disabling its trigger event.
 */
int tricore_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct tricore_info *tricore = target_to_tricore(target);
	uint8_t event_id = breakpoint->number;
	int ret = ERROR_OK;

	if (event_id >= TRICORE_NUM_EVENTS) {
		LOG_TARGET_WARNING(target, "Invalid breakpoint event ID %d", event_id);
		return ERROR_OK;
	}

	if (!tricore->events[event_id].enable) {
		LOG_TARGET_WARNING(target, "Breakpoint at 0x%" PRIx64 " already disabled", breakpoint->address);
		return ERROR_OK;
	}

	/* Disable the event */
	tricore->events[event_id].enable = false;
	if (breakpoint->linked_brp < TRICORE_NUM_EVENTS)
		tricore->events[breakpoint->linked_brp].enable = false;

	ret = tricore_event_set(target, event_id);
	if (ret) {
		LOG_TARGET_ERROR(target, "Failed to unset breakpoint at 0x%" PRIx64, breakpoint->address);
		return ret;
	}

	breakpoint->is_set = false;
	LOG_TARGET_DEBUG(target, "Breakpoint removed at 0x%" PRIx64, breakpoint->address);

	return ret;
}

/*
 * target break-/watchpoint control
 * rw: 0 = write, 1 = read, 2 = access
 *
 * Target must be halted while this is invoked as this
 * will actually set up breakpoints on target.
 *
 * The breakpoint hardware will be set up upon adding the
 * first breakpoint.
 *
 * Upon GDB connection all breakpoints/watchpoints are cleared.
 */
int tricore_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int available_events = 0;
	int i;

	/* Only hardware breakpoints are supported */
	if (breakpoint->type != BKPT_HARD) {
		LOG_TARGET_ERROR(target, "Only hardware breakpoints are supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Count available events */
	for (i = 0; i < TRICORE_NUM_EVENTS; i++) {
		if (!tricore->events[i].enable)
			available_events++;
	}

	/* Check if we have enough events */
	if (available_events < 1) {
		LOG_TARGET_ERROR(target, "No hardware event available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Range breakpoints need 2 consecutive events */
	if (breakpoint->length > 4 && available_events < 2) {
		LOG_TARGET_ERROR(target, "Range breakpoint requires 2 hardware events");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return tricore_set_breakpoint(target, breakpoint, true);
}

/* remove breakpoint. hw will only be updated if the target
 * is currently halted.
 * However, this method can be invoked on unresponsive targets.
 */
int tricore_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	if (breakpoint->is_set)
		return tricore_unset_breakpoint(target, breakpoint);

	return ERROR_OK;
}

/* add watchpoint ... see add_breakpoint() comment above. */
int tricore_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct tricore_info *tricore = target_to_tricore(target);
	uint8_t event_id = 0;
	int available_events = 0;
	int i, ret;

	/* Count available events */
	for (i = 0; i < TRICORE_NUM_EVENTS; i++) {
		if (!tricore->events[i].enable)
			available_events++;
	}

	if (available_events < 1) {
		LOG_TARGET_ERROR(target, "No hardware event available for watchpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Find an available event slot. Range watchpoints need a free
	 * consecutive even/odd pair, not just a free even slot. */
	for (event_id = 0; event_id < TRICORE_NUM_EVENTS; event_id++) {
		if (tricore->events[event_id].enable)
			continue;
		if (watchpoint->length <= 4)
			break;
		if ((event_id % 2) != 0 || event_id + 1 >= TRICORE_NUM_EVENTS)
			continue;
		if (!tricore->events[event_id + 1].enable)
			break;
	}

	if (event_id >= TRICORE_NUM_EVENTS) {
		LOG_TARGET_ERROR(target, "No hardware event available for watchpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	watchpoint->number = event_id;

	/* Configure the event for memory access */
	struct tricore_event *event = &tricore->events[event_id];
	event->type = TRICORE_EVENT_ADDRESS;
	event->bbm = false;
	event->compare = (watchpoint->length > 4) ? TRICORE_EVENT_RANGE : TRICORE_EVENT_EQUALITY;
	event->address = watchpoint->address;
	event->enable = true;

	/* Set access type based on watchpoint rw value */
	switch (watchpoint->rw) {
	case WPT_READ:
		event->access = TRICORE_EVENT_LOAD;
		break;
	case WPT_WRITE:
		event->access = TRICORE_EVENT_STORE;
		break;
	case WPT_ACCESS:
		event->access = TRICORE_EVENT_LOAD | TRICORE_EVENT_STORE;
		break;
	default:
		event->enable = false;
		LOG_TARGET_ERROR(target, "Invalid watchpoint type");
		return ERROR_FAIL;
	}

	/* For range watchpoints, configure second event */
	if (watchpoint->length > 4) {
		struct tricore_event *event2 = &tricore->events[event_id + 1];
		event2->address = watchpoint->address + watchpoint->length;
		event2->enable = true;
	}

	/* Write to hardware if target is halted */
	if (target->state == TARGET_HALTED) {
		ret = tricore_event_set(target, event_id);
		if (ret) {
			event->enable = false;
			if (watchpoint->length > 4)
				tricore->events[event_id + 1].enable = false;
			LOG_TARGET_ERROR(target, "Failed to set watchpoint at 0x%" PRIx64, watchpoint->address);
			return ret;
		}
	}

	watchpoint->is_set = true;
	LOG_TARGET_DEBUG(target, "Watchpoint set at 0x%" PRIx64 " using event %d", watchpoint->address, event_id);

	return ERROR_OK;
}

/* remove watchpoint. hw will only be updated if the target
 * is currently halted.
 * However, this method can be invoked on unresponsive targets.
 */
int tricore_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct tricore_info *tricore = target_to_tricore(target);
	uint8_t event_id = watchpoint->number;
	int ret = ERROR_OK;

	if (!watchpoint->is_set) {
		LOG_TARGET_WARNING(target, "Watchpoint at 0x%" PRIx64 " not set", watchpoint->address);
		return ERROR_OK;
	}

	if (event_id >= TRICORE_NUM_EVENTS || !tricore->events[event_id].enable) {
		LOG_TARGET_WARNING(target, "Invalid watchpoint event ID %d", event_id);
		return ERROR_OK;
	}

	/* Disable the event */
	tricore->events[event_id].enable = false;

	/* If it's a range watchpoint, disable the second event too */
	if (watchpoint->length > 4 && (event_id + 1) < TRICORE_NUM_EVENTS)
		tricore->events[event_id + 1].enable = false;

	/* Clear hardware registers if target is halted */
	if (target->state == TARGET_HALTED) {
		ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXEVT(event_id)), 0);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to clear TRxEVT[%d]", event_id);
			return ret;
		}

		ret = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXADR(event_id)), 0);
		if (ret) {
			LOG_TARGET_ERROR(target, "Failed to clear TRxADR[%d]", event_id);
			return ret;
		}

		/* Clear second event for range watchpoints */
		if (watchpoint->length > 4) {
			int err = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXEVT(event_id + 1)),
											TRICORE_EVENT_DISABLED);
			if (err) {
				LOG_TARGET_ERROR(target, "Failed to clear TRxEVT[%d]", event_id + 1);
				/* Keep going so the address register is disarmed too, but do
				 * not let the second write's result hide this failure. */
				if (ret == ERROR_OK)
					ret = err;
			}

			err = ocmts_io_write_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TRXADR(event_id + 1)),
									 0xFFFFFFFF);
			if (err) {
				LOG_TARGET_ERROR(target, "Failed to clear TRxADR[%d]", event_id + 1);
				if (ret == ERROR_OK)
					ret = err;
			}
		}
	}

	watchpoint->is_set = false;
	LOG_TARGET_DEBUG(target, "Watchpoint removed at 0x%" PRIx64, watchpoint->address);

	return ret;
}

/* Find out just hit watchpoint. After the target hits a watchpoint, the
 * information could assist gdb to locate where the modified/accessed memory is.
 */
int tricore_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
{
	struct tricore_info *tricore = target_to_tricore(target);
	struct watchpoint *wp;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "Target not halted");
		return ERROR_FAIL;
	}

	/* Check which trigger event caused the halt */
	if (tricore->active_event >= TRICORE_EVENT_TRIGGER0 && tricore->active_event <= TRICORE_EVENT_TRIGGER7) {
		uint8_t event_id = tricore->active_event - TRICORE_EVENT_TRIGGER0;

		/* Look for a watchpoint using this event */
		for (wp = target->watchpoints; wp; wp = wp->next) {
			if (wp->is_set && wp->number == event_id) {
				/* Verify it's an address event, not a PC event (breakpoint) */
				if (tricore->events[event_id].type == TRICORE_EVENT_ADDRESS) {
					*hit_watchpoint = wp;
					LOG_TARGET_DEBUG(target, "Hit watchpoint at 0x%" PRIx64, wp->address);
					return ERROR_OK;
				}
			}
		}
	}

	return ERROR_FAIL;
}

static int tricore_algorithm_save_context(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	unsigned int num_regs = target->reg_cache->num_regs;
	uint32_t *context;

	if (tricore->algorithm_context_regs != num_regs) {
		/* Do not assign the result of realloc() directly: on failure that
		 * would leak the buffer that is still live. */
		context = realloc(tricore->algorithm_context, num_regs * sizeof(*context));
		if (!context) {
			LOG_TARGET_ERROR(target, "Failed to allocate algorithm context");
			return ERROR_FAIL;
		}

		tricore->algorithm_context = context;
		tricore->algorithm_context_regs = num_regs;
	}

	for (unsigned int i = 0; i < num_regs; i++) {
		struct reg *reg = &target->reg_cache->reg_list[i];

		if (!reg->exist)
			continue;

		if (!reg->valid) {
			int ret = reg->type->get(reg);
			if (ret != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed to read register %s", reg->name);
				return ret;
			}
		}

		tricore->algorithm_context[i] = buf_get_u32(reg->value, 0, reg->size);
	}

	tricore->algorithm_debug_reason = target->debug_reason;

	return ERROR_OK;
}

static int tricore_algorithm_restore_context(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret;

	if (!tricore->algorithm_context)
		return ERROR_OK;

	for (unsigned int i = 0; i < tricore->algorithm_context_regs; i++) {
		struct reg *reg = &target->reg_cache->reg_list[i];

		if (!reg->exist)
			continue;

		buf_set_u32(reg->value, 0, reg->size, tricore->algorithm_context[i]);
		reg->valid = true;
		reg->dirty = true;
	}

	ret = tricore_restore_reg_cache(target);
	if (ret != ERROR_OK)
		return ret;

	target->debug_reason = tricore->algorithm_debug_reason;

	return ERROR_OK;
}

/**
 * Target algorithm support.  Do @b not call this method directly,
 * use target_run_algorithm() instead.
 */
int tricore_start_algorithm(struct target *target, int num_mem_params, struct mem_param *mem_params, int num_reg_params,
							struct reg_param *reg_param, target_addr_t entry_point, target_addr_t exit_point,
							void *arch_info)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret;
	bool context_saved = false;

	(void)exit_point;
	(void)arch_info;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted (start target algo)");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = tricore_algorithm_save_context(target);
	if (ret != ERROR_OK)
		return ret;
	context_saved = true;

	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;

		ret = target_write_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to write memory parameter at 0x%" TARGET_PRIxADDR, mem_params[i].address);
			goto restore_context;
		}
	}

	for (int i = 0; i < num_reg_params; i++) {
		struct reg *reg;

		if (reg_param[i].direction == PARAM_IN)
			continue;

		reg = register_get_by_name(target->reg_cache, reg_param[i].reg_name, false);
		if (!reg) {
			LOG_TARGET_ERROR(target, "BUG: register '%s' not found", reg_param[i].reg_name);
			ret = ERROR_COMMAND_SYNTAX_ERROR;
			goto restore_context;
		}

		if (reg->size != reg_param[i].size) {
			LOG_TARGET_ERROR(target, "BUG: register '%s' size doesn't match reg_param[i].size", reg_param[i].reg_name);
			ret = ERROR_COMMAND_SYNTAX_ERROR;
			goto restore_context;
		}

		ret = reg->type->set(reg, reg_param[i].value);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to set register %s", reg->name);
			goto restore_context;
		}
	}

	ret = tricore_resume(target, false, entry_point, true, true);
	if (ret == ERROR_OK)
		return ERROR_OK;

restore_context:
	if (context_saved) {
		int restore_ret = tricore_algorithm_restore_context(target);
		if (restore_ret != ERROR_OK)
			LOG_TARGET_ERROR(target, "Failed to restore target state after algorithm setup error");
	}
	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_HALTED;
	target->debug_reason = tricore->algorithm_debug_reason;

	return ret;
}

int tricore_wait_algorithm(struct target *target, int num_mem_params, struct mem_param *mem_params, int num_reg_params,
						   struct reg_param *reg_param, target_addr_t exit_point, unsigned int timeout_ms,
						   void *arch_info)
{
	struct tricore_info *tricore = target_to_tricore(target);
	int ret;
	int retval = ERROR_OK;

	(void)arch_info;

	ret = target_wait_state(target, TARGET_HALTED, timeout_ms);
	if (ret != ERROR_OK || target->state != TARGET_HALTED) {
		ret = target_halt(target);
		if (ret != ERROR_OK)
			return ret;

		ret = target_wait_state(target, TARGET_HALTED, 500);
		if (ret != ERROR_OK)
			return ret;

		retval = ERROR_TARGET_TIMEOUT;
	} else if (exit_point) {
		ret = tricore->pc->type->get(tricore->pc);
		if (ret != ERROR_OK)
			return ret;

		uint32_t pc = buf_get_u32(tricore->pc->value, 0, tricore->pc->size);
		if (pc != exit_point) {
			LOG_TARGET_DEBUG(target, "failed algorithm halted at 0x%08" PRIx32 ", expected 0x%08" TARGET_PRIxADDR, pc,
							 exit_point);
			retval = ERROR_TARGET_ALGO_EXIT;
		}
	}

	if (retval == ERROR_OK) {
		for (int i = 0; i < num_mem_params; i++) {
			if (mem_params[i].direction == PARAM_OUT)
				continue;

			ret = target_read_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value);
			if (ret != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed to read memory parameter at 0x%" TARGET_PRIxADDR,
								 mem_params[i].address);
				retval = ret;
				break;
			}
		}
	}

	if (retval == ERROR_OK) {
		for (int i = 0; i < num_reg_params; i++) {
			struct reg *reg;

			if (reg_param[i].direction == PARAM_OUT)
				continue;

			reg = register_get_by_name(target->reg_cache, reg_param[i].reg_name, false);
			if (!reg) {
				LOG_TARGET_ERROR(target, "BUG: register '%s' not found", reg_param[i].reg_name);
				retval = ERROR_COMMAND_SYNTAX_ERROR;
				break;
			}

			if (reg->size != reg_param[i].size) {
				LOG_TARGET_ERROR(target, "BUG: register '%s' size doesn't match reg_param[i].size",
								 reg_param[i].reg_name);
				retval = ERROR_COMMAND_SYNTAX_ERROR;
				break;
			}

			ret = reg->type->get(reg);
			if (ret != ERROR_OK) {
				retval = ret;
				break;
			}

			buf_cpy(reg->value, reg_param[i].value, reg->size);
		}
	}

	ret = tricore_algorithm_restore_context(target);
	if (ret != ERROR_OK)
		return ret;

	register_cache_invalidate(target->reg_cache);

	return retval;
}

int tricore_run_algorithm(struct target *target, int num_mem_params, struct mem_param *mem_params, int num_reg_params,
						  struct reg_param *reg_param, target_addr_t entry_point, target_addr_t exit_point,
						  unsigned int timeout_ms, void *arch_info)
{
	int ret;

	ret = tricore_start_algorithm(target, num_mem_params, mem_params, num_reg_params, reg_param, entry_point,
								  exit_point, arch_info);
	if (ret != ERROR_OK)
		return ret;

	return tricore_wait_algorithm(target, num_mem_params, mem_params, num_reg_params, reg_param, exit_point, timeout_ms,
								  arch_info);
}

/* called when target is created */
int tricore_target_create(struct target *target)
{
	struct tricore_private_config *pc = target->private_config;
	if (!pc->ocmts) {
		LOG_TARGET_ERROR(target, "OCMTS not configured for target");
		return ERROR_FAIL;
	}

	struct tricore_info *tricore = calloc(1, sizeof(struct tricore_info));
	if (!tricore) {
		LOG_TARGET_ERROR(target, "Failed to allocate target memory");
		return ERROR_FAIL;
	}
	target->arch_info = tricore;
	tricore->ocmts = pc->ocmts;

	return ERROR_OK;
}

static const struct jim_nvp nvp_config_opts[] = {{.name = "-ocmts", .value = 0}, {.name = NULL, .value = -1}};
/* called for various config parameters
 * returns JIM_CONTINUE - if option not understood
 * otherwise: JIM_OK, or JIM_ERR,*/
int tricore_target_jim_configure(struct target *target, struct jim_getopt_info *goi)
{
	int e;
	struct jim_nvp *n;
	struct tricore_private_config *pc = target->private_config;

	if (!pc) {
		pc = calloc(1, sizeof(struct tricore_private_config));
		if (!pc) {
			LOG_ERROR("Out of memory");
			return JIM_ERR;
		}
		target->private_config = pc;
	}

	if (!goi->argc)
		return JIM_OK;

	Jim_SetEmptyResult(goi->interp);

	e = jim_nvp_name2value_obj(goi->interp, nvp_config_opts, goi->argv[0], &n);
	if (e != JIM_OK)
		return JIM_CONTINUE;

	e = jim_getopt_obj(goi, NULL);
	if (e != JIM_OK)
		return e;

	switch (n->value) {
	case 0:
		if (goi->is_configure) {
			Jim_Obj *o_t;
			struct ocmts *ocmts;
			e = jim_getopt_obj(goi, &o_t);
			if (e != JIM_OK)
				return e;
			ocmts = ocmts_by_jim_obj(goi->interp, o_t);
			if (!ocmts) {
				Jim_SetResultString(goi->interp, "OCMTS name invalid!", -1);
				return JIM_ERR;
			}
			if (pc->ocmts && pc->ocmts != ocmts) {
				Jim_SetResultString(goi->interp, "OCMTS assignment cannot be changed!", -1);
				return JIM_ERR;
			}
			pc->ocmts = ocmts;
		} else {
			if (goi->argc)
				goto err_no_param;
			if (!pc->ocmts) {
				Jim_SetResultString(goi->interp, "OCMTS not configured", -1);
				return JIM_ERR;
			}
			Jim_SetResultString(goi->interp, pc->ocmts->name, -1);
		}
		break;
	}

	if (pc->ocmts) {
		target->tap = pc->ocmts->tap;
		target->ocmts_configured = true;
		target->has_ocmts = true;
	}

	return JIM_OK;

err_no_param:
	Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "No parameters");
	return JIM_ERR;
}

static int tricore_examine_first(struct target *target)
{
	int ret = ERROR_OK;
	struct tricore_info *tricore = target_to_tricore(target);
	uint32_t CPU_ID;

	ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_CPU_ID), &CPU_ID);
	if (ret)
		return ret;

	/*
	 * 0xC0C0xx is the CPU_ID family code used by TC1.6.2P/TC1.8 cores
	 * (TC3x/TC4x). 0xB7C0xx is the family code used by the plain
	 * TC1.6E core found on some TC2x devices; unlike the 0xC0C0xx
	 * family it does not encode further sub-variants in the low byte.
	 */
	if ((CPU_ID & 0xFFFF00) != 0xC0C000 && (CPU_ID & 0xFFFF00) != 0xB7C000) {
		LOG_TARGET_ERROR(target, "Invalid CPU_ID value: 0x%08x", CPU_ID);
		return ERROR_TARGET_INVALID;
	}
	if ((CPU_ID & 0xFFFF00) == 0xC0C000 && ((CPU_ID & 0xFF) == 0x31 || (CPU_ID & 0xFF) == 0x32)) {
		uint32_t tccon;
		LOG_TARGET_INFO(target, "TriCore version 1.8 found");
		tricore->version = TRICORE_1_8;
		ret = ocmts_io_read_u32(tricore->ocmts, tricore_get_reg_addr(target, TRICORE_TCCON), &tccon);
		if (ret)
			return ret;
		if (tccon & 0x1)
			tricore->fpu = TRICORE_FPU_SINGLE;
		if (tccon & 0x2)
			tricore->fpu = TRICORE_FPU_DOUBLE;
		if (tccon & 0x8)
			tricore->virt_enabled = true;
		LOG_TARGET_INFO(target, "TriCore 1.8 features: FPU=%s, Virtualization=%s",
						(tricore->fpu == TRICORE_FPU_DOUBLE)   ? "double"
						: (tricore->fpu == TRICORE_FPU_SINGLE) ? "single"
															   : "none",
						tricore->virt_enabled ? "yes" : "no");
	} else if ((CPU_ID & 0xFFFF00) == 0xC0C000 && (CPU_ID & 0xFF) == 0x21) {
		tricore->version = TRICORE_1_6_2;
		tricore->fpu = TRICORE_FPU_SINGLE;
		LOG_TARGET_INFO(target, "TriCore version 1.6.2P found");
	} else if ((CPU_ID & 0xFFFF00) == 0xC0C000 && ((CPU_ID & 0xFF) == 0x11 || (CPU_ID & 0xFF) == 0x12)) {
		tricore->version = TRICORE_1_6;
		tricore->fpu = TRICORE_FPU_SINGLE;
		LOG_TARGET_INFO(target, "TriCore version 1.6P found");
	} else if ((CPU_ID & 0xFFFF00) == 0xB7C000) {
		tricore->version = TRICORE_1_6;
		tricore->fpu = TRICORE_FPU_SINGLE;
		LOG_TARGET_INFO(target, "TriCore version 1.6E found");
	} else {
		LOG_TARGET_ERROR(target, "Unknown TriCore version found, CPU_ID: 0x%08x", CPU_ID);
		return ERROR_TARGET_INVALID;
	}
	target_set_examined(target);

	return ERROR_OK;
}

int tricore_examine(struct target *target)
{
	int ret;

	if (!target_was_examined(target)) {
		ret = tricore_examine_first(target);
		if (ret)
			return ret;
	}

	ret = tricore_init_debug_access(target);
	if (ret)
		return ret;

	ret = tricore_poll(target);
	if (ret)
		return ret;

	return ERROR_OK;
}

int tricore_init_target(struct command_context *cmd_ctx, struct target *target)
{
	int ret = ERROR_OK;
	ret = tricore_build_reg_cache(target, &tricore_reg_type);
	if (ret != ERROR_OK)
		return ret;
	return ERROR_OK;
}

void tricore_deinit_target(struct target *target)
{
	struct tricore_info *tricore = target_to_tricore(target);

	/*
	 * jim_configure() runs before target_create(), so the private config
	 * can be present even when arch_info was never allocated. Release it
	 * first and unconditionally, the way aarch64 and riscv do.
	 */
	free(target->private_config);
	target->private_config = NULL;

	/* target_create() may have failed before arch_info was allocated */
	if (!tricore)
		return;

	tricore_free_reg_cache(target);
	free(tricore->algorithm_context);
	free(tricore);
}

/* after reset is complete, the target can check if things are properly set
 * up.
 *
 * This can be used to check if e.g. DCC memory writes have been enabled for
 * arm7/9 targets, which they really should except in the most contrived
 * circumstances.
 */
int tricore_check_reset(struct target *target)
{
	return ERROR_OK;
}

/* Parse target-specific GDB query commands.
 * The string pointer "response_p" is always assigned by the called function
 * to a pointer to a NULL-terminated string, even when the function returns
 * an error. The string memory is not freed by the caller, so this function
 * must pay attention for possible memory leaks if the string memory is
 * dynamically allocated.
 */
int tricore_gdb_query_custom(struct target *target, const char *packet, char **response_p)
{
	return ERROR_FAIL;
}

unsigned int tricore_address_bits(struct target *target)
{
	return 32;
}

unsigned int tricore_data_bits(struct target *target)
{
	return 32;
}

static const struct command_registration tricore_commands[] = {COMMAND_REGISTRATION_DONE};

struct target_type tricore_target = {
	.name = "tricore",

	.poll = tricore_poll,
	.arch_state = tricore_arch_state,

	.halt = tricore_halt,
	.resume = tricore_resume,
	.step = tricore_step,

	.assert_reset = tricore_assert_reset,
	.deassert_reset = tricore_deassert_reset,
	.soft_reset_halt = tricore_soft_reset_halt,

	.read_memory = tricore_read_memory,
	.write_memory = tricore_write_memory,

	.checksum_memory = tricore_checksum_memory,

	.get_gdb_arch = tricore_get_gdb_arch,
	.get_gdb_reg_list = tricore_get_gdb_reg_list,

	.run_algorithm = tricore_run_algorithm,
	.start_algorithm = tricore_start_algorithm,
	.wait_algorithm = tricore_wait_algorithm,

	.add_breakpoint = tricore_add_breakpoint,
	.remove_breakpoint = tricore_remove_breakpoint,

	.add_watchpoint = tricore_add_watchpoint,
	.remove_watchpoint = tricore_remove_watchpoint,

	.target_create = tricore_target_create,

	.target_jim_configure = tricore_target_jim_configure,

	.init_target = tricore_init_target,
	.examine = tricore_examine,
	.deinit_target = tricore_deinit_target,

	.commands = tricore_commands,
};
