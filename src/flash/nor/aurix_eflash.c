// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   AURIX eFlash Driver for Infineon AURIX                                *
 *   Copyright (C) 2026 Infineon Technologies AG                           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "flash/common.h"
#include "flash/nor/core.h"
#include "flash/nor/driver.h"
#include "helper/bits.h"
#include "helper/log.h"
#include "helper/time_support.h"
#include "helper/types.h"
#include "jtag/jtag.h"
#include "target/algorithm.h"
#include "target/tricore/aurix.h"
#include "target/tricore/ocmts.h"
#include "target/tricore/tricore.h"
#include "target/target.h"

struct aurix_eflash_bank {
	/** Address of the command sequencer */
	target_addr_t cmd_addr;
	/** Address of the DMU registers */
	target_addr_t reg_addr;
	/** Address of the FSI registers */
	target_addr_t fsi_addr;
	/** Offset of the error register for a given command sequencer */
	target_addr_t err_offset;
	/** Offset of the status register for a given command sequencer */
	target_addr_t sts_offset;
	/** Size of a physical sector, where earase commands cannot cross */
	uint32_t phys_sector_size;
	/** Size of a (logical) sector. (Smallest earasable unit) */
	uint16_t sector_size;
	/** Maximum number of erasable sectors */
	uint16_t num_erase_sectors;
	/** Number of bytes for a burst program operation */
	uint16_t burst_size;
	/** Number of bytes for a page program operation. (Smallest programmable
	 * unit)
	 */
	uint16_t page_size;
	/** Offset of the busy bit for the bank in the status register */
	uint8_t busy_bit;
	/** Offset of the page bit for the bank in the status register */
	uint8_t page_bit;
	/** Bank probed */
	bool probed;
	/** TC2 eflash */
	bool tc2x;
	/** TC4 eflash */
	bool tc4x;
	/** DFLASH */
	bool dflash;
	/** DFLASH1, which is owned by the HSM */
	bool dflash1;
	/** User configuration block */
	bool ucb;
	/** PFLASH bank index, selects the DMU PROCONP register set */
	uint8_t pflash_index;
	/** Fallback mode for flash write */
	bool fallback_mode;
	/** Write timeout in milliseconds */
	uint32_t write_timeout_ms;
	/** Erase timeout in milliseconds */
	uint32_t erase_timeout_ms;
};

static uint32_t clear_status_key = 0xFA;
static uint32_t reset_to_read_key = 0xF0;
static uint32_t page_mode_p_key = 0x50;
static uint32_t page_mode_d_key = 0x5D;

#define UCB_CHIPID 0xAE400008
#define UCB_CHIPID_PROD 0x3C000000
#define SCU_CHIPID 0xF0036140

/* Chip ID read retry: how long to keep re-reading a chip ID register that
 * still reads as zero or as the wrong family, and how long to wait between
 * attempts. See tc3x_eflash_probe() for why this is needed. */
#define AURIX_CHIPID_TRIES    40
#define AURIX_CHIPID_DELAY_MS 25

/* aurix_eflash_check_busy() pre-check retry: aurix_bank->busy_bit is a
 * per-bank status flag distinct from the bit 31 "operation in flight" flag
 * that aurix_eflash_handle_error() polls at the end of a program/erase --
 * the two have been observed to clear a short, variable moment apart on
 * TC4x, so a command issued immediately after handle_error() saw bit 31
 * clear can still find busy_bit set here. 50 tries * 2ms = 100ms worst
 * case, polled with jtag_sleep() (microseconds) like handle_error() uses,
 * so a bank that really is busy still fails fast and a bank that is not
 * busy returns on the first iteration without any added delay. */
#define AURIX_EFLASH_CHECK_BUSY_TRIES    50
#define AURIX_EFLASH_CHECK_BUSY_DELAY_US 2000

/* Safety watchdog control register, ENDINIT protected. Its value on a TC375
 * Lite Kit at power-on is 0xfffc000f, i.e. armed. */
#define TC3X_WDTCON0           0xF003624C
#define TC3X_WDTCON1           0xF0036250
#define TC3X_WDTCPU0SR         0xF0036254
#define TC3X_WDTSCON0          0xF00362A8
#define TC3X_WDTSCON1          0xF00362AC
#define TC3X_WDTSSR            0xF00362B0
#define TC3X_ENDINIT_PW_MASK   0xFFFFFF00u
#define TC3X_ENDINIT_PW_STEP1  0x000000F1u
#define TC3X_ENDINIT_PW_STEP2  0x000000F2u
#define TC3X_ENDINIT_SET       0x00000003u
/* WDTxCON1.DR (Disable Request), see AURIX_TC3xx_Part1_UserManual.pdf
 * section 9.4.6: only takes effect in WDTxSR.DS once ENDINIT is re-asserted
 * with a valid Modify Access. */
#define TC3X_WDT_DR            0x00000008u
/* WDTxSR.DS (bit 3): the watchdog has actually reached Disabled Mode. This is
 * the authoritative indicator - unlike WDTxCON0, whose low byte reads back
 * the *inverted* password/lock bits rather than the Disabled Mode state, so
 * it cannot be used to tell Disabled Mode apart from an ordinary locked,
 * still-running watchdog. */
#define TC3X_WDTSR_DS          0x00000008u

/* SMU_core (Safety Management Unit) command/debug interface. Unlike
 * WDTxCON0/WDTxCON1, SMU_CMD is a plain register: writable from the host over
 * JTAG with no ENDINIT or per-CPU restriction. */
#define TC3X_SMU_CMD           0xF0036820
#define TC3X_SMU_DBG           0xF0036838
/* SMU_DBG.SSM (bits 1:0): 00 = START, 01 = RUN, 10 = FAULT, 11 = reserved. */
#define TC3X_SMU_SSM_MASK      0x00000003u
#define TC3X_SMU_SSM_START     0x00000000u
/* SMU_Start(ARG=0): CMD field (bits 3:0) = 0x0, ARG field (bits 7:4) = 0. */
#define TC3X_SMU_CMD_START     0x00000000u
#define SCU_CHIPID_CHREV 0x3F
#define SCU_CHIPID_CHTEC 0xC0

/* Protection configuration registers, as offsets into the DMU register block.
 * The PFLASH sector lock bits are spread over DMU_PROCONP_REGS registers of 32
 * bits each, one bit per logical sector, and the register set repeats every
 * 0x100 bytes for each PFLASH bank. */
#define DMU_HF_PROCONPF 0x080
#define DMU_HF_PROCONDF 0x088
#define DMU_HP_PROCONP(bank, reg) (0x010000 + (bank) * 0x100 + (reg) * 4)
#define DMU_PROCONP_REGS 6
/* Read protection, implies write protection for the whole flash. */
#define DMU_PROCON_RPRO BIT(31)
/* DF0_EEPROM locked for write protection. */
#define DMU_PROCONDF_L BIT(0)
#define SCU_CHIPID_FSIZE 0x0F000000
/* DF0 user mode control, bits 1:0 select single ended (0) or complement
 * sensing (any other value).  Loaded from the UCB during startup. */
#define DMU_HF_PROCONUSR 0x074
#define DMU_PROCONUSR_MODE 0x3

/* Program flash size encoded in SCU_CHIPID.FSIZE, in KiB. */
static const uint32_t tc3x_pflash_size_kb[16] = {
	256, 512, 1024, 1536, 2048, 2560, 3072, 4096,
	5120, 6144, 7168, 8192, 10240, 12288, 14336, 16384,
};

/* --- TC2x (first generation AURIX) -------------------------------------
 *
 * TC2x differs from TC3x/TC4x in three ways that matter to this driver:
 *
 *  1. It has a single command sequencer for every bank (HOST_CMD_ADDR,
 *     0xAF000000) instead of one per flash module, and a single 32 bit
 *     FSR that carries the busy/state bits AND the error bits in the same
 *     word. There is no separate error register, and no FSI.
 *  2. The FSR error bits sit at completely different positions from TC3x:
 *     bits 11/12/13 and 25/26 rather than bits 0..6. Everything else in
 *     the word is state, not error, so the error value must be masked
 *     before it is interpreted or the busy bits read as errors.
 *  3. PFLASH writes are protected by ENDINIT and must be issued with it
 *     temporarily lifted, via the two-step password toggle on WDTS_CON0.
 *     DFLASH writes are not, and erase is not.
 *
 * All addresses and masks below are corroborated by
 * contrib/loaders/flash/aurix/tc2x-flash.h, which was written independently
 * of any flash driver and implements the on-target loader for this same
 * part. Where a value could not be corroborated that way, or could not be
 * confirmed on silicon, the comment says so and the driver is written to
 * avoid depending on it.
 */

/* Flash status register. One word: busy bits 1..6, PROG 7, ERASE 8,
 * page-mode-ready 9 (PFLASH) / 10 (DFLASH), errors 11,12,13,25,26. */
#define TC2X_FSR 0xF8002010
/* Banked form for the ocmts register helpers. FSR = base + 0x1010, and
 * because status and error are the same word they share one offset. */
#define TC2X_REG_BASE 0xF8001000
#define TC2X_FSR_OFFSET 0x1010
/* Bits of the FSR that are errors. Identical to FSR_ERRORS in tc2x-flash.h. */
#define TC2X_FSR_ERRORS (BIT(11) | BIT(12) | BIT(13) | BIT(25) | BIT(26))
/* Busy bits, identical to TC2X_STATUS_BUSY_* in tc2x-flash.h. Note the gap
 * at bit 0: there is no "DFLASH0 busy" at bit 0. The upstream reference
 * implementation assumes there is, and is off by one for both data flash
 * banks; see tc2x_busy_bit_for(). */
#define TC2X_STATUS_BUSY_DFLASH0 BIT(1)
#define TC2X_STATUS_BUSY_DFLASH1 BIT(2)
#define TC2X_STATUS_BUSY_PFLASH0 BIT(3)
#define TC2X_STATUS_BUSY_PFLASH1 BIT(4)
#define TC2X_STATUS_BUSY_PFLASH2 BIT(5)
#define TC2X_STATUS_BUSY_PFLASH3 BIT(6)
/* Page-mode-ready bit, per flash type. */
#define TC2X_PAGE_READY_PFLASH BIT(9)
#define TC2X_PAGE_READY_DFLASH BIT(10)
/* Base of program flash on TC2x. Each program flash bank is 2 MiB and they
 * are numbered upwards from here, which is how pflash_index is derived. */
#define TC2X_PFLASH_BASE 0x80000000
/* Safety watchdog timer control register, used to toggle ENDINIT. */
#define TC2X_WDTS_CON0 0xF00360F0
/* Chip identification fields unique to TC2x. */
#define SCU_CHIPID_CHID 0xF000
#define SCU_CHIPID_CHID_TC27X 0x7000
#define SCU_CHIPID_CHID_TC29X 0x9000
/* CHTEC (bits 7:6) distinguishes the families: 0x40 is TC2x, 0x80 is TC3x. */
#define SCU_CHIPID_CHTEC_TC2X 0x40
/* Burst program command. TC2x uses 0x7A where TC3x/TC4x use 0xA6. */
#define TC2X_FLASH_WRITE_BURST_CMD 0x7A
#define TC3X_FLASH_WRITE_BURST_CMD 0xA6

/* Rejects a bank that does not fit into the program flash the device reports.
 * A too large bank in the configuration would otherwise only be noticed when an
 * erase or write runs off the end of the physical flash. */
static int tc3x_eflash_check_pflash_size(struct flash_bank *bank, uint32_t chipid)
{
	struct aurix_eflash_bank *tc3x_bank = bank->driver_priv;
	uint32_t size_kb, end;

	if (tc3x_bank->dflash || tc3x_bank->ucb)
		return ERROR_OK;

	size_kb = tc3x_pflash_size_kb[(chipid & SCU_CHIPID_FSIZE) >> 24];
	/* Segment number aside, the PFLASH banks start at the beginning of the
	 * program flash, so the segment offset is the offset into it. */
	end = (bank->base & 0x0fffffff) + bank->size;

	if (end > size_kb * 1024) {
		LOG_ERROR("Bank %s ends at 0x%08" PRIx32 " of program flash, but the device only has %" PRIu32
				  " KiB. Check the flash bank declaration.",
				  bank->name, end, size_kb);
		return ERROR_FLASH_BANK_INVALID;
	}

	return ERROR_OK;
}

/*
 * Permanently disable the Safety watchdog (WDTSCON0/WDTSCON1) for the
 * duration of the session.
 *
 * Per AURIX_TC3xx_Part1_UserManual.pdf section 9.4.5.1 ("Timer Modes"), a
 * watchdog's very first Password Access to WDTxCON0 unconditionally switches
 * it to Time-Out Mode: an independent counter starts at 0xFFFC and counts up
 * regardless of whatever ENDINIT value is written afterwards. "Time-Out Mode
 * can only be exited properly by setting ENDINIT = 1 with a correct access
 * sequence [...] If an improper access to the WDT is performed, or if the
 * timer overflows before ENDINIT is set, an alarm request is sent to the
 * Safety Management Unit (SMU)." Measured on a TC375 Lite Kit, that overflow
 * happens roughly 100ms after the first Password Access.
 *
 * Simply clearing ENDINIT and leaving it there (what this function used to
 * do) never performs that "proper exit", so the Time-Out Mode counter keeps
 * running in the background and the SMU alarm (confirmed via
 * RSTSTAT.SMU, offset 0xF0036050, read back as set after the failure) resets
 * the core out from under any flash operation slower than the ~100ms window.
 *
 * The only documented way to stop the counter for good is to transition into
 * Disabled Mode, which requires BOTH of:
 *   1. WDTxCON1.DR = 1 (Disable Request), writable only while WDTxCON0.ENDINIT
 *      is de-asserted (plain write, not password protected itself), and
 *   2. ENDINIT re-asserted (set back to 1) via a correct Password+Modify
 *      Access sequence - this is what actually commits DR into WDTxSR.DS and
 *      performs the "proper exit" from Time-Out Mode, landing in Disabled
 *      Mode (timer stopped) instead of Normal Mode (timer running, needs
 *      servicing).
 *
 * So, in order: Password Access (step 1) -> Modify Access with ENDINIT=0
 * (step 2, also required before WDTxCON1 is writable) -> write WDTxCON1.DR=1
 * -> Password Access (step 1 again) -> Modify Access with ENDINIT=1 (commits
 * Disabled Mode).
 *
 * This only works here because the Safety watchdog is a shared/system
 * resource, writable as a plain external memory access over JTAG. Per-CPU
 * watchdogs (WDTCPUyCON0/CON1) are NOT: confirmed experimentally that the
 * exact same sequence, byte for byte, silently has no effect at all when
 * aimed at WDTCPU0CON0 or WDTCPU1CON0 (every readback stays at the entry
 * value) regardless of which OpenOCD target context issues the writes. Only
 * code executing on that specific CPU can modify its own watchdog; see
 * tc3x_eflash_disable_cpu_wdt_algo() for the on-target equivalent used for
 * CPU0's watchdog.
 *
 * This cannot be done once at build time: the state comes back after a reset,
 * so it is re-applied on every probe. The register writes are idempotent.
 */
static int tc3x_eflash_disable_one_wdt(struct flash_bank *bank, uint32_t con0_addr, uint32_t con1_addr,
		uint32_t sr_addr, const char *name)
{
	uint32_t con0, sr, pw;
	int retval;

	retval = target_read_u32(bank->target, sr_addr, &sr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Cannot read %s status to check the watchdog state.", name);
		return ERROR_OK;
	}

	/* Already disabled. Checking WDTxSR.DS directly (rather than inferring it
	 * from WDTxCON0) is required: WDTxCON0's low byte reads back the
	 * password/lock bits, which looks the same whether the watchdog is
	 * disabled or merely locked and still counting, so it cannot be used to
	 * decide whether to skip re-running the sequence. Skipping when already
	 * disabled keeps the common case (second and later probes in the same
	 * session) cheap and avoids restarting Time-Out Mode for no reason. */
	if (sr & TC3X_WDTSR_DS)
		return ERROR_OK;

	retval = target_read_u32(bank->target, con0_addr, &con0);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Cannot read %s to check the watchdog state.", name);
		return ERROR_OK;
	}

	pw = con0 & TC3X_ENDINIT_PW_MASK;

	/* Step 1: Password Access (unlock). This alone switches the watchdog to
	 * Time-Out Mode, starting the ~100ms-ish countdown to an SMU alarm if
	 * nothing below completes correctly. */
	retval = target_write_u32(bank->target, con0_addr, pw | TC3X_ENDINIT_PW_STEP1);
	if (retval != ERROR_OK)
		return retval;
	/* Step 2: Modify Access, ENDINIT=0. De-asserting ENDINIT is required
	 * before WDTxCON1 becomes writable. */
	retval = target_write_u32(bank->target, con0_addr, pw | TC3X_ENDINIT_PW_STEP2);
	if (retval != ERROR_OK)
		return retval;

	/* Step 3: Request Disabled Mode. WDTxCON1 is not itself password
	 * protected; it is gated solely by the corresponding WDTxCON0.ENDINIT
	 * being de-asserted, which step 2 just did. */
	retval = target_write_u32(bank->target, con1_addr, TC3X_WDT_DR);
	if (retval != ERROR_OK) {
		LOG_WARNING("Failed to request %s disable (DR); the watchdog will stay in Time-Out Mode and PFLASH operations may be reset out from under them.",
				name);
		return ERROR_OK;
	}

	/* Step 4/5: re-assert ENDINIT via a fresh Password+Modify Access. This is
	 * the "proper exit" from Time-Out Mode the manual requires, and it is
	 * what commits WDTxCON1.DR=1 into WDTxSR.DS=1, i.e. it is what actually
	 * reaches Disabled Mode (timer stopped) rather than leaving the counter
	 * running. Without this, the DR request above is never applied. */
	retval = target_write_u32(bank->target, con0_addr, pw | TC3X_ENDINIT_PW_STEP1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(bank->target, con0_addr, pw | TC3X_ENDINIT_SET | TC3X_ENDINIT_PW_STEP2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(bank->target, sr_addr, &sr);
	if (retval != ERROR_OK || !(sr & TC3X_WDTSR_DS))
		LOG_WARNING("%s did not reach Disabled Mode; PFLASH operations may be reset out from under them.", name);

	return ERROR_OK;
}

static const uint8_t tc3x_wdt_disable_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc3x-wdt-disable.inc"
};

/*
 * Disable a per-CPU watchdog (WDTCPUyCON0/CON1) by running the same
 * Disabled Mode sequence as tc3x_eflash_disable_one_wdt(), but as code
 * executing on the target CPU itself via tc3x-wdt-disable.c, since per-CPU
 * watchdog registers silently ignore writes issued externally over JTAG -
 * see tc3x_eflash_disable_one_wdt()'s comment for how that was confirmed.
 *
 * Reuses the same fixed scratch work area (0x70100000) as the flash write
 * algorithm and the checksum helper; safe because none of them ever run
 * concurrently with each other.
 */
static int tc3x_eflash_disable_cpu_wdt_algo(struct flash_bank *bank, uint32_t con0_addr, uint32_t con1_addr,
		uint32_t sr_addr, const char *name)
{
	struct target *target = bank->target;
	struct reg_param reg_params[3];
	uint32_t sr;
	int retval;

	retval = target_read_u32(target, sr_addr, &sr);
	if (retval == ERROR_OK && (sr & TC3X_WDTSR_DS))
		return ERROR_OK; /* Already in Disabled Mode. */

	retval = target_write_buffer(target, 0x70100000, sizeof(tc3x_wdt_disable_code), tc3x_wdt_disable_code);
	if (retval != ERROR_OK) {
		LOG_WARNING("Failed to download the %s disable helper to the target; it will stay in Time-Out Mode.", name);
		return ERROR_OK;
	}

	init_reg_param(&reg_params[0], "a4", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "a5", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "d2", 32, PARAM_IN);

	buf_set_u32(reg_params[0].value, 0, 32, con0_addr);
	buf_set_u32(reg_params[1].value, 0, 32, con1_addr);

	retval = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params, 0x70100000, 0, 100, NULL);
	if (retval != ERROR_OK)
		LOG_WARNING("Failed to run the %s disable helper on the target; it will stay in Time-Out Mode.", name);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	retval = target_read_u32(target, sr_addr, &sr);
	if (retval != ERROR_OK || !(sr & TC3X_WDTSR_DS))
		LOG_WARNING("%s did not reach Disabled Mode; PFLASH operations may be reset out from under them.", name);

	return ERROR_OK;
}

/*
 * The SMU_core (Safety Management Unit) safety state machine defaults to the
 * START state out of reset (confirmed via SMU_DBG reading 0x00000000 on a
 * TC375 Lite Kit) and stays there forever unless something issues
 * SMU_Start(ARG=0) - normally done early on by boot software, but this board
 * never runs any (its flash is what we are trying to program in the first
 * place). While in START, PFLASH erase/program commands fail with
 * "Protection error" even once both watchdogs above are confirmed disabled.
 *
 * Unlike WDTxCON0, SMU_CMD is a plain register: not ENDINIT protected and not
 * gated per-CPU, so a host-side write is sufficient (confirmed: issuing this
 * over JTAG moved SMU_DBG.SSM from START (00) to RUN (01) immediately).
 */
static int tc3x_eflash_smu_ensure_run(struct flash_bank *bank)
{
	uint32_t dbg;
	int retval;

	retval = target_read_u32(bank->target, TC3X_SMU_DBG, &dbg);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Cannot read SMU_DBG to check the SMU_core state.");
		return ERROR_OK;
	}

	if ((dbg & TC3X_SMU_SSM_MASK) != TC3X_SMU_SSM_START)
		return ERROR_OK; /* Already RUN (or FAULT, not handled here). */

	retval = target_write_u32(bank->target, TC3X_SMU_CMD, TC3X_SMU_CMD_START);
	if (retval != ERROR_OK)
		LOG_WARNING("Failed to start the SMU_core state machine; PFLASH operations may fail with a protection error.");

	return ERROR_OK;
}

static int tc3x_eflash_disable_safety_wdt(struct flash_bank *bank)
{
	struct aurix_eflash_bank *tc3x_bank = bank->driver_priv;
	int retval;

	/* Only PFLASH is affected. The DFLASH banks have their own gate and the
	 * TC4x safety watchdog layout has not been verified here. */
	if (tc3x_bank->dflash)
		return ERROR_OK;

	retval = tc3x_eflash_disable_one_wdt(bank, TC3X_WDTSCON0, TC3X_WDTSCON1, TC3X_WDTSSR, "the safety watchdog");
	if (retval != ERROR_OK)
		return retval;

	/* The CPU0 watchdog is a second, independent problem: it is also armed at
	 * power-on and it resets the core out from under the flash loader, which
	 * is what shows up as "flash write algorithm aborted by target". Same
	 * Disabled Mode target state, reached by running the sequence on-target
	 * instead, since CPU0's watchdog registers do not respond to external
	 * writes (see tc3x_eflash_disable_cpu_wdt_algo()). */
	retval = tc3x_eflash_disable_cpu_wdt_algo(bank, TC3X_WDTCON0, TC3X_WDTCON1, TC3X_WDTCPU0SR, "the CPU0 watchdog");
	if (retval != ERROR_OK)
		return retval;

	/* Third, independent problem: the SMU_core safety state machine itself
	 * also needs to be kicked out of its post-reset START state, or
	 * PFLASH commands fail regardless of either watchdog's state. */
	retval = tc3x_eflash_smu_ensure_run(bank);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("Disabled the watchdogs so that PFLASH operations and the flash loader can run.");
	return ERROR_OK;
}

/*
 * Briefly lift (endinit=0) or restore (endinit=1) Safety ENDINIT write
 * protection around a single PFLASH command sequence.
 *
 * This is a fourth, independent problem from the three handled by
 * tc3x_eflash_disable_safety_wdt() above: even with both watchdogs in
 * Disabled Mode and the SMU in RUN state (all confirmed via register
 * readback on real hardware), PFLASH erase/program commands still failed
 * with "Protection error" until this was added. Per
 * AURIX_TC3xx_Part1_UserManual.pdf section 6.3.1, "Safety Endinit
 * protection": "All command sequences that can modify PFLASH content are
 * protected by Safety Endinit. A PROER is generated if there is an access
 * to the PFLASH without removal of Safety Endinit protection." This is a
 * requirement at command issue time, separate from (and in a sense the
 * opposite of) the watchdog Disabled Mode sequence, which deliberately
 * re-locks ENDINIT=1 as its final, committing step.
 *
 * Even a permanently disabled watchdog keeps monitoring ENDINIT accesses
 * (section 9.4.5.1, "Disabled Mode": "If an access to register WDTxCON0 is
 * performed in Disabled Mode, Time-Out Mode is entered if the access was
 * valid [...] the ENDINIT monitor function [...] will still be active"),
 * so this briefly re-enters Time-Out Mode and restarts the ~100ms-ish
 * overflow countdown described in tc3x_eflash_disable_one_wdt()'s comment.
 * Restoring ENDINIT (a valid service, since it is written with 1) exits
 * Time-Out Mode well within that window and - because the disable request
 * from before is still latched - lands back in Disabled Mode automatically
 * ("the WDT switches back to its former mode of operation", section
 * 9.4.5.1, "Servicing the Watchdog Timer"), rather than Normal Mode.
 *
 * Only ever call this around the handful of fast, queued register writes
 * that make up one command sequence (e.g. the four ocmts_queue_write_u32()
 * calls in aurix_eflash_erase()'s loop body) - never around the slow
 * busy-wait that follows, which routinely takes far longer than the
 * overflow margin. The on-target write algorithm has its own equivalent,
 * tc3x_safety_endinit() in aurix-flash-program.c, run from the target CPU
 * for the same reason rather than reusing this host-side helper.
 */
static int tc3x_eflash_endinit_toggle(struct flash_bank *bank, uint32_t endinit)
{
	uint32_t con0, pw;
	int retval;

	retval = target_read_u32(bank->target, TC3X_WDTSCON0, &con0);
	if (retval != ERROR_OK)
		return retval;

	pw = con0 & TC3X_ENDINIT_PW_MASK;

	retval = target_write_u32(bank->target, TC3X_WDTSCON0, pw | TC3X_ENDINIT_PW_STEP1);
	if (retval != ERROR_OK)
		return retval;

	return target_write_u32(bank->target, TC3X_WDTSCON0,
			pw | (endinit ? TC3X_ENDINIT_SET : 0) | TC3X_ENDINIT_PW_STEP2);
}

/* The DFLASH sector layout and the minimum erase size differ between single
 * ended and complement sensing mode, and this driver only implements the single
 * ended layout.  Refuse the bank instead of erasing with the wrong geometry. */
static int tc3x_eflash_check_sensing_mode(struct flash_bank *bank)
{
	struct aurix_eflash_bank *tc3x_bank = bank->driver_priv;
	uint32_t proconusr;
	int retval;

	if (!tc3x_bank->dflash)
		return ERROR_OK;

	retval = target_read_u32(bank->target, tc3x_bank->reg_addr + DMU_HF_PROCONUSR, &proconusr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Cannot read DMU_HF_PROCONUSR to determine the DFLASH sensing mode.");
		return retval;
	}

	if (proconusr & DMU_PROCONUSR_MODE) {
		LOG_ERROR("DFLASH is configured for complement sensing mode, which this driver does not support.");
		return ERROR_FLASH_BANK_INVALID;
	}

	return ERROR_OK;
}

static int tc3x_eflash_probe(struct flash_bank *bank)
{
	struct aurix_eflash_bank *tc3x_bank = bank->driver_priv;
	uint32_t flash_addr = bank->base;
	uint32_t chipid;
	int retval;

	if (tc3x_bank->probed)
		return ERROR_OK;

	/* aurix_tap_device_type(), not tap->idcode directly: the ifxdap transport
	 * never scans a real IDCODE, so tap->idcode is always 0 and testing it
	 * would make the TC4x branch below permanently unreachable. */
	/*
	 * The chip ID registers are not populated the instant the DAP comes up.
	 * SCU_CHIPID and UCB_CHIPID belong to peripherals that the boot code and
	 * the OCDS bring-up sequence initialise, so a read issued right after the
	 * reset in ifxdap_connect() can legitimately return zero. That is a chip
	 * that is not ready yet, not a wrong chip, but treating it as a mismatch
	 * made auto_probe fail for the whole session: with no bank registered,
	 * every flash command stays unavailable until the next run, which is far
	 * more disruptive than waiting a few milliseconds here.
	 *
	 * Measured on a TC375 Lite Kit: the same register reads 0x00000000 when
	 * probed straight after init and 0x89227580 a moment later, in the same
	 * session. ocmts_wait_io_client() already applies this retry shape to
	 * OSTATE for the same reason.
	 */
	bool is_tc4x = aurix_device_is_tc4x(aurix_tap_device_type(bank->target->tap));
	uint32_t chipid_addr = is_tc4x ? UCB_CHIPID : SCU_CHIPID;

	for (unsigned int tries = 0; tries < AURIX_CHIPID_TRIES; tries++) {
		bool match;

		retval = target_read_u32(bank->target, chipid_addr, &chipid);
		if (retval != ERROR_OK) {
			LOG_ERROR("Cannot read CHIPID register.");
			return retval;
		}

		if (is_tc4x)
			match = ((chipid & UCB_CHIPID_PROD) >> 26) == 13;
		else
			match = (chipid & SCU_CHIPID_CHTEC) == 0x80;

		if (match)
			break;

		if (tries + 1 == AURIX_CHIPID_TRIES) {
			LOG_ERROR("CHIPID register does not match %s (read 0x%08" PRIx32 ").",
				  is_tc4x ? "tc4x with eFLASH" : "tc3x", chipid);
			return ERROR_FAIL;
		}

		alive_sleep(AURIX_CHIPID_DELAY_MS);
	}

	if (!is_tc4x) {
		/* Must happen before any erase or program, and it has to be redone on
		 * every probe because a reset brings the watchdog back. */
		retval = tc3x_eflash_disable_safety_wdt(bank);
		if (retval != ERROR_OK)
			return retval;

		retval = tc3x_eflash_check_pflash_size(bank, chipid);
		if (retval != ERROR_OK)
			return retval;

		retval = tc3x_eflash_check_sensing_mode(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	bank->write_start_alignment = tc3x_bank->burst_size;
	bank->write_end_alignment = tc3x_bank->burst_size;
	bank->num_sectors = bank->size / tc3x_bank->sector_size;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors) {
		LOG_ERROR("Out of memory while allocating flash sectors.");
		return ERROR_FAIL;
	}
	/* The erase and protection state must be left unknown here, they get filled
	 * in by other API calls. aurix_eflash_erase_check() determines the erase
	 * state with the flash controller's verify erased command; determining it
	 * here by reading the sectors would be slow and would also fail, as erased
	 * eflash raises an ECC error on read. aurix_eflash_protect_check() reads
	 * the protection state from the DMU configuration registers. */
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = tc3x_bank->sector_size;
		bank->sectors[i].offset = flash_addr - bank->base;
		flash_addr += tc3x_bank->sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	tc3x_bank->probed = true;

	return ERROR_OK;
}

static int tc3x_eflash_auto_probe(struct flash_bank *bank)
{
	struct aurix_eflash_bank *tc3x_bank = bank->driver_priv;

	if (tc3x_bank->probed)
		return ERROR_OK;

	return tc3x_eflash_probe(bank);
}

/* Logical sector sizes of a TC2x program flash bank, in ascending address
 * order. Eight 16 KiB, eight 32 KiB, four 64 KiB, three 128 KiB and four
 * 256 KiB sectors, which is exactly 2 MiB.
 *
 * This is the one PFLASH geometry value in the driver that is NOT confirmed
 * on silicon. Two independent sources agree on it: the upstream reference
 * driver, and the fact that the sequence must sum to the bank size, so a
 * wrong size in the table would immediately overflow or under-fill the bank
 * and the probe below would truncate. It is also consistent with the board's
 * own factory image, which ends 24 KiB into the bank: the first sector is
 * 16 KiB, so that is a whole sector plus half of the second.
 *
 * phfls_sector_count() guards the table length so that a future part with a
 * smaller bank cannot index past the end of the array. */
static const uint32_t tc2x_pflash_sector_sizes[] = {
	16 * 1024, 16 * 1024, 16 * 1024, 16 * 1024,
	16 * 1024, 16 * 1024, 16 * 1024, 16 * 1024,
	32 * 1024, 32 * 1024, 32 * 1024, 32 * 1024,
	32 * 1024, 32 * 1024, 32 * 1024, 32 * 1024,
	64 * 1024, 64 * 1024, 64 * 1024, 64 * 1024,
	128 * 1024, 128 * 1024, 128 * 1024,
	256 * 1024, 256 * 1024, 256 * 1024, 256 * 1024,
};

/* Program flash erase must stay inside one physical sector: the hardware
 * rejects an erase command that crosses a physical sector boundary, with a
 * command sequence error, and erases nothing at all.
 *
 * MEASURED on a KIT_AURIX_LITE_KIT V1.1 (TC275), 2026-10-03, with
 * multi-sector erase commands issued one per OpenOCD session so no state
 * carried over between them:
 *
 *   - count 2 starting at bank offset 0x140000, i.e. 0x140000..0x1FFFFF,
 *     which straddles the 0x180000 boundary: REJECTED, "Command sequence
 *     error", both neighbouring sectors verified still holding their markers.
 *   - count 2 starting at bank offset 0x100000, i.e. 0x100000..0x17FFFF,
 *     exactly one 512 KiB block: ACCEPTED, and both the 256 KiB logical
 *     sectors inside that block were erased.
 *
 * The only value consistent with both results is 512 KiB, with blocks
 * aligned at 0x000000, 0x080000, 0x100000 and 0x180000 in a 2 MiB bank --
 * four blocks that tile the bank exactly. A 521 KiB physical sector cannot
 * do this: 3 x 521 KiB leaves 496 KiB of the bank that no erase command can
 * ever reach, and 4 x 521 KiB overshoots the end by 36 KiB. The reference
 * driver in tricore-oss-openocd declares 521 KiB in its bank table while its
 * own tc2x_eflash_get_pflash_sector_count() helper hardcodes 512 KiB; the
 * measurement above says the helper is the correct one.
 *
 * Erase checks are limited the same way, because the erase-verify command
 * carries the same "must not cross a physical sector" constraint.
 */
#define TC2X_PHYS_SECTOR_SIZE (512 * 1024)

/* Longest run of logical sectors starting at `first` that still fits inside
 * one physical sector, and never more than up to `last`. Returns a count of
 * at least 1, because the smallest logical sector (16 KiB) is well inside the
 * 512 KiB physical sector. */
static uint32_t tc2x_pflash_erase_sector_count(struct flash_bank *bank, unsigned int first,
												unsigned int last)
{
	for (unsigned int i = first; i <= last; i++) {
		/* The last sector of the bank, or the next one starting a new
		 * physical sector, ends the run. */
		if (i + 1 == bank->num_sectors || i == last)
			return i - first + 1;
		if (bank->sectors[i + 1].offset % TC2X_PHYS_SECTOR_SIZE == 0)
			return i - first + 1;
	}

	return last - first + 1;
}

static int tc2x_eflash_probe(struct flash_bank *bank)
{
	struct aurix_eflash_bank *tc2x_bank = bank->driver_priv;
	uint32_t flash_addr = bank->base;
	uint32_t chipid;
	int retval;

	if (tc2x_bank->probed)
		return ERROR_OK;

	retval = target_read_u32(bank->target, SCU_CHIPID, &chipid);
	if (retval != ERROR_OK) {
		LOG_ERROR("Cannot read tc2x CHIPID register.");
		return retval;
	}

	if ((chipid & SCU_CHIPID_CHTEC) != SCU_CHIPID_CHTEC_TC2X) {
		LOG_ERROR("CHIPID register does not match tc2x (read 0x%08" PRIx32 ").", chipid);
		return ERROR_FAIL;
	}

	/* CHID selects the exact part, which decides how much program flash it
	 * has. The bank size declared in the configuration is checked against
	 * the sector table below either way, so an unknown CHID only means the
	 * bank layout cannot be pre-checked. */
	uint32_t chipid_variant = chipid & SCU_CHIPID_CHID;
	if (chipid_variant != SCU_CHIPID_CHID_TC27X && chipid_variant != SCU_CHIPID_CHID_TC29X) {
		LOG_WARNING("Unknown TC2x CHIPID variant 0x%08" PRIx32
					", treating it like a TC27x for bank layout.",
					chipid_variant);
	}

	bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	bank->write_start_alignment = tc2x_bank->burst_size;
	bank->write_end_alignment = tc2x_bank->burst_size;

	/* Data flash banks are a uniform array of fixed size logical sectors,
	 * so their sector count is the declared bank size over the sector size.
	 * Program flash banks are the variable-size sequence above, truncated at
	 * whatever fits in the declared bank size. */
	if (tc2x_bank->dflash || tc2x_bank->ucb) {
		bank->num_sectors = bank->size / tc2x_bank->sector_size;

		if (!bank->num_sectors) {
			LOG_ERROR("Bank %s is smaller (%" PRIu32 " bytes) than one %u byte sector.",
					  bank->name, bank->size, tc2x_bank->sector_size);
			return ERROR_FLASH_BANK_INVALID;
		}
	} else {
		unsigned int count = 0;
		uint32_t offset = 0;

		for (count = 0; count < ARRAY_SIZE(tc2x_pflash_sector_sizes); count++) {
			offset += tc2x_pflash_sector_sizes[count];
			if (offset > bank->size)
				break;
		}
		bank->num_sectors = count;

		if (!bank->num_sectors) {
			LOG_ERROR("Bank %s is smaller (%" PRIu32 " bytes) than one program flash sector.",
					  bank->name, bank->size);
			return ERROR_FLASH_BANK_INVALID;
		}
	}

	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors) {
		LOG_ERROR("Out of memory while allocating flash sectors.");
		return ERROR_FAIL;
	}

	/* See tc3x_eflash_probe() for why the erase and protection state are
	 * left unknown here. */
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = tc2x_bank->dflash || tc2x_bank->ucb ?
									 tc2x_bank->sector_size :
									 tc2x_pflash_sector_sizes[i];
		bank->sectors[i].offset = flash_addr - bank->base;
		flash_addr += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	tc2x_bank->probed = true;

	return ERROR_OK;
}

static int tc2x_eflash_auto_probe(struct flash_bank *bank)
{
	struct aurix_eflash_bank *tc2x_bank = bank->driver_priv;

	if (tc2x_bank->probed)
		return ERROR_OK;

	return tc2x_eflash_probe(bank);
}

static int tc4x_eflash_probe(struct flash_bank *bank)
{
	struct aurix_eflash_bank *tc4x_bank = bank->driver_priv;
	uint32_t flash_addr = bank->base;
	uint32_t chipid;
	int retval;

	if (tc4x_bank->probed)
		return ERROR_OK;

	retval = target_read_u32(bank->target, UCB_CHIPID, &chipid);
	if (retval != ERROR_OK) {
		LOG_ERROR("Cannot read CHIPID register.");
		return retval;
	}

	if (((chipid & UCB_CHIPID_PROD) >> 26) != 13) {
		LOG_ERROR("CHIPID register does not match tc4x with eFLASH.");
		return ERROR_FAIL;
	}
	/* The TC4x UCB_CHIPID does not encode the flash size in a way that is
	 * documented in the public manual, so the bank size declared in the
	 * configuration cannot be validated here. */

	bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	bank->write_start_alignment = tc4x_bank->burst_size;
	bank->write_end_alignment = tc4x_bank->burst_size;
	bank->num_sectors = bank->size / tc4x_bank->sector_size;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors) {
		LOG_ERROR("Out of memory while allocating flash sectors.");
		return ERROR_FAIL;
	}
	/* See tc3x_eflash_probe() for why these stay unknown here. */
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = tc4x_bank->sector_size;
		bank->sectors[i].offset = flash_addr - bank->base;
		flash_addr += tc4x_bank->sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	tc4x_bank->probed = true;

	return ERROR_OK;
}

static int tc4x_eflash_auto_probe(struct flash_bank *bank)
{
	struct aurix_eflash_bank *tc4x_bank = bank->driver_priv;

	if (tc4x_bank->probed)
		return ERROR_OK;

	return tc4x_eflash_probe(bank);
}

static inline int aurix_eflash_reset_to_read(struct flash_bank *bank)
{
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;

	return ocmts_io_write_u32(ocmts, aurix_bank->cmd_addr + 0x5554, reset_to_read_key);
}

static inline int aurix_eflash_clear_status(struct flash_bank *bank)
{
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;

	return ocmts_io_write_u32(ocmts, aurix_bank->cmd_addr + 0x5554, clear_status_key);
}

struct aurix_eflash_error_desc {
	uint32_t mask;
	const char *text;
};

static const struct aurix_eflash_error_desc tc4x_eflash_errors[] = {
	{ BIT(0), " SRI bus address error" },
	{ BIT(1), " Command sequence error" },
	{ BIT(2), " Protection error" },
	{ BIT(4), " Abort error" },
	{ BIT(5), " Clear error" },
	{ BIT(6), " Program verify error" },
	{ BIT(7), " Erase verify error" },
	{ BIT(8), " Flash operation error" },
	{ BIT(29), " No page mode entry" },
	{ BIT(30), " Flash operation timeout" },
	{ BIT(31), " Flash busy" },
};

static const struct aurix_eflash_error_desc tc3x_eflash_errors[] = {
	{ BIT(0), " Flash Operation Error" },
	{ BIT(1), " Command sequence error" },
	{ BIT(2), " Protection error" },
	{ BIT(3), " Program Verify Error" },
	{ BIT(4), " Erase Verify Error" },
	{ BIT(5), " SRI Bus Address ECC Error" },
	{ BIT(6), " Original Error" },
	{ BIT(30), " Flash operation timeout" },
	{ BIT(31), " Flash busy" },
};

/* TC2x puts its errors at entirely different bit positions from TC3x/TC4x:
 * 11 flash operation, 12 command sequence, 13 protection, 25 program verify,
 * 26 erase verify. Bits 30 and 31 are synthesised by the driver (timeout and
 * busy) rather than read from the register. */
static const struct aurix_eflash_error_desc tc2x_eflash_errors[] = {
	{ BIT(11), " Flash operation error" },
	{ BIT(12), " Command sequence error" },
	{ BIT(13), " Protection error" },
	{ BIT(25), " Program Verify Error" },
	{ BIT(26), " Erase Verify Error" },
	{ BIT(29), " No page mode entry" },
	{ BIT(30), " Flash operation timeout" },
	{ BIT(31), " Flash busy" },
};

/* Decodes the flash error bits into err_str, truncating instead of overflowing. */
static void aurix_eflash_get_error_string(struct aurix_eflash_bank *aurix_bank, uint32_t flash_err,
										  char *err_str, size_t size)
{
	const struct aurix_eflash_error_desc *desc;
	size_t count, len = 0;

	if (!size)
		return;

	if (aurix_bank->tc2x) {
		desc = tc2x_eflash_errors;
		count = ARRAY_SIZE(tc2x_eflash_errors);
	} else if (aurix_bank->tc4x) {
		desc = tc4x_eflash_errors;
		count = ARRAY_SIZE(tc4x_eflash_errors);
	} else {
		desc = tc3x_eflash_errors;
		count = ARRAY_SIZE(tc3x_eflash_errors);
	}

	err_str[0] = '\0';

	for (size_t i = 0; i < count; i++) {
		if (!(flash_err & desc[i].mask))
			continue;

		int written = snprintf(err_str + len, size - len, "%s", desc[i].text);
		if (written < 0 || (size_t)written >= size - len)
			break;
		len += written;
	}
}

static inline int aurix_eflash_handle_error(struct flash_bank *bank, uint32_t sts_bit, int64_t timeout_ms,
											bool *verify_error)
{
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	uint32_t flash_err = 0;
	uint32_t flash_busy = 0xFFFFFFFF;
	int ret = ERROR_OK;
	int64_t start_time = timeval_ms();
	bool timeout_occurred = true;

	while (start_time + timeout_ms > timeval_ms()) {
		ret = ocmts_queue_read_u32(ocmts, aurix_bank->reg_addr + aurix_bank->err_offset, &flash_err);
		if (ret)
			goto status_err;
		ret = ocmts_queue_read_u32(ocmts, aurix_bank->reg_addr + aurix_bank->sts_offset, &flash_busy);
		if (ret)
			goto status_err;
		ret = ocmts_run(ocmts);

status_err:
		if (ret) {
			LOG_ERROR("Failed to read flash operation status");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		/* On TC2x the status and the error bits live in the same word, so
		 * only the error bits are meaningful as an error. Masking here is
		 * mandatory: the busy bits occupy 1..6 and the state bits 7,8 and
		 * 9/10, and every one of them is set during a normal operation. */
		if (aurix_bank->tc2x)
			flash_err &= TC2X_FSR_ERRORS;
		if (aurix_bank->tc4x) {
			if (flash_busy & (1 << 31)) {
				timeout_occurred = false;
				break;
			}
		} else {
			if (!(flash_busy & (1 << sts_bit))) {
				timeout_occurred = false;
				break;
			}
		}
		jtag_sleep(100);
	}

	if (timeout_occurred)
		flash_err |= (1 << 30);

	if (flash_err) {
		/* Verify and fatal bits are at different positions per family:
		 * erase-verify is 7 on TC4x, 4 on TC3x and 26 on TC2x; the fatal
		 * "cannot recover" bits are 8|5 on TC4x, 0 on TC3x and 11 on TC2x
		 * (a flash operation error leaves the controller in an
		 * indeterminate state). */
		uint32_t verify_bit = aurix_bank->tc2x ? 26 : aurix_bank->tc4x ? 7 : 4;
		uint32_t fatal_mask = aurix_bank->tc2x ? BIT(11) :
							   aurix_bank->tc4x ? ((1 << 8) | (1 << 5)) : (1 << 0);
		if (verify_error) {
			if (flash_err & (1 << verify_bit)) {
				*verify_error = true;
				ret = aurix_eflash_clear_status(bank);
				if (ret)
					LOG_ERROR("Failed to clear flash status. Please reset device to continue");
				return ERROR_OK;
			}
		}
		char err_str[256] = {0};
		aurix_eflash_get_error_string(aurix_bank, flash_err, err_str, sizeof(err_str));
		LOG_ERROR("Flash operation failed with error:%s", err_str);
		if (flash_err & fatal_mask)
			LOG_ERROR("Critical flash error. Please reset device to continue");
		else if (flash_err & (aurix_bank->tc2x ? (BIT(12) | BIT(13)) : ((1 << 1) | (1 << 2))))
			ret = aurix_eflash_reset_to_read(bank);
		else
			ret = aurix_eflash_clear_status(bank);
		if (ret)
			LOG_ERROR("Failed to clear flash status. Please reset device to continue");
		ret = ERROR_FLASH_OPERATION_FAILED;
	} else {
		if (verify_error)
			*verify_error = false;
	}

	return ret;
}

static inline int aurix_eflash_check_busy(struct flash_bank *bank)
{
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	uint32_t flash_sts;
	int ret = ERROR_OK;

	for (unsigned int tries = 0; tries < AURIX_EFLASH_CHECK_BUSY_TRIES; tries++) {
		ret = ocmts_io_read_u32(ocmts, aurix_bank->reg_addr + aurix_bank->sts_offset, &flash_sts);
		if (ret) {
			LOG_ERROR("Failed to read flash status");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		if (!(flash_sts & (1 << aurix_bank->busy_bit)))
			return ERROR_OK;
		if (tries + 1 < AURIX_EFLASH_CHECK_BUSY_TRIES)
			jtag_sleep(AURIX_EFLASH_CHECK_BUSY_DELAY_US);
	}
	LOG_ERROR("Flash is busy with another operation.");
	return ERROR_FLASH_BUSY;
}

static inline int aurix_eflash_enter_page_mode(struct flash_bank *bank, bool dflash)
{
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	uint32_t flash_sts;
	uint32_t flash_err;

	uint32_t page_mode_key = dflash ? page_mode_d_key : page_mode_p_key;
	int ret = ocmts_io_write_u32(ocmts, aurix_bank->cmd_addr + 0x5554, page_mode_key);
	if (ret)
		goto err;
	ret = ocmts_queue_read_u32(ocmts, aurix_bank->reg_addr + aurix_bank->err_offset, &flash_err);
	if (ret)
		goto err;
	ret = ocmts_queue_read_u32(ocmts, aurix_bank->reg_addr + aurix_bank->sts_offset, &flash_sts);
	if (ret)
		goto err;
	ret = ocmts_run(ocmts);
	if (ret)
		goto err;
	/* TC2x: status and errors share one word, so drop the state bits before
	 * deciding whether the command failed. */
	if (aurix_bank->tc2x)
		flash_err &= TC2X_FSR_ERRORS;
	if (!(flash_sts & (1 << aurix_bank->page_bit)))
		flash_err |= (1 << 29);
	if (flash_err) {
		char err_str[256] = {0};
		aurix_eflash_get_error_string(aurix_bank, flash_err, err_str, sizeof(err_str));
		LOG_ERROR("Failed to enter page mode with error: %s", err_str);
		ret = aurix_eflash_reset_to_read(bank);
		if (ret)
			LOG_WARNING("Failed to reset flash to read mode. Please reset device to continue");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
err:
	LOG_ERROR("Failed to execute enter page sequence");
	/* The page-mode key has already been written, so the bank is in page
	 * mode; put it back into read mode as the error path above does. */
	if (aurix_eflash_reset_to_read(bank) != ERROR_OK)
		LOG_WARNING("Failed to reset flash to read mode. Please reset device to continue");
	return ret;
}

/* --- TC2x ENDINIT handling ---------------------------------------------
 *
 * TC2x program flash erase and program commands are protected by ENDINIT.
 * While ENDINIT is set the flash sequencer ignores them and raises a
 * protection error (FSR bit 13) instead.
 *
 * Lifting it is the documented two-step password write to WDTS_CON0: write
 * the password with bit 0 clear, then write it again with bit 1 set and
 * bit 0 carrying the wanted ENDINIT value. Both writes of a pair must be
 * derived from the register value read BEFORE the pair, which is why
 * aurix_eflash_tc2x_endinit_values() reads it once and returns all three
 * values together. A pair must also be re-derived for every operation: the
 * register changes after each toggle, so reusing the values across a
 * multi-sector erase would write a stale password on the second sector.
 *
 * Verified on silicon (KIT_AURIX_LITE_KIT V1.1, TC275, 2026-10-03): the
 * first erase after power-up succeeds while ENDINIT reads 0, the on-target
 * loader in contrib/loaders/flash/aurix/aurix-flash-program.c then restores
 * ENDINIT to 1 when it finishes a page program, and every later erase then
 * fails with "Protection error" until ENDINIT is lifted here. DFLASH and the
 * UCBs are not protected this way, which matches aurix-flash-program.c.
 *
 * These helpers only QUEUE their writes. They must not call ocmts_run():
 * they are used in the middle of a flash command sequence, and flushing
 * there would split the sequence and leave the sequencer in the wrong state.
 */
/*
 * aurix-mac-dap: every flash command that names a target address (erase,
 * program, verify-erased) goes through this first. A range touching a UCB is
 * refused before anything reaches the command interface, so neither a
 * configuration that declares UCB banks nor an image that carries a BMHD can
 * get one modified. See aurix_range_ucb().
 */
static int aurix_eflash_refuse_ucb(const char *what, uint64_t addr, uint64_t len)
{
	const char *ucb = aurix_range_ucb(addr, len);

	if (!ucb)
		return ERROR_OK;
	LOG_ERROR("Refusing to %s 0x%08" PRIx64 "..0x%08" PRIx64 ": it touches %s, which this build never modifies",
			  what, addr, addr + len - 1, ucb);
	return ERROR_FLASH_OPERATION_FAILED;
}

/* Bytes covered by sectors [first, first + count) of @a bank. */
static uint64_t aurix_eflash_sector_span(struct flash_bank *bank, unsigned int first, uint32_t count)
{
	uint64_t span = 0;

	for (unsigned int i = first; i < first + count && i < bank->num_sectors; i++)
		span += bank->sectors[i].size;
	return span;
}

static int aurix_eflash_tc2x_endinit_values(struct ocmts *ocmts, uint32_t *unlock, uint32_t *clear,
											uint32_t *set)
{
	uint32_t value = 0;

	if (ocmts_io_read_u32(ocmts, TC2X_WDTS_CON0, &value) != ERROR_OK) {
		LOG_ERROR("Failed to read TC2X WDTS_CON0 before toggling ENDINIT.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Bits 7:2 of the low byte carry the password; bit 0 is ENDINIT and
	 * bit 1 selects that the write is a state change. */
	const uint32_t password = (value & 0xFFFFFF00u) | (~value & 0xFCu);

	*unlock = password | 0x01u;
	*clear = password | 0x02u; /* endinit = 0 */
	*set = password | 0x03u;	 /* endinit = 1 */

	return ERROR_OK;
}

/* Queues one complete two-step toggle. Must be called with no other flash
 * command sequence open. */
static int aurix_eflash_tc2x_queue_endinit(struct ocmts *ocmts, uint32_t unlock, uint32_t state)
{
	int ret = ocmts_queue_write_u32(ocmts, TC2X_WDTS_CON0, &unlock);

	if (ret)
		return ret;

	return ocmts_queue_write_u32(ocmts, TC2X_WDTS_CON0, &state);
}

/* Reads WDTS_CON0 back until ENDINIT actually reads `want`, or gives up.
 *
 * The AURIX user manuals state that clearing an ENDINIT bit takes some time
 * and that Endinit-protected registers must only be accessed once the bit is
 * really cleared, recommending a read-back. That guidance is written for the
 * TC3xx part manual (Section 9.4.4, WDTSCON0 at SRU offset 0x02A8 = 0xF00362A8
 * on TC3xx). THE TC3xx MANUAL DOES NOT DOCUMENT THE TC2x WDTS_CON0 AT 0x0F0
 * (0xF00360F0), so it cannot be used to confirm the TC2x register layout; the
 * layout here comes from contrib/loaders/flash/aurix/tc2x-flash.h instead.
 *
 * The read-back is applied anyway, because it is free, because it is what the
 * independent on-target loader already does for the same register
 * (tc2x_safety_endinit() polls WDTS_CON0 in a loop,
 * contrib/loaders/flash/aurix/aurix-flash-program.c), and because touching an
 * Endinit-protected register before the bit has settled is exactly the kind
 * of race that fails intermittently rather than reproducibly.
 *
 * A timeout here is reported, not ignored: it means the toggle did not take
 * effect, and every following erase would silently fail with a protection
 * error.
 */
static int aurix_eflash_tc2x_wait_endinit(struct ocmts *ocmts, uint32_t want)
{
	const int64_t deadline = timeval_ms() + 100;

	while (timeval_ms() < deadline) {
		uint32_t value = 0;

		if (ocmts_io_read_u32(ocmts, TC2X_WDTS_CON0, &value) != ERROR_OK) {
			LOG_ERROR("Failed to read back TC2X WDTS_CON0 while toggling ENDINIT.");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if ((value & 1u) == want)
			return ERROR_OK;

		jtag_sleep(10);
	}

	LOG_ERROR("TC2X ENDINIT did not settle to %u within the timeout.", want);

	return ERROR_FLASH_OPERATION_FAILED;
}

int aurix_eflash_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	int ret;

	ret = aurix_eflash_refuse_ucb("erase bank", bank->base, bank->size);
	if (ret)
		return ret;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* A reset re-arms the safety watchdog, and while it is armed the flash
	 * controller answers every erase with "Protection error". The probe at
	 * flash init disables it, but "reset halt" between init and the erase puts
	 * it back, so re-apply it here. See tc3x_eflash_disable_safety_wdt().
	 * TC2x is excluded: every register that function touches (WDTSCON0,
	 * WDTCON0/1, WDTCPU0SR, SMU_CMD/SMU_DBG) is a TC3x-specific address, and
	 * TC2x's own tc2x_eflash_probe() does not call it either. Hardware
	 * testing (see aurix_eflash_tc2x_endinit_values()'s comment) found
	 * lifting ENDINIT alone sufficient to erase/program a TC275 Lite Kit, so
	 * TC2x apparently does not need an equivalent step here. */
	if (!aurix_bank->tc4x && !aurix_bank->dflash && !aurix_bank->tc2x) {
		ret = tc3x_eflash_disable_safety_wdt(bank);
		if (ret != ERROR_OK)
			return ret;
	}

	ret = aurix_eflash_check_busy(bank);
	if (ret) {
		LOG_ERROR("Flash erase failed: flash is busy");
		return ret;
	}

	ret = aurix_eflash_clear_status(bank);
	if (ret) {
		LOG_ERROR("Flash erase failed: failed to clear flash status");
		return ret;
	}

	while (first <= last) {
		uint32_t sector_count;
		unsigned int failed_first = first;
		/* Program flash erase is ENDINIT protected on TC2x. Re-derived for
		 * every sector because the password depends on the current register
		 * value, which the previous iteration changed. DFLASH and the UCBs
		 * are not protected, so they skip this entirely. */
		bool lift_endinit = aurix_bank->tc2x && !aurix_bank->dflash && !aurix_bank->ucb;
		uint32_t endinit_unlock = 0, endinit_clear = 0, endinit_set = 0;
		if (lift_endinit) {
			ret = aurix_eflash_tc2x_endinit_values(ocmts, &endinit_unlock, &endinit_clear, &endinit_set);
			if (ret)
				return ret;
		}

		if (aurix_bank->tc2x) {
			/* Erase as many logical sectors as fit inside one physical
			 * sector. A multi-sector erase must not cross a physical sector
			 * boundary; the hardware rejects one that does, with a command
			 * sequence error, and erases nothing. */
			sector_count = tc2x_pflash_erase_sector_count(bank, first, last);
		} else if (aurix_bank->tc4x) {
			/* Limit to logical sector erase count. */
			sector_count = MIN(aurix_bank->num_erase_sectors, last - first + 1);
		} else {
			/* Align sector count to physical sector boundary */
			uint32_t sectors_to_boundary =
				MIN(last - first + 1, aurix_bank->phys_sector_size / aurix_bank->sector_size -
										  (first % (aurix_bank->phys_sector_size / aurix_bank->sector_size)));
			/* Limit to logical sector erase count. */
			sector_count = MIN(aurix_bank->num_erase_sectors, MIN(last - first + 1, sectors_to_boundary));
		}

		/* Put address always in segment 0xA */
		uint32_t addr = (~0xF0000000 & (bank->base + bank->sectors[first].offset)) + 0xA0000000;

		/* See tc3x_eflash_endinit_toggle()'s comment: PFLASH command
		 * sequences need Safety ENDINIT actively removed, on top of (and
		 * separate from) both watchdogs being in Disabled Mode. DFLASH and
		 * TC4x are not affected. TC2x is excluded here too and handled by
		 * the lift_endinit block below instead: its Safety WDT CON0 lives
		 * at a different address (TC2X_WDTS_CON0, not TC3X_WDTSCON0), and
		 * the clear/set pair must be queued inside this same OCMTS sequence
		 * rather than issued as separate immediate writes. */
		if (!aurix_bank->tc4x && !aurix_bank->dflash && !aurix_bank->tc2x) {
			ret = tc3x_eflash_endinit_toggle(bank, 0);
			if (ret)
				goto sequence_err;
		}

		if (lift_endinit) {
			/* The clear has to complete before the erase command, which
			 * touches an Endinit-protected register, so this pair is run on
			 * its own and then read back before anything else is queued.
			 * See aurix_eflash_tc2x_wait_endinit(). */
			ret = aurix_eflash_tc2x_queue_endinit(ocmts, endinit_unlock, endinit_clear);
			if (ret)
				goto sequence_err;
			ret = ocmts_run(ocmts);
			if (ret)
				goto sequence_err;
			ret = aurix_eflash_tc2x_wait_endinit(ocmts, 0);
			if (ret)
				goto sequence_err;
		}

		ret = aurix_eflash_refuse_ucb("erase", addr, aurix_eflash_sector_span(bank, first, sector_count));
		if (ret)
			goto sequence_err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAA50, &addr);
		if (ret)
			goto sequence_err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAA58, &sector_count);
		if (ret)
			goto sequence_err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &(uint32_t){0x80});
		if (ret)
			goto sequence_err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &(uint32_t){0x50});
		if (ret)
			goto sequence_err;

		if (lift_endinit) {
			ret = aurix_eflash_tc2x_queue_endinit(ocmts, endinit_unlock, endinit_set);
			if (ret)
				goto sequence_err;
		}

		ret = ocmts_run(ocmts);
		/* Hint: User manual requires Wait for 2*1/fFSI ns (DFlash) or 3*1/fFSI + 8*1/fSRI ns (PFlash)
		 * The OCMTS delay for the next instruction is sufficient. */
sequence_err:
		/* Always try to restore ENDINIT, even if something above failed,
		 * so a failed erase never leaves Safety ENDINIT removed. Only
		 * report the restore step's own failure if nothing earlier already
		 * failed, so a real erase-sequence error is never masked. TC2x is
		 * excluded: its restore is already queued inside the lift_endinit
		 * block above, in the same OCMTS sequence, and calling
		 * tc3x_eflash_endinit_toggle() here too would write TC3X_WDTSCON0,
		 * the wrong register for this family. */
		if (!aurix_bank->tc4x && !aurix_bank->dflash && !aurix_bank->tc2x) {
			int endinit_ret = tc3x_eflash_endinit_toggle(bank, 1);
			if (ret == ERROR_OK)
				ret = endinit_ret;
		}
		if (ret) {
			ret = aurix_eflash_reset_to_read(bank);
			if (ret)
				LOG_WARNING("Failed to reset flash to read mode. Please reset device to continue");
			LOG_ERROR("Flash erase failed: failed to execute erase sequence address: 0x%08x, "
					  "sector count: %u",
					  addr, sector_count);
			goto restore_endinit;
		}
		first += sector_count;

		ret = aurix_eflash_handle_error(bank, aurix_bank->busy_bit, aurix_bank->erase_timeout_ms, NULL);
		if (ret) {
			LOG_ERROR("Flash erase failed: operation failed at address 0x%08" PRIx64,
					  bank->base + bank->sectors[failed_first].offset);
			goto restore_endinit;
		}

		if (lift_endinit) {
			ret = aurix_eflash_tc2x_wait_endinit(ocmts, 1);
			if (ret)
				goto restore_endinit;
		}
		continue;

restore_endinit:
		/* Never leave ENDINIT cleared on a safety-critical register, even on
		 * the failure path. */
		if (lift_endinit) {
			uint32_t u = 0, c = 0, s = 0;
			int restore_ret = ERROR_FAIL;

			if (aurix_eflash_tc2x_endinit_values(ocmts, &u, &c, &s) == ERROR_OK &&
				aurix_eflash_tc2x_queue_endinit(ocmts, u, s) == ERROR_OK)
				restore_ret = ocmts_run(ocmts);
			if (restore_ret == ERROR_OK)
				LOG_WARNING("Restored ENDINIT to 1 after a failed erase.");
			else
				LOG_ERROR("Failed to restore ENDINIT after the erase failed.");
		}
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

int aurix_eflash_erase_check(struct flash_bank *bank)
{
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	int ret;
	size_t sector;

	/* Verify-erased is not destructive, but no command sequence aimed at a
	 * UCB is sent at all. */
	ret = aurix_eflash_refuse_ucb("erase-check bank", bank->base, bank->size);
	if (ret)
		return ret;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = aurix_eflash_check_busy(bank);
	if (ret) {
		LOG_ERROR("Flash verify failed: flash is busy");
		return ret;
	}

	ret = aurix_eflash_clear_status(bank);
	if (ret) {
		LOG_ERROR("Flash verify failed: failed to clear flash status");
		return ret;
	}

	/* The erase-verify command carries the same "must not cross a physical
	 * sector" constraint as the erase command, so TC2x walks the same
	 * physical-sector-aligned runs that aurix_eflash_erase() does. The run
	 * lengths are NOT uniform (16, 4, 3 and 4 logical sectors for a 2 MiB
	 * TC2x program flash bank), so the count is recomputed per iteration
	 * rather than used as a fixed stride. */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Put address always in segment 0xA */
		uint32_t addr = (~0xF0000000 & (bank->base + bank->sectors[sector].offset)) + 0xA0000000;
		uint32_t verify_sectors =
			aurix_bank->tc2x ?
				tc2x_pflash_erase_sector_count(bank, sector, bank->num_sectors - 1) :
				aurix_bank->num_erase_sectors;

		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAA50, &addr);
		if (ret)
			goto sequence_err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAA58, &verify_sectors);
		if (ret)
			goto sequence_err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &(uint32_t){0x80});
		if (ret)
			goto sequence_err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &(uint32_t){0x5F});
		if (ret)
			goto sequence_err;
		ret = ocmts_run(ocmts);
sequence_err:
		if (ret) {
			ret = aurix_eflash_reset_to_read(bank);
			if (ret)
				LOG_WARNING("Failed to reset flash to read mode. Please reset device to continue");
			LOG_ERROR("Flash verify failed: failed to execute erase sequence address: 0x%08x, "
					  "sector count: %u",
					  addr, aurix_bank->num_erase_sectors);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bool verify_error = false;
		ret = aurix_eflash_handle_error(bank, aurix_bank->busy_bit, aurix_bank->erase_timeout_ms, &verify_error);
		if (ret) {
			LOG_ERROR("Flash verify failed: operation failed at address 0x%08" PRIx64,
					  bank->base + bank->sectors[sector].offset);
			return ret;
		}

		uint32_t i;
		if (!verify_error) {
			for (i = sector; i < sector + verify_sectors && i < bank->num_sectors; i++)
				bank->sectors[i].is_erased = 1;
		} else {
			uint8_t failed_sector = 0;
			if (aurix_bank->tc4x) {
				ret = ocmts_io_read_u8(ocmts, aurix_bank->fsi_addr + 0x1, &failed_sector);
				if (ret) {
					LOG_ERROR("Failed to read failed sector address");
					return ERROR_FLASH_OPERATION_FAILED;
				}
			}

			for (i = sector + failed_sector; i < sector + verify_sectors && i < bank->num_sectors; i++) {
				/* Put address always in segment 0xA */
				addr = (~0xF0000000 & (bank->base + bank->sectors[i].offset)) + 0xA0000000;
				uint32_t single_sector = 1;
				ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAA50, &addr);
				if (ret)
					goto sequence_err2;
				ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAA58, &single_sector);
				if (ret)
					goto sequence_err2;
				ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &(uint32_t){0x80});
				if (ret)
					goto sequence_err2;
				ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &(uint32_t){0x5F});
				if (ret)
					goto sequence_err2;
				ret = ocmts_run(ocmts);
sequence_err2:
				if (ret) {
					ret = aurix_eflash_reset_to_read(bank);
					if (ret)
						LOG_WARNING("Failed to reset flash to read mode. Please reset device to continue");
					LOG_ERROR("Flash verify failed: failed to execute erase sequence address: 0x%08x, "
							  "sector count: %u",
							  addr, aurix_bank->num_erase_sectors);
					return ERROR_FLASH_OPERATION_FAILED;
				}

				ret =
					aurix_eflash_handle_error(bank, aurix_bank->busy_bit, aurix_bank->erase_timeout_ms, &verify_error);
				if (ret) {
					LOG_ERROR("Flash verify failed: operation failed at address 0x%08" PRIx64,
							  bank->base + bank->sectors[sector].offset);
					return ret;
				}
				bank->sectors[i].is_erased = verify_error ? 0 : 1;
			}
		}

		sector += verify_sectors - 1;
	}
	return ERROR_OK;
}

static void aurix_eflash_protection_unknown(struct flash_bank *bank)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = -1;
}

static void aurix_eflash_protection_set(struct flash_bank *bank, int protected)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = protected;
}

int aurix_eflash_protect_check(struct flash_bank *bank)
{
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t procon;
	int ret;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The UCBs hold the protection configuration rather than being described
	 * by it, and the TC4x DMU lays its configuration out differently. Report
	 * those as unknown instead of reporting a wrong state.
	 *
	 * TC2x is in the same position for a different reason: its protection
	 * state is a single register with no per-sector and no per-bank
	 * breakdown, so it cannot be mapped onto the per sector flags this
	 * interface is asked for. Reading the TC3x/TC4x DMU layout here would
	 * return unrelated registers. */
	if (aurix_bank->ucb || aurix_bank->tc4x || aurix_bank->tc2x) {
		aurix_eflash_protection_unknown(bank);
		return ERROR_OK;
	}

	if (aurix_bank->dflash) {
		/* PROCONDF only describes DFLASH0. DFLASH1 belongs to the HSM and is
		 * not visible here. */
		if (aurix_bank->dflash1) {
			aurix_eflash_protection_unknown(bank);
			return ERROR_OK;
		}

		ret = target_read_u32(target, aurix_bank->reg_addr + DMU_HF_PROCONDF, &procon);
		if (ret != ERROR_OK) {
			LOG_ERROR("Cannot read DMU_HF_PROCONDF register.");
			return ret;
		}

		/* A single lock bit covers all EEPROM sectors of the bank. */
		aurix_eflash_protection_set(bank, (procon & DMU_PROCONDF_L) ? 1 : 0);
		return ERROR_OK;
	}

	ret = target_read_u32(target, aurix_bank->reg_addr + DMU_HF_PROCONPF, &procon);
	if (ret != ERROR_OK) {
		LOG_ERROR("Cannot read DMU_HF_PROCONPF register.");
		return ret;
	}

	/* Read protection covers the whole PFLASH and implies write protection,
	 * so the per sector lock bits do not need to be looked at. */
	if (procon & DMU_PROCON_RPRO) {
		aurix_eflash_protection_set(bank, 1);
		return ERROR_OK;
	}

	if (bank->num_sectors > DMU_PROCONP_REGS * 32) {
		LOG_ERROR("Bank has %u sectors, but only %u lock bits are available.",
				  bank->num_sectors, DMU_PROCONP_REGS * 32);
		aurix_eflash_protection_unknown(bank);
		return ERROR_FAIL;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i += 32) {
		target_addr_t reg = aurix_bank->reg_addr + DMU_HP_PROCONP(aurix_bank->pflash_index, i / 32);

		ret = target_read_u32(target, reg, &procon);
		if (ret != ERROR_OK) {
			LOG_ERROR("Cannot read DMU_HP_PROCONP%u%u register.", aurix_bank->pflash_index, i / 32);
			aurix_eflash_protection_unknown(bank);
			return ret;
		}

		for (unsigned int j = 0; j < 32 && i + j < bank->num_sectors; j++)
			bank->sectors[i + j].is_protected = (procon & (1U << j)) ? 1 : 0;
	}

	return ERROR_OK;
}

static const uint8_t tc2x_pflash_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc2x-pflash-program.inc"
};

static const uint8_t tc2x_dflash_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc2x-dflash-program.inc"
};

static const uint8_t tc3x_pflash_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc3x-pflash-program.inc"
};

static const uint8_t tc3x_dflash_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc3x-dflash-program.inc"
};

static const uint8_t tc4x_pflash_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc4x-pflash-program.inc"
};

static const uint8_t tc4x_dflash_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc4x-dflash-program.inc"
};

static const uint8_t tc4x_pflashcs_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc4x-pflashcs-program.inc"
};

static const uint8_t tc4x_dflashcs_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc4x-dflashcs-program.inc"
};

/* Start a low level flash write for the specified region */
static int aurix_eflash_write_algo(struct flash_bank *bank, uint32_t address, const uint8_t *buffer, uint32_t bytes)
{
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;
	struct target *target = bank->target;
	struct reg_param reg_params[5];
	uint32_t buffer_size = 0x8000;
	int ret;
	struct working_area *source;
	const uint8_t *code;
	size_t code_size;

	/* Last host-side check before the on-target loader is handed the range. */
	ret = aurix_eflash_refuse_ucb("program", address, bytes);
	if (ret)
		return ret;

	/*
	 * Select the loader variant matching this bank's family (TC2x and TC3x
	 * have a single command interface; TC4x has one per module, "cs" being
	 * the second/CSRM one), and bank kind (dflash is also true for UCB banks,
	 * which use the DFLASH command sequence).
	 */
	if (aurix_bank->tc2x) {
		code = aurix_bank->dflash ? tc2x_dflash_write_code : tc2x_pflash_write_code;
		code_size = aurix_bank->dflash ? sizeof(tc2x_dflash_write_code) : sizeof(tc2x_pflash_write_code);
	} else if (!aurix_bank->tc4x) {
		code = aurix_bank->dflash ? tc3x_dflash_write_code : tc3x_pflash_write_code;
		code_size = aurix_bank->dflash ? sizeof(tc3x_dflash_write_code) : sizeof(tc3x_pflash_write_code);
	} else if (aurix_bank->cmd_addr == 0xF80C0000) {
		code = aurix_bank->dflash ? tc4x_dflashcs_write_code : tc4x_pflashcs_write_code;
		code_size = aurix_bank->dflash ? sizeof(tc4x_dflashcs_write_code) : sizeof(tc4x_pflashcs_write_code);
	} else {
		code = aurix_bank->dflash ? tc4x_dflash_write_code : tc4x_pflash_write_code;
		code_size = aurix_bank->dflash ? sizeof(tc4x_dflash_write_code) : sizeof(tc4x_pflash_write_code);
	}

	ret = target_write_buffer(target, 0x70100000, code_size, code);
	if (ret != ERROR_OK)
		return ret;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			LOG_WARNING("No large enough working area available, can't do block "
						"memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "a4", 32, PARAM_OUT); /* buffer start */
	init_reg_param(&reg_params[1], "d4", 32, PARAM_OUT); /* buffer size */
	init_reg_param(&reg_params[2], "d5", 32, PARAM_OUT); /* addr */
	init_reg_param(&reg_params[3], "d6", 32, PARAM_OUT); /* size */
	init_reg_param(&reg_params[4], "d2", 32, PARAM_IN);	 /* return */

	buf_set_u32(reg_params[0].value, 0, 32, (uint32_t)source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, bytes);

	ret = target_run_flash_async_algorithm(target, buffer, bytes / 8, 8, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
										   source->address, source->size, 0x70100000, 0, NULL);

	if (ret == ERROR_FLASH_OPERATION_FAILED) {
		uint32_t status = buf_get_u32(reg_params[4].value, 0, 32);
		char err_str[256] = {0};
		aurix_eflash_get_error_string(aurix_bank, status, err_str, sizeof(err_str));
		LOG_ERROR("Flash algorithm failed: %s", err_str);
	}
	target_free_working_area(target, source);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return ret;
}

static int aurix_eflash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct aurix_eflash_bank *aurix_bank = bank->driver_priv;
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	uint32_t page_offset = 0;
	int ret;

	ret = aurix_eflash_refuse_ucb("program bank", bank->base, bank->size);
	if (ret == ERROR_OK)
		ret = aurix_eflash_refuse_ucb("program", (uint64_t)bank->base + offset, count);
	if (ret)
		return ret;

	/* Same reason as in aurix_eflash_erase(): "program" resets the target
	 * between the erase and the write, which re-arms the safety watchdog, and
	 * the on-target loader is then reset out from under itself part way
	 * through programming. TC2x is excluded; see aurix_eflash_erase()'s
	 * identical guard for why. */
	if (!aurix_bank->tc4x && !aurix_bank->dflash && !aurix_bank->tc2x) {
		ret = tc3x_eflash_disable_safety_wdt(bank);
		if (ret != ERROR_OK)
			return ret;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & (aurix_bank->page_size - 1) || count % aurix_bank->page_size != 0)
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;

	if (aurix_bank->fallback_mode)
		goto fallback;

	ret = aurix_eflash_write_algo(bank, bank->base + offset, buffer, count);
	if (ret == ERROR_OK)
		return ERROR_OK;
	else if (ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
		return ret;
	LOG_WARNING("No enough resources to run flash write algorithm, fallback to "
				"slow flash write sequence.");

fallback:
	ret = aurix_eflash_check_busy(bank);
	if (ret)
		return ret;

	while (page_offset < count) {
		const bool burst_mode =
			(page_offset % aurix_bank->burst_size) == 0 && (count - page_offset >= aurix_bank->burst_size);
		const uint32_t copy_size = burst_mode ? aurix_bank->burst_size : aurix_bank->page_size;
		uint32_t i;
		/* Declared before the first 'goto err' so the error path never
		 * reads it uninitialised. */
		uint32_t addr = 0;

		ret = aurix_eflash_clear_status(bank);
		if (ret) {
			LOG_ERROR("Flash program failed: failed to clear flash status");
			return ret;
		}

		ret = aurix_eflash_enter_page_mode(bank, aurix_bank->dflash);
		if (ret) {
			LOG_ERROR("Flash program failed: failed to enter page mode");
			if (aurix_eflash_reset_to_read(bank) != ERROR_OK)
				LOG_ERROR("Failed to reset flash to read mode. Please reset device to continue");
			return ret;
		}

		if (aurix_bank->tc4x) {
			for (i = 0; i < copy_size; i += 4) {
				uint32_t word = le_to_h_u32(buffer + page_offset + i);

				ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0x55F4, &word);
				if (ret)
					goto err;
			}
			/* Clear status to reset request done from load page*/
			ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0x5554, &clear_status_key);
			if (ret) {
				if (aurix_eflash_reset_to_read(bank) != ERROR_OK)
					LOG_ERROR("Failed to reset flash to read mode. Please reset device to continue");
				LOG_ERROR("Flash program failed: failed to clear flash status");
				return ERROR_FLASH_OPERATION_FAILED;
			}
		} else {
			for (i = 0; i < copy_size; i += 4) {
				uint32_t word = le_to_h_u32(buffer + page_offset + i);

				ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0x55F0 + ((i % 8) == 0 ? 0 : 4),
											&word);
				if (ret)
					goto err;
			}
		}

		/* Put address always in segment 0xA */
		addr = (~0xF0000000 & (bank->base + offset + page_offset)) + 0xA0000000;
		page_offset += copy_size;

		/* See tc3x_eflash_endinit_toggle()'s comment: the write command
		 * sequence below needs Safety ENDINIT actively removed, same as
		 * aurix_eflash_erase(). The page load writes above are not gated by
		 * it. TC2x is excluded here: its PFLASH program flash is also
		 * ENDINIT protected, but the toggle is deliberately NOT done here
		 * for it. In the algo path the on-target loader lifts and restores
		 * ENDINIT itself around the write command (tc2x_safety_endinit(),
		 * contrib/loaders/flash/aurix/aurix-flash-program.c), and in this
		 * fallback path a host-side toggle cannot be placed correctly: it
		 * would have to sit between the 0xA0 and the 0xAA/0x7A write
		 * commands, and the ocmts transaction cannot be flushed there
		 * without splitting the sequence. If ENDINIT really is set, the
		 * sequencer raises a protection error (FSR bit 13) which
		 * aurix_eflash_handle_error() below reports, so the failure is
		 * visible rather than silent. */
		if (!aurix_bank->tc4x && !aurix_bank->dflash && !aurix_bank->tc2x) {
			ret = tc3x_eflash_endinit_toggle(bank, 0);
			if (ret)
				goto err;
		}

		/* Execute page write sequence */
		ret = aurix_eflash_refuse_ucb("program", addr, copy_size);
		if (ret)
			goto err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAA50, &addr);
		if (ret)
			goto err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAA58, &(uint32_t){0});
		if (ret)
			goto err;
		ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &(uint32_t){0xA0});
		if (ret)
			goto err;

		/* Check for burst sequence. The burst command differs per family:
		 * TC2x uses 0x7A, TC3x and TC4x use 0xA6. The single-page command
		 * is 0xAA everywhere. */
		if (burst_mode) {
			uint32_t burst_cmd = aurix_bank->tc2x ? TC2X_FLASH_WRITE_BURST_CMD :
														TC3X_FLASH_WRITE_BURST_CMD;
			ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &burst_cmd);
			if (ret)
				goto err;
		} else {
			ret = ocmts_queue_write_u32(ocmts, aurix_bank->cmd_addr + 0xAAA8, &(uint32_t){0xAA});
			if (ret)
				goto err;
		}
		ret = ocmts_run(ocmts);
err:
		/* Always try to restore ENDINIT, even if something above failed. See
		 * aurix_eflash_erase()'s identical handling. TC2x is excluded: it
		 * was never lifted above (see the comment there), so there is
		 * nothing host-side to restore, and tc3x_eflash_endinit_toggle()
		 * would write the wrong register (TC3X_WDTSCON0) for this family. */
		if (!aurix_bank->tc4x && !aurix_bank->dflash && !aurix_bank->tc2x) {
			int endinit_ret = tc3x_eflash_endinit_toggle(bank, 1);
			if (ret == ERROR_OK)
				ret = endinit_ret;
		}
		if (ret) {
			ret = aurix_eflash_reset_to_read(bank);
			if (ret)
				LOG_ERROR("Failed to reset flash to read mode. Please reset device to continue");
			LOG_ERROR("Flash program failed: failed to execute flash program sequence at address: 0x%08x, "
					  "data size: %u",
					  addr, copy_size);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (aurix_eflash_handle_error(bank, aurix_bank->busy_bit, aurix_bank->write_timeout_ms, NULL) != ERROR_OK) {
			LOG_ERROR("Flash program failed: operation failed at address 0x%08" PRIx64,
					  bank->base + offset + page_offset - copy_size);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int aurix_eflash_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return target_read_buffer(bank->target, ((bank->base + offset) & ~0xF0000000) + 0xA0000000, count, buffer);
}

static void aurix_free_driver_priv(struct flash_bank *bank)
{
	free(bank->driver_priv);
}

/* Maps a TC2x bank onto its FSR busy bit.
 *
 * The upstream reference implementation returns 0 for DFLASH0 and 1 for
 * DFLASH1. That is an off-by-one against the register: tc2x-flash.h, written
 * independently of any driver, puts DFLASH0 busy at bit 1 and DFLASH1 busy at
 * bit 2, with bit 0 unused. Reading bit 0 means the DFLASH0 busy poll always
 * sees "not busy" and returns before the operation has finished, and reading
 * bit 1 for DFLASH1 actually watches DFLASH0's busy bit. Both are latent and
 * neither shows up on a part that is never busy, which is why it survived.
 *
 * tc2x-flash.h is trusted here because it is the same source the on-target
 * loader is built from and it was not derived from the driver, so the two
 * agreeing is a real cross-check rather than a copy. */
static uint8_t tc2x_busy_bit_for(bool is_dflash, bool is_dflash1, bool is_ucb, unsigned int pflash_index)
{
	if (is_ucb) {
		/* The UCB sits inside the DFLASH0 array, so DFLASH0's busy bit
		 * covers it. */
		return 1;
	}

	if (is_dflash)
		return is_dflash1 ? 2 : 1;

	/* Program flash banks are numbered from 0, busy at bits 3..6. */
	return 3 + pflash_index;
}

FLASH_BANK_COMMAND_HANDLER(tc2x_flash_bank_command)
{
	struct aurix_eflash_bank *tc2x_bank;
	uint32_t bank_base = bank->base & 0x0FFFFFFF;
	bool is_flash_segment = (bank->base & 0xF0000000) == 0x80000000 ||
							(bank->base & 0xF0000000) == 0xA0000000;
	bool is_pflash = is_flash_segment && bank_base < 0x01000000;
	bool is_ucb = bank->base == 0xAF100000;
	bool is_dflash0 = bank->base == 0xAF000000;
	bool is_dflash1 = bank->base == 0xAF110000;
	bool is_dflash = is_dflash0 || is_dflash1;

	if (is_pflash) {
		/* Program flash banks are 2 MiB on both TC27x and TC29x, and a bank
		 * must start on a 2 MiB boundary so the variable sector table can
		 * start at sector 0 of the table. */
		if (bank->size != 2 * 1024 * 1024) {
			LOG_ERROR("Invalid pflash bank size. TC2x program flash banks are 2MB.");
			return ERROR_FLASH_BANK_INVALID;
		}
		if (bank->base % (2 * 1024 * 1024) != 0) {
			LOG_ERROR("Invalid pflash bank base address. Base should be aligned to 2MB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_ucb) {
		/* 16 KiB. Corroborated by the reference driver and by the board
		 * configuration, which agree; not independently confirmed on
		 * silicon, and programming the UCB is not supported anyway (see the
		 * probe: UCB protection state is reported as unknown). */
		if (bank->size != 16 * 1024) {
			LOG_ERROR("Invalid UCB bank size. Size should be 16KB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_dflash0) {
		/* DFLASH0 EEPROM. 384 KiB is what this TC275 reports: a
		 * single-sector erase succeeds up to 0xAF05EFFF and raises a command
		 * sequence error at 0xAF060000, on a board whose CHIPID reads
		 * 0x47467072 (CHID 0x7 = TC27x). The reference driver says 348 KiB
		 * for this part and 768 KiB in its TC29x table; 348 KiB is not even
		 * a multiple of the 8 KiB logical sector size, so it cannot be
		 * right. See CONSTANTS.md. */
		if (bank->size != 384 * 1024) {
			LOG_ERROR("Invalid dflash0 bank size. This device's DFLASH0 is 384KB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_dflash1) {
		/* DFLASH1 standby RAM. 64 KiB from the reference driver only; this
		 * driver cannot confirm it and this bank is owned by the HSM, so
		 * nothing here has been exercised against hardware. */
		if (bank->size != 64 * 1024) {
			LOG_ERROR("Invalid dflash1 bank size. Size should be 64KB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else {
		LOG_ERROR("Invalid flash bank base address: 0x%08" PRIx64, bank->base);
		return ERROR_FLASH_BANK_INVALID;
	}

	tc2x_bank = malloc(sizeof(struct aurix_eflash_bank));
	if (!tc2x_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* TC2x has one command sequencer for every bank and one status/error
	 * word. There is no second module and no FSI. */
	tc2x_bank->cmd_addr = 0xAF000000;
	tc2x_bank->reg_addr = TC2X_REG_BASE;
	tc2x_bank->fsi_addr = 0;
	tc2x_bank->sts_offset = TC2X_FSR_OFFSET;
	tc2x_bank->err_offset = TC2X_FSR_OFFSET;

	/* Logical sector size. Program flash uses the variable table in
	 * tc2x_eflash_probe(), so sector_size only drives the data flash and
	 * UCB banks. */
	tc2x_bank->sector_size = is_ucb ? 1024 : 8 * 1024;
	tc2x_bank->phys_sector_size = TC2X_PHYS_SECTOR_SIZE;
	/* Never read by the TC2x paths: erase and verify use one sector per
	 * command, so there is no maximum to enforce. */
	tc2x_bank->num_erase_sectors = 1;

	/* Smallest programmable unit and the burst length, both from
	 * tc2x-flash.h (PAGE_SIZE / BURST_SIZE). The 4 KiB figure in the
	 * reference board configuration is wrong: it is neither the program
	 * page nor the data flash page, and using it would make
	 * aurix_eflash_write() reject every 32 byte program. */
	tc2x_bank->page_size = is_dflash || is_ucb ? 8 : 32;
	tc2x_bank->burst_size = is_dflash || is_ucb ? 32 : 256;
	/* Page-mode-ready bit in the FSR, per flash type. */
	tc2x_bank->page_bit = is_dflash || is_ucb ? 10 : 9;

	/* Program flash bank index, used to pick the FSR busy bit. Derived from
	 * the bank BASE ADDRESS, not from bank->bank_number: bank_number counts
	 * every flash bank in the session including the data flash, the UCBs and
	 * the "virtual" aliases, so it only happens to agree with the program
	 * flash index in one particular declaration order. The base address is
	 * fixed by the part, so it cannot drift. */
	uint8_t pflash_index = is_pflash ? (bank->base - TC2X_PFLASH_BASE) / (2 * 1024 * 1024) : 0;
	if (pflash_index > 3) {
		LOG_ERROR("Invalid TC2x program flash bank base 0x%08" PRIx64, bank->base);
		free(tc2x_bank);
		return ERROR_FLASH_BANK_INVALID;
	}
	tc2x_bank->busy_bit = tc2x_busy_bit_for(is_dflash, is_dflash1, is_ucb, pflash_index);

	tc2x_bank->write_timeout_ms = 10;
	tc2x_bank->erase_timeout_ms = is_ucb ? 200 : is_dflash ? 500 : 400;

	tc2x_bank->dflash = is_dflash || is_ucb;
	tc2x_bank->dflash1 = is_dflash1;
	tc2x_bank->ucb = is_ucb;
	tc2x_bank->pflash_index = pflash_index;
	tc2x_bank->fallback_mode = false;
	tc2x_bank->tc2x = true;
	tc2x_bank->tc4x = false;
	tc2x_bank->probed = false;
	bank->driver_priv = tc2x_bank;

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(tc3x_flash_bank_command)
{
	struct aurix_eflash_bank *tc3x_bank;
	uint32_t bank_base = bank->base & 0x0FFFFFFF;
	bool is_flash_segment = (bank->base & 0xF0000000) == 0x80000000 || (bank->base & 0xF0000000) == 0xA0000000;
	bool is_pflash = is_flash_segment && bank_base < 0x01000000;
	bool is_ucb = bank->base == 0xAF400000;
	bool is_dflash0 = bank->base == 0xAF000000 || bank->base == 0xAF400000;
	bool is_dflash1 = bank->base == 0xAFC00000;
	bool is_dflash = is_dflash0 || is_dflash1;

	if (is_pflash) {
		if (bank->size != 1 * 1024 * 1024 && bank->size != 2 * 1024 * 1024 && bank->size != 3 * 1024 * 1024) {
			LOG_ERROR("Invalid pflash bank size. Size should be 1MB,"
					  " 2MB or 3MB.");
			return ERROR_FLASH_BANK_INVALID;
		}
		if (bank->base % (1 * 1024 * 1024) != 0) {
			LOG_ERROR("Invalid pflash bank base address. Base should be "
					  "aligned to 1MB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_ucb) {
		if (bank->size != 24 * 1024) {
			LOG_ERROR("Invalid UCB bank size. Size should be 24KB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_dflash0) {
		if (bank->size != 1 * 1024 * 1024) {
			LOG_ERROR("Invalid dflash0 bank size. Size should be 1MB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_dflash1) {
		if (bank->size != 128 * 1024) {
			LOG_ERROR("Invalid dflash1 bank size. Size should be 128KB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else {
		LOG_ERROR("Invalid flash bank base address: 0x%08" PRIx64, bank->base);
		return ERROR_FLASH_BANK_INVALID;
	}

	tc3x_bank = malloc(sizeof(struct aurix_eflash_bank));
	if (!tc3x_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* cmd_reg and offsets might be changed during probe, if HSM sequencer is
	 * required */
	tc3x_bank->cmd_addr = 0xAF000000;
	tc3x_bank->reg_addr = 0xF8040000;
	tc3x_bank->fsi_addr = 0xF8030000;
	tc3x_bank->sts_offset = 0x10;
	tc3x_bank->err_offset = 0x34;
	tc3x_bank->busy_bit = is_pflash ? bank->bank_number + 2 : is_dflash0 ? 0 : 1;
	tc3x_bank->page_bit = is_dflash ? 20 : 21;
	tc3x_bank->page_size = is_dflash ? 8 : 32;
	tc3x_bank->burst_size = is_dflash ? 32 : 256;
	tc3x_bank->sector_size = is_ucb ? 512 : is_dflash ? 4 * 1024 : 16 * 1024;
	tc3x_bank->phys_sector_size = is_ucb	   ? 24 * 1024
								  : is_dflash0 ? 1 * 1024 * 1024
								  : is_dflash1 ? 128 * 1024
											   : 1024 * 1024;
	tc3x_bank->num_erase_sectors = is_ucb	   ? 1
								   : is_dflash ? 256 * 1024 / tc3x_bank->sector_size
											   : 512 * 1024 / tc3x_bank->sector_size;
	/* The geometry above describes the DFLASH in single ended mode. Complement
	 * sensing mode halves the usable size and uses a 2 KiB minimum erase size,
	 * which is not implemented; tc3x_eflash_check_sensing_mode() rejects the
	 * bank at probe time, once the target can be read. */

	tc3x_bank->write_timeout_ms = 10;
	tc3x_bank->erase_timeout_ms = is_ucb ? 200 : is_dflash ? 500 : 400;

	tc3x_bank->dflash = is_dflash;
	tc3x_bank->dflash1 = is_dflash1;
	tc3x_bank->ucb = is_ucb;
	/* Mirrors the busy bit convention above: the PFLASH banks are declared in
	 * index order, so the bank number selects the DMU register set. */
	tc3x_bank->pflash_index = is_pflash ? bank->bank_number : 0;
	tc3x_bank->fallback_mode = false;
	tc3x_bank->tc2x = false;
	tc3x_bank->tc4x = false;
	tc3x_bank->probed = false;
	bank->driver_priv = tc3x_bank;

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(tc4x_flash_bank_command)
{
	struct aurix_eflash_bank *tc4x_bank;
	uint32_t bank_base = bank->base & 0x0FFFFFFF;
	bool is_flash_segment = (bank->base & 0xF0000000) == 0x80000000 || (bank->base & 0xF0000000) == 0xA0000000;
	bool is_pflash0 = is_flash_segment && bank_base < 0x01400000;
	bool is_pflash1 = is_flash_segment && bank_base == 0x04000000;
	bool is_ucb0 = bank->base == 0xAE400000;
	bool is_ucb1 = bank->base == 0xAEC00000;
	bool is_dflash0 = bank->base == 0xAE000000 || bank->base == 0xAE400000;
	bool is_dflash1 = bank->base == 0xAE800000 || bank->base == 0xAEC00000;
	bool is_dflash = is_dflash0 || is_dflash1;
	bool is_ucb = is_ucb0 || is_ucb1;

	if (is_pflash0) {
		if (bank->size != 1 * 1024 * 1024 && bank->size != 2 * 1024 * 1024) {
			LOG_ERROR("Invalid pflash0 bank size. Size should be 1MB or 2MB.");
			return ERROR_FLASH_BANK_INVALID;
		}
		if (bank->base % bank->size != 0) {
			LOG_ERROR("Invalid pflash0 bank base address. Base should be "
					  "aligned to bank size.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_pflash1) {
		if (bank->size != 1 * 1024 * 1024) {
			LOG_ERROR("Invalid pflash1 bank size. Size should be 1MB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_ucb0) {
		if (bank->size != 80 * 1024) {
			LOG_ERROR("Invalid UCB bank size. Size should be 80KB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_ucb1) {
		if (bank->size != 52 * 1024) {
			LOG_ERROR("Invalid UCB bank size. Size should be 52KB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_dflash0) {
		if (bank->size != 1024 * 1024 && bank->size != 512 * 1024 && bank->size != 128 * 1024) {
			LOG_ERROR("Invalid dflash bank size. Size should be 128KB, 512KB "
					  "or 1MB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else if (is_dflash1) {
		if (bank->size != 128 * 1024) {
			LOG_ERROR("Invalid dflash1 bank size. Size should be 128KB.");
			return ERROR_FLASH_BANK_INVALID;
		}
	} else {
		LOG_ERROR("Invalid flash bank base address: 0x%08" PRIx64, bank->base);
		return ERROR_FLASH_BANK_INVALID;
	}

	tc4x_bank = malloc(sizeof(struct aurix_eflash_bank));
	if (!tc4x_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	/* Select the command sequence interface based on bank addresses */
	if (is_pflash1 || is_dflash1) {
		tc4x_bank->cmd_addr = 0xF80C0000;
		tc4x_bank->sts_offset = 0x84;
		tc4x_bank->err_offset = 0x90;
		tc4x_bank->fsi_addr = 0xF8028000;
	} else {
		tc4x_bank->cmd_addr = 0xF8080000;
		tc4x_bank->sts_offset = 0x4;
		tc4x_bank->err_offset = 0x10;
		tc4x_bank->fsi_addr = 0xF8008000;
	}
	tc4x_bank->reg_addr = 0xF8040000;
	tc4x_bank->busy_bit = is_pflash0 ? bank->bank_number : is_pflash1 ? 18 : is_dflash0 ? 16 : 17;
	tc4x_bank->page_bit = is_dflash ? 24 : 25;
	tc4x_bank->page_size = is_dflash ? 8 : 32;
	tc4x_bank->burst_size = is_dflash ? 32 : 512;
	tc4x_bank->sector_size = is_ucb ? 512 : is_dflash ? 2 * 1024 : 16 * 1024;
	tc4x_bank->phys_sector_size = is_ucb0	  ? 80 * 1024
								  : is_ucb1	  ? 52 * 1024
								  : is_dflash ? (bank->size == 128 * 1024 ? 64 * 1024 : 128 * 1024)
											  : 512 * 1024;
	tc4x_bank->num_erase_sectors = is_ucb	   ? 1
								   : is_dflash ? 256 * 1024 / tc4x_bank->sector_size
											   : 512 * 1024 / tc4x_bank->sector_size;

	tc4x_bank->write_timeout_ms = 10;
	tc4x_bank->erase_timeout_ms = is_ucb ? 200 : is_dflash ? 500 : 400;

	tc4x_bank->dflash = is_dflash;
	tc4x_bank->dflash1 = is_dflash1;
	tc4x_bank->ucb = is_ucb;
	tc4x_bank->pflash_index = (is_pflash0 || is_pflash1) ? bank->bank_number : 0;
	/* Default to the slow command-sequence write path on TC4x, not the
	 * RAM-resident write algorithm. On a TC4D7 Lite Kit, the algorithm path
	 * reliably fails with "flash write algorithm aborted by target" /
	 * "Command sequence error" (target.c's generic message for "the target
	 * halted for a reason other than reaching the algorithm's exit point"),
	 * which is the exact signature tc3x_eflash_disable_safety_wdt() above was
	 * written to fix on TC3x: a watchdog resets the core mid-algorithm. TC4x
	 * has no equivalent disable sequence here because its per-CPU watchdog
	 * register addresses are not available in any sourceable reference
	 * (unlike TC3x's, no manual and no vendor header for TC4x define them),
	 * so that fix cannot be ported without inventing addresses. The command
	 * sequence path below does not run on-target code and is unaffected by
	 * this, and has been confirmed to erase and program real TC4D7 PFLASH
	 * correctly. Revisit this default if the TC4x watchdog registers become
	 * available and the algorithm path is fixed and validated the same way.
	 */
	tc4x_bank->fallback_mode = true;
	tc4x_bank->tc2x = false;
	tc4x_bank->tc4x = true;
	tc4x_bank->probed = false;
	bank->driver_priv = tc4x_bank;

	return ERROR_OK;
}

COMMAND_HANDLER(aurix_eflash_set_fallback_mode_command)
{
	struct flash_bank *bank;
	struct aurix_eflash_bank *aurix_bank;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	aurix_bank = bank->driver_priv;

	if (CMD_ARGC == 2) {
		if (strcmp(CMD_ARGV[1], "on") == 0) {
			aurix_bank->fallback_mode = true;
		} else if (strcmp(CMD_ARGV[1], "off") == 0) {
			aurix_bank->fallback_mode = false;
		} else {
			LOG_ERROR("Invalid fallback mode: %s. Should be 'on' or 'off'.", CMD_ARGV[1]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	} else {
		aurix_bank->fallback_mode = !aurix_bank->fallback_mode;
	}

	return ERROR_OK;
}

static const struct command_registration aurix_eflash_subcommand_handlers[] = {
	{
		.name = "fallback",
		.handler = aurix_eflash_set_fallback_mode_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank> [on|off]",
		.help = "Set or toggle fallback mode for flash write. In fallback mode, the driver will use slow flash write "
				"sequence instead of flash write algorithm.",
	},
	COMMAND_REGISTRATION_DONE};

static const struct command_registration aurix_eflash_command_handlers[] = {
	{
		.name = "aurix_eflash",
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Commands for the AURIX eFlash driver",
		.chain = aurix_eflash_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE};

const struct flash_driver tc2x_eflash = {
	.name = "tc2x_eflash",
	.flash_bank_command = tc2x_flash_bank_command,
	.commands = aurix_eflash_command_handlers,
	.probe = tc2x_eflash_probe,
	.auto_probe = tc2x_eflash_auto_probe,
	.erase = aurix_eflash_erase,
	.write = aurix_eflash_write,
	.read = aurix_eflash_read,
	.erase_check = aurix_eflash_erase_check,
	.protect_check = aurix_eflash_protect_check,
	.free_driver_priv = aurix_free_driver_priv,
};

const struct flash_driver tc3x_eflash = {
	.name = "tc3x_eflash",
	.flash_bank_command = tc3x_flash_bank_command,
	.commands = aurix_eflash_command_handlers,
	.probe = tc3x_eflash_probe,
	.auto_probe = tc3x_eflash_auto_probe,
	.erase = aurix_eflash_erase,
	.write = aurix_eflash_write,
	.read = aurix_eflash_read,
	.erase_check = aurix_eflash_erase_check,
	.protect_check = aurix_eflash_protect_check,
	.free_driver_priv = aurix_free_driver_priv,
};

const struct flash_driver tc4x_eflash = {
	.name = "tc4x_eflash",
	.flash_bank_command = tc4x_flash_bank_command,
	.commands = aurix_eflash_command_handlers,
	.probe = tc4x_eflash_probe,
	.auto_probe = tc4x_eflash_auto_probe,
	.erase = aurix_eflash_erase,
	.write = aurix_eflash_write,
	.read = aurix_eflash_read,
	.erase_check = aurix_eflash_erase_check,
	.protect_check = aurix_eflash_protect_check,
	.free_driver_priv = aurix_free_driver_priv,
};
