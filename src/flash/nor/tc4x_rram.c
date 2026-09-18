// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   AURIX TC4x RRAM Driver for Infineon AURIX                             *
 *   Copyright (C) 2026 Infineon Technologies AG                           *
 ***************************************************************************/

/*
 * TC4x parts with resistive RAM (RRAM) replace the eFlash program/data
 * flash used by earlier AURIX families with a PMUR/DMUR-based RRAM
 * controller. RRAM cannot be written with plain memory-mapped stores: a
 * page has to be handed to the controller through an INITWR/STWR request
 * handshake, one word at a time, via a WDATA FIFO.
 *
 * This driver is self-contained: it does not share any state with
 * aurix_eflash.c, since the register layout and programming sequence are
 * unrelated. Just like the eFlash driver it can fall back from the async
 * flash algorithm (contrib/loaders/flash/aurix/aurix-rram-program.c) to a
 * slow, single-page register-sequence write when no large enough work
 * area is available.
 *
 * The {PMUR,DMUR}.UR register layout is undocumented in the public AURIX
 * manuals and was recovered by observing controller behaviour rather than
 * from a register reference; see contrib/loaders/flash/aurix/tc4x-rram.h.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flash/common.h"
#include "flash/nor/core.h"
#include "flash/nor/driver.h"
#include "helper/bits.h"
#include "helper/log.h"
#include "helper/time_support.h"
#include "helper/types.h"
#include "jtag/jtag.h"
#include "target/algorithm.h"
#include "target/tricore/ocmts.h"
#include "target/tricore/tricore.h"
#include "target/target.h"

/* UCB_CHIPID on TC4x parts fitted with RRAM. Distinct from the UCB_CHIPID
 * address used by the (older) TC4x eFlash parts in aurix_eflash.c. */
#define UCB_CHIPID 0xAE607808
#define UCB_CHIPID_PROD 0x3C000000

#define MODULE_DMUR0_BASE 0xF8080000
#define MODULE_DMUR1_BASE 0xF80C0000
#define MODULE_PMURCS_BASE 0xF8590000

#define MODULE_PMUR0A_BASE 0xF8410000
#define MODULE_PMUR0B_BASE 0xF8420000
#define MODULE_PMUR1A_BASE 0xF8450000
#define MODULE_PMUR1B_BASE 0xF8460000
#define MODULE_PMUR2A_BASE 0xF8490000
#define MODULE_PMUR2B_BASE 0xF84A0000
#define MODULE_PMUR3A_BASE 0xF84D0000
#define MODULE_PMUR3B_BASE 0xF84E0000
#define MODULE_PMUR4A_BASE 0xF8510000
#define MODULE_PMUR4B_BASE 0xF8520000

#define CPU_CFI0_BASE 0xF8400000
#define CPU_CFI1_BASE 0xF8440000
#define CPU_CFI2_BASE 0xF8480000
#define CPU_CFI3_BASE 0xF84C0000
#define CPU_CFI4_BASE 0xF8500000
#define CPU_CFICS_BASE 0xF8580000

#define CPU_CFI_PFIBUFDIS_OFFSET 0x320

#define UR_NVMADDR_OFFSET 0x3C
#define UR_WDATA0_OFFSET 0x40
#define UR_REQUEST_OFFSET 0x74
#define UR_REQSTAT_OFFSET 0x78
#define UR_CLRSTAT_OFFSET 0x7C

#define UR_REQUEST_INITWR BIT(0)
#define UR_REQUEST_STWR BIT(1)

#define UR_REQSTAT_BUSY BIT(0)
#define UR_REQSTAT_DONE BIT(1)
#define UR_REQSTAT_ERROR_MASK 0x0018FFFC

#define UR_CLRSTAT_CLEAR_ALL                                                                                \
	(BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(9) | BIT(10))

/* Error bits returned by the flash algorithm in %d2, see errors.h. */
#define RRAM_ALGO_ERROR_BUSY BIT(31)
#define RRAM_ALGO_ERROR_TIMEOUT BIT(30)
#define RRAM_ALGO_ERROR_PAGE BIT(29)

enum tc4x_rram_type {
	TC4X_RRAM_PRRAM,
	TC4X_RRAM_DNVM,
	TC4X_RRAM_UCB,
};

struct tc4x_rram_bank {
	/** Bank probed */
	bool probed;
	/** Fallback mode for flash write */
	bool fallback_mode;
	/** User configuration block unlocked for write/erase */
	bool ucb_unlocked;
	/** Program RRAM, data RRAM or UCB */
	enum tc4x_rram_type type;
	/** Programmable unit: 32 bytes for PMUR, 8 bytes for DMUR/UCB */
	uint32_t page_size;
	/** Smallest erasable unit reported to the flash subsystem. RRAM does
	 * not require erase before write, so this only affects sector count. */
	uint32_t sector_size;
	/** Register-sequence write timeout in milliseconds */
	uint32_t write_timeout_ms;
};

struct tc4x_rram_bank_info {
	target_addr_t base;
	uint32_t size;
	enum tc4x_rram_type type;
	uint32_t page_size;
	uint32_t sector_size;
};

static const uint8_t tc4x_rram_pnvm_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc4x-rram-pnvm-program.inc"
};

static const uint8_t tc4x_rram_dnvm_write_code[] = {
#include "../../../contrib/loaders/flash/aurix/tc4x-rram-dnvm-program.inc"
};

/* The RRAM controller is addressed through a fixed NVM segment (0xA0......),
 * regardless of which memory-mapped alias the bank was configured with. */
static inline uint32_t tc4x_rram_to_nvm_segment(uint32_t address)
{
	return (address & 0x0FFFFFFF) | 0xA0000000;
}

static inline bool tc4x_rram_is_dnvm_addr(uint32_t nvm_addr)
{
	return (nvm_addr & 0xFF000000) == 0xAE000000;
}

static target_addr_t tc4x_rram_get_pmur_base(uint32_t nvm_addr)
{
	uint32_t index;

	if (nvm_addr < 0xA0000000 || nvm_addr > 0xAFFFFFFF)
		return MODULE_PMURCS_BASE;

	index = (nvm_addr - 0xA0000000) >> 21;

	switch (index) {
	case 0:
		return MODULE_PMUR0A_BASE;
	case 1:
		return MODULE_PMUR0B_BASE;
	case 2:
		return MODULE_PMUR1A_BASE;
	case 3:
		return MODULE_PMUR1B_BASE;
	case 4:
		return MODULE_PMUR2A_BASE;
	case 5:
		return MODULE_PMUR2B_BASE;
	case 6:
		return MODULE_PMUR3A_BASE;
	case 7:
		return MODULE_PMUR3B_BASE;
	case 8:
		return MODULE_PMUR4A_BASE;
	case 9:
		return MODULE_PMUR4B_BASE;
	default:
		return MODULE_PMURCS_BASE;
	}
}

static target_addr_t tc4x_rram_get_module_base(uint32_t nvm_addr)
{
	if (tc4x_rram_is_dnvm_addr(nvm_addr)) {
		if (nvm_addr >= 0xAE800000)
			return MODULE_DMUR1_BASE;
		return MODULE_DMUR0_BASE;
	}

	return tc4x_rram_get_pmur_base(nvm_addr);
}

/* The prefetch buffer of the CPU that owns a given RRAM instance has to be
 * disabled while reading through the memory-mapped alias, or stale data can
 * be returned after a write. Each CPU has its own CFI register block. */
static target_addr_t tc4x_rram_get_cfi_base(uint32_t nvm_addr)
{
	target_addr_t module_base = tc4x_rram_get_module_base(nvm_addr);

	switch (module_base) {
	case MODULE_DMUR0_BASE:
	case MODULE_PMUR0A_BASE:
	case MODULE_PMUR0B_BASE:
		return CPU_CFI0_BASE;
	case MODULE_DMUR1_BASE:
	case MODULE_PMUR1A_BASE:
	case MODULE_PMUR1B_BASE:
		return CPU_CFI1_BASE;
	case MODULE_PMUR2A_BASE:
	case MODULE_PMUR2B_BASE:
		return CPU_CFI2_BASE;
	case MODULE_PMUR3A_BASE:
	case MODULE_PMUR3B_BASE:
		return CPU_CFI3_BASE;
	case MODULE_PMUR4A_BASE:
	case MODULE_PMUR4B_BASE:
		return CPU_CFI4_BASE;
	case MODULE_PMURCS_BASE:
		return CPU_CFICS_BASE;
	default:
		return CPU_CFI0_BASE;
	}
}

static void tc4x_rram_error_append(char *err_str, size_t err_str_size, const char *msg)
{
	size_t len;

	if (!err_str || !msg || err_str_size == 0)
		return;

	len = strlen(err_str);
	if (len >= err_str_size - 1)
		return;

	snprintf(err_str + len, err_str_size - len, "%s%s", len ? ", " : "", msg);
}

static void tc4x_rram_decode_sfr_status(uint32_t reqstat, char *err_str, size_t err_str_size)
{
	uint32_t known_mask = 0;

	if (reqstat & BIT(2)) {
		known_mask |= BIT(2);
		tc4x_rram_error_append(err_str, err_str_size, "protection error");
	}
	if (reqstat & BIT(3)) {
		known_mask |= BIT(3);
		tc4x_rram_error_append(err_str, err_str_size, "command sequence error");
	}
	if (reqstat & BIT(4)) {
		known_mask |= BIT(4);
		tc4x_rram_error_append(err_str, err_str_size, "abort error");
	}
	if (reqstat & BIT(5)) {
		known_mask |= BIT(5);
		tc4x_rram_error_append(err_str, err_str_size, "clear error");
	}
	if (reqstat & BIT(6)) {
		known_mask |= BIT(6);
		tc4x_rram_error_append(err_str, err_str_size, "program verify error");
	}
	if (reqstat & BIT(7)) {
		known_mask |= BIT(7);
		tc4x_rram_error_append(err_str, err_str_size, "erase verify error");
	}
	if (reqstat & BIT(8)) {
		known_mask |= BIT(8);
		tc4x_rram_error_append(err_str, err_str_size, "flash operation error");
	}
	if (reqstat & BIT(19)) {
		known_mask |= BIT(19);
		tc4x_rram_error_append(err_str, err_str_size, "no page mode entry");
	}
	if (reqstat & BIT(20)) {
		known_mask |= BIT(20);
		tc4x_rram_error_append(err_str, err_str_size, "flash operation timeout");
	}

	uint32_t unknown_mask = (reqstat & UR_REQSTAT_ERROR_MASK) & ~known_mask;
	for (unsigned int bit = 0; bit < 32; bit++) {
		if (unknown_mask & BIT(bit)) {
			char bit_desc[32];

			snprintf(bit_desc, sizeof(bit_desc), "unknown SFR error bit %u", bit);
			tc4x_rram_error_append(err_str, err_str_size, bit_desc);
		}
	}
}

static void tc4x_rram_decode_algo_status(uint32_t status, char *err_str, size_t err_str_size)
{
	if (status & RRAM_ALGO_ERROR_BUSY)
		tc4x_rram_error_append(err_str, err_str_size, "algorithm busy");
	if (status & RRAM_ALGO_ERROR_TIMEOUT)
		tc4x_rram_error_append(err_str, err_str_size, "algorithm timeout");
	if (status & RRAM_ALGO_ERROR_PAGE)
		tc4x_rram_error_append(err_str, err_str_size, "algorithm page mode failure");

	tc4x_rram_decode_sfr_status(status, err_str, err_str_size);
}

static int tc4x_rram_queue_clear_status(struct ocmts *ocmts, target_addr_t module_base)
{
	uint32_t clr = UR_CLRSTAT_CLEAR_ALL;

	return ocmts_queue_write_u32(ocmts, module_base + UR_CLRSTAT_OFFSET, &clr);
}

static int tc4x_rram_wait_request_done(struct flash_bank *bank, target_addr_t module_base,
										uint32_t timeout_ms, uint32_t *reqstat_out)
{
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	int64_t start_time = timeval_ms();
	uint32_t reqstat = 0;
	int ret;

	while (start_time + timeout_ms > timeval_ms()) {
		ret = ocmts_queue_read_u32(ocmts, module_base + UR_REQSTAT_OFFSET, &reqstat);
		if (ret == ERROR_OK)
			ret = ocmts_run(ocmts);
		if (ret != ERROR_OK)
			return ret;

		if (!(reqstat & UR_REQSTAT_BUSY) && (reqstat & UR_REQSTAT_DONE)) {
			if (reqstat_out)
				*reqstat_out = reqstat;
			return ERROR_OK;
		}

		jtag_sleep(100);
	}

	if (reqstat_out)
		*reqstat_out = reqstat;

	char err_str[256] = {0};

	tc4x_rram_decode_sfr_status(reqstat, err_str, sizeof(err_str));
	if (err_str[0])
		LOG_ERROR("RRAM request timed out (module=0x%08" TARGET_PRIxADDR ", reqstat=0x%08" PRIx32 ", %s)",
				  module_base, reqstat, err_str);
	else
		LOG_ERROR("RRAM request timed out (module=0x%08" TARGET_PRIxADDR ", reqstat=0x%08" PRIx32 ")",
				  module_base, reqstat);
	return ERROR_FLASH_OPERATION_FAILED;
}

static int tc4x_rram_write_page_register(struct flash_bank *bank, uint32_t page_addr,
										  const uint8_t *buffer, uint32_t page_size, uint32_t timeout_ms)
{
	struct ocmts *ocmts = target_to_tricore(bank->target)->ocmts;
	target_addr_t module_base = tc4x_rram_get_module_base(page_addr);
	uint32_t reqstat;
	uint32_t initwr = UR_REQUEST_INITWR;
	uint32_t stwr = UR_REQUEST_STWR;
	int ret;

	ret = tc4x_rram_queue_clear_status(ocmts, module_base);
	if (ret != ERROR_OK)
		return ret;

	ret = ocmts_queue_write_u32(ocmts, module_base + UR_REQUEST_OFFSET, &initwr);
	if (ret != ERROR_OK)
		return ret;

	ret = ocmts_queue_write_u32(ocmts, module_base + UR_NVMADDR_OFFSET, &page_addr);
	if (ret != ERROR_OK)
		return ret;

	for (uint32_t i = 0; i < page_size; i += 4) {
		uint32_t word;

		/* buffer is a raw byte pointer with no alignment guarantee; go
		 * through memcpy() rather than casting to uint32_t * so this
		 * builds clean with -Wcast-align (and is well-defined even on
		 * strict-alignment hosts) regardless of the caller's offset. */
		memcpy(&word, buffer + i, sizeof(word));
		ret = ocmts_queue_write_u32(ocmts, module_base + UR_WDATA0_OFFSET + i, &word);
		if (ret != ERROR_OK)
			return ret;
	}

	ret = ocmts_queue_write_u32(ocmts, module_base + UR_REQUEST_OFFSET, &stwr);
	if (ret != ERROR_OK)
		return ret;

	ret = ocmts_run(ocmts);
	if (ret != ERROR_OK)
		return ret;

	ret = tc4x_rram_wait_request_done(bank, module_base, timeout_ms, &reqstat);
	if (ret != ERROR_OK)
		return ret;

	if (reqstat & UR_REQSTAT_ERROR_MASK) {
		char err_str[256] = {0};

		tc4x_rram_decode_sfr_status(reqstat, err_str, sizeof(err_str));
		if (err_str[0])
			LOG_ERROR("RRAM write failed (addr=0x%08" PRIx32 ", reqstat=0x%08" PRIx32 ", %s)",
					  page_addr, reqstat, err_str);
		else
			LOG_ERROR("RRAM write failed (addr=0x%08" PRIx32 ", reqstat=0x%08" PRIx32 ")",
					  page_addr, reqstat);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

/* TC48x RRAM layout: nine PMUR (program RRAM) instances, two DMUR (data
 * RRAM) instances and their associated UCBs. Other RRAM-equipped product
 * families are not supported yet; extend this table when adding one. */
static int tc4x_rram_set_bank(struct flash_bank *bank, struct tc4x_rram_bank *rram_bank, uint32_t chipid)
{
	static const struct tc4x_rram_bank_info tc4x_rram_tc48x_layout[] = {
		{0x80000000, 0x200000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0x80200000, 0x200000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0x80400000, 0x200000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0x80600000, 0x200000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0x80800000, 0x200000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0x80900000, 0x200000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0x80A00000, 0x200000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0x80C00000, 0x200000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0x84000000, 0x080000, TC4X_RRAM_PRRAM, 32, 0x80000},
		{0xAE000000, 0x040000, TC4X_RRAM_DNVM, 8, 0x40000},
		{0xAE400000, 0x008000, TC4X_RRAM_UCB, 8, 0x8000},
		{0xAE800000, 0x020000, TC4X_RRAM_DNVM, 8, 0x20000},
		{0xAEC00000, 0x008000, TC4X_RRAM_UCB, 8, 0x8000},
	};
	uint32_t prod_family;
	const struct tc4x_rram_bank_info *layout;
	size_t layout_count;

	prod_family = (chipid & UCB_CHIPID_PROD) >> 26;
	switch (prod_family) {
	case 8:
		layout = tc4x_rram_tc48x_layout;
		layout_count = ARRAY_SIZE(tc4x_rram_tc48x_layout);
		break;
	default:
		LOG_ERROR("CHIPID product family %" PRIu32 " is not supported by the tc4x_rram driver.", prod_family);
		return ERROR_FLASH_BANK_INVALID;
	}

	for (size_t i = 0; i < layout_count; i++) {
		if (layout[i].base == bank->base && layout[i].size == bank->size) {
			rram_bank->type = layout[i].type;
			rram_bank->page_size = layout[i].page_size;
			rram_bank->sector_size = layout[i].sector_size;
			rram_bank->write_timeout_ms = 10;
			return ERROR_OK;
		}
	}

	LOG_ERROR("Failed to find TC4x RRAM bank at 0x%08" TARGET_PRIxADDR " size 0x%08" PRIx32,
			  bank->base, bank->size);
	return ERROR_FLASH_BANK_INVALID;
}

static int tc4x_rram_probe(struct flash_bank *bank)
{
	struct tc4x_rram_bank *rram_bank = bank->driver_priv;
	uint32_t flash_addr = bank->base;
	uint32_t chipid;
	int retval;

	if (rram_bank->probed)
		return ERROR_OK;

	retval = target_read_u32(bank->target, UCB_CHIPID, &chipid);
	if (retval != ERROR_OK) {
		LOG_ERROR("Cannot read CHIPID register.");
		return retval;
	}

	retval = tc4x_rram_set_bank(bank, rram_bank, chipid);
	if (retval != ERROR_OK)
		return retval;

	bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	bank->write_start_alignment = rram_bank->page_size;
	bank->write_end_alignment = rram_bank->page_size;
	bank->num_sectors = bank->size / rram_bank->sector_size;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors) {
		LOG_ERROR("Out of memory while allocating flash sectors.");
		return ERROR_FAIL;
	}

	/* RRAM does not require erase before write, unlike eFlash, so the
	 * erase state is always reported as erased. */
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = rram_bank->sector_size;
		bank->sectors[i].offset = flash_addr - bank->base;
		flash_addr += rram_bank->sector_size;
		bank->sectors[i].is_erased = 1;
		bank->sectors[i].is_protected = -1;
	}

	rram_bank->probed = true;

	return ERROR_OK;
}

static int tc4x_rram_auto_probe(struct flash_bank *bank)
{
	struct tc4x_rram_bank *rram_bank = bank->driver_priv;

	if (rram_bank->probed)
		return ERROR_OK;

	return tc4x_rram_probe(bank);
}

static int tc4x_rram_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct tc4x_rram_bank *rram_bank = bank->driver_priv;

	if (rram_bank->type == TC4X_RRAM_UCB && !rram_bank->ucb_unlocked) {
		LOG_WARNING("UCB bank has not been unlocked. Skipping operation.");
		return ERROR_OK;
	}

	if (first > last || last >= bank->num_sectors)
		return ERROR_FLASH_SECTOR_INVALID;

	for (unsigned int i = first; i <= last; i++)
		bank->sectors[i].is_erased = 1;

	LOG_INFO("TC4x RRAM does not require erase; operation treated as success.");

	return ERROR_OK;
}

static int tc4x_rram_erase_check(struct flash_bank *bank)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;

	return ERROR_OK;
}

static int tc4x_rram_write_algo(struct flash_bank *bank, uint32_t address, const uint8_t *buffer, uint32_t bytes)
{
	struct tc4x_rram_bank *rram_bank = bank->driver_priv;
	struct target *target = bank->target;
	struct reg_param reg_params[6];
	const uint8_t *write_code;
	size_t write_code_size;
	target_addr_t module_base;
	uint32_t buffer_size = 0x8000;
	const uint32_t nvmaddr = tc4x_rram_to_nvm_segment(address);
	int ret;
	struct working_area *source;

	if (rram_bank->type == TC4X_RRAM_PRRAM) {
		write_code = tc4x_rram_pnvm_write_code;
		write_code_size = sizeof(tc4x_rram_pnvm_write_code);
	} else {
		write_code = tc4x_rram_dnvm_write_code;
		write_code_size = sizeof(tc4x_rram_dnvm_write_code);
	}

	/* Loaded at a fixed PSPR address, same convention as the eFlash write
	 * algorithm in aurix_eflash.c; only the ring buffer below comes from a
	 * dynamically allocated work area. */
	ret = target_write_buffer(target, 0x70100000, write_code_size, write_code);
	if (ret != ERROR_OK)
		return ret;

	module_base = tc4x_rram_get_module_base(nvmaddr);

	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL;
		if (buffer_size <= 256) {
			LOG_WARNING("No enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "a4", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "d4", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "d5", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "d6", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "a5", 32, PARAM_OUT);
	init_reg_param(&reg_params[5], "d2", 32, PARAM_IN);

	buf_set_u32(reg_params[0].value, 0, 32, (uint32_t)source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->size);
	buf_set_u32(reg_params[2].value, 0, 32, nvmaddr);
	buf_set_u32(reg_params[3].value, 0, 32, bytes);
	buf_set_u32(reg_params[4].value, 0, 32, (uint32_t)module_base);

	ret = target_run_flash_async_algorithm(target, buffer, bytes / 4, 4, 0, NULL, ARRAY_SIZE(reg_params),
											reg_params, source->address, source->size, 0x70100000, 0, NULL);

	if (ret == ERROR_FLASH_OPERATION_FAILED) {
		uint32_t status = buf_get_u32(reg_params[5].value, 0, 32);
		char err_str[256] = {0};

		tc4x_rram_decode_algo_status(status, err_str, sizeof(err_str));
		if (err_str[0])
			LOG_ERROR("RRAM write algorithm failed with status 0x%08" PRIx32 " (%s)", status, err_str);
		else
			LOG_ERROR("RRAM write algorithm failed with status 0x%08" PRIx32, status);
	}

	target_free_working_area(target, source);
	for (size_t i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	return ret;
}

static int tc4x_rram_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct tc4x_rram_bank *rram_bank = bank->driver_priv;
	int ret;

	if (rram_bank->type == TC4X_RRAM_UCB && !rram_bank->ucb_unlocked) {
		LOG_WARNING("UCB bank has not been unlocked. Skipping operation.");
		return ERROR_OK;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = tc4x_rram_auto_probe(bank);
	if (ret != ERROR_OK)
		return ret;

	if ((offset & (rram_bank->page_size - 1)) || count % rram_bank->page_size != 0)
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;

	if (!rram_bank->fallback_mode) {
		ret = tc4x_rram_write_algo(bank, bank->base + offset, buffer, count);
		if (ret == ERROR_OK)
			return ERROR_OK;
		if (ret != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			return ret;

		LOG_WARNING("No enough resources to run RRAM write algorithm, fallback to slow register sequence.");
	}

	for (uint32_t page_offset = 0; page_offset < count; page_offset += rram_bank->page_size) {
		uint32_t page_addr = tc4x_rram_to_nvm_segment(bank->base + offset + page_offset);

		ret = tc4x_rram_write_page_register(bank, page_addr, buffer + page_offset, rram_bank->page_size,
											 rram_bank->write_timeout_ms);
		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

static int tc4x_rram_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	uint32_t mapped_addr = ((bank->base + offset) & ~0xF0000000) + 0xA0000000;
	uint32_t nvm_addr = tc4x_rram_to_nvm_segment(bank->base + offset);
	target_addr_t cfi_pfibufdis = tc4x_rram_get_cfi_base(nvm_addr) + CPU_CFI_PFIBUFDIS_OFFSET;
	int ret, restore_ret;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = tc4x_rram_auto_probe(bank);
	if (ret != ERROR_OK)
		return ret;

	/* Disable the owning CPU's prefetch buffer while reading through the
	 * memory-mapped alias, or stale data can be returned right after a
	 * write; same rationale as aurix_eflash.c's non-cached read segment. */
	ret = target_write_u32(bank->target, cfi_pfibufdis, 1);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to disable prefetch buffer at 0x%08" TARGET_PRIxADDR " (ret=%d)", cfi_pfibufdis, ret);
		return ret;
	}

	ret = target_read_buffer(bank->target, mapped_addr, count, buffer);

	restore_ret = target_write_u32(bank->target, cfi_pfibufdis, 0);
	if (restore_ret != ERROR_OK) {
		LOG_ERROR("Failed to re-enable prefetch buffer at 0x%08" TARGET_PRIxADDR " (ret=%d)",
				  cfi_pfibufdis, restore_ret);
		if (ret == ERROR_OK)
			ret = restore_ret;
	}

	return ret;
}

static void tc4x_rram_free_driver_priv(struct flash_bank *bank)
{
	free(bank->driver_priv);
}

FLASH_BANK_COMMAND_HANDLER(tc4x_rram_flash_bank_command)
{
	struct tc4x_rram_bank *rram_bank = malloc(sizeof(struct tc4x_rram_bank));

	if (!rram_bank)
		return ERROR_FLASH_OPERATION_FAILED;

	rram_bank->probed = false;
	rram_bank->fallback_mode = false;
	rram_bank->ucb_unlocked = false;
	rram_bank->type = TC4X_RRAM_PRRAM;
	rram_bank->page_size = 0;
	rram_bank->sector_size = 0;
	rram_bank->write_timeout_ms = 10;
	bank->driver_priv = rram_bank;

	return ERROR_OK;
}

COMMAND_HANDLER(tc4x_rram_set_fallback_mode_command)
{
	struct flash_bank *bank;
	struct tc4x_rram_bank *rram_bank;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	rram_bank = bank->driver_priv;

	if (CMD_ARGC == 2) {
		if (strcmp(CMD_ARGV[1], "on") == 0) {
			rram_bank->fallback_mode = true;
		} else if (strcmp(CMD_ARGV[1], "off") == 0) {
			rram_bank->fallback_mode = false;
		} else {
			LOG_ERROR("Invalid fallback mode: %s. Should be 'on' or 'off'.", CMD_ARGV[1]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	} else {
		rram_bank->fallback_mode = !rram_bank->fallback_mode;
	}

	return ERROR_OK;
}

/*
 * UCB banks are locked by default, so erase() and write() refuse to touch
 * them until the user has explicitly unlocked the bank with this command.
 * The lock only exists in host memory, so it has to be settable from a
 * command: without one, a locked UCB bank silently ignored every write.
 */
COMMAND_HANDLER(tc4x_rram_set_ucb_unlocked_command)
{
	struct flash_bank *bank;
	struct tc4x_rram_bank *rram_bank;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	rram_bank = bank->driver_priv;

	if (rram_bank->type != TC4X_RRAM_UCB) {
		LOG_ERROR("Bank %s is not a UCB bank.", bank->name);
		return ERROR_FLASH_BANK_INVALID;
	}

	if (CMD_ARGC == 2) {
		if (strcmp(CMD_ARGV[1], "on") == 0) {
			rram_bank->ucb_unlocked = true;
		} else if (strcmp(CMD_ARGV[1], "off") == 0) {
			rram_bank->ucb_unlocked = false;
		} else {
			LOG_ERROR("Invalid unlock state: %s. Should be 'on' or 'off'.", CMD_ARGV[1]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	} else {
		rram_bank->ucb_unlocked = !rram_bank->ucb_unlocked;
	}

	LOG_INFO("UCB bank %s write/erase %s.", bank->name,
		 rram_bank->ucb_unlocked ? "unlocked" : "locked");

	return ERROR_OK;
}

static const struct command_registration tc4x_rram_subcommand_handlers[] = {
	{
		.name = "fallback",
		.handler = tc4x_rram_set_fallback_mode_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank> [on|off]",
		.help = "Set or toggle fallback mode for flash write. In fallback mode, the driver will use slow "
				"register-sequence writes instead of the flash write algorithm.",
	},
	{
		.name = "unlock",
		.handler = tc4x_rram_set_ucb_unlocked_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank> [on|off]",
		.help = "Set or toggle write/erase permission for a UCB bank. UCB banks are locked by default, because a "
				"UCB write can change the device's security configuration; erase and write on a locked bank are "
				"skipped.",
	},
	COMMAND_REGISTRATION_DONE};

static const struct command_registration tc4x_rram_command_handlers[] = {
	{
		.name = "tc4x_rram",
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Commands for the TC4x RRAM driver",
		.chain = tc4x_rram_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE};

const struct flash_driver tc4x_rram = {
	.name = "tc4x_rram",
	.flash_bank_command = tc4x_rram_flash_bank_command,
	.commands = tc4x_rram_command_handlers,
	.probe = tc4x_rram_probe,
	.auto_probe = tc4x_rram_auto_probe,
	.erase = tc4x_rram_erase,
	.write = tc4x_rram_write,
	.read = tc4x_rram_read,
	.erase_check = tc4x_rram_erase_check,
	.free_driver_priv = tc4x_rram_free_driver_priv,
};
