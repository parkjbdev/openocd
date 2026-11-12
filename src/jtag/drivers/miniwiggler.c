// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * OpenOCD adapter driver for Infineon miniWiggler V3 (FT2232H, VID:PID 058b:0043)
 *
 * Implements ocmts_ops + ifxdap_driver over the shared jtag/ifxdap.[ch]
 * protocol library, itself built on the generic MPSSE helpers in
 * jtag/drivers/mpsse.[ch]. This file only supplies the miniWiggler V3's own
 * FTDI VID:PID and GPIO pin table (schematic-specific), without requiring
 * Infineon's closed-source DAS/TAS daemon. Derived from protocol analysis
 * in epozzobon/tricore-things (HackInBo 2025).
 *
 * Supports: AURIX TC2xx, TC3xx (TC375), TC4xx
 * Transport: TRANSPORT_IFXDAP
 *
 * Copyright (C) 2026 Rafael J. Cruz <rafa.junio@hardenedlinux.org>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <helper/log.h>
#include <helper/time_support.h>
#include <helper/types.h>
#include <jtag/interface.h>
#include <jtag/jtag.h>
#include <jtag/adapter.h>
#include <jtag/ifxdap.h>
#include <target/tricore/ocmts.h>
#include <target/tricore/aurix.h>

/* ---------------------------------------------------------------------- */
/* Module-global state                                                      */
/* ---------------------------------------------------------------------- */

/** Default Infineon miniWiggler V3 custom VID:PID burnt into FT2232H EEPROM */
#define MW_DEFAULT_VID  0x058b
#define MW_DEFAULT_PID  0x0043

/* miniWiggler V3 uses FT2232H Channel A (0-based MPSSE channel 0) for DAP. */
#define MW_CHANNEL      0

#define MW_SLOW_HZ      720000   /* Initial unlock frequency */
#define MW_FAST_HZ      5000000  /* Normal operating frequency */

static uint16_t mw_vid = MW_DEFAULT_VID;
static uint16_t mw_pid = MW_DEFAULT_PID;

/*
 * miniWiggler V3 (FT2232H Channel A) GPIO pin assignments, expressed as
 * MPSSE "Set Data Bits Low/High Byte" (value, direction) pairs.
 *
 * Source: epozzobon/tricore-things ftdi_dap.py MiniWigglerBatch class.
 * ADBUS (low byte):
 *   ADBUS0 -> DAP0 (clock), ADBUS1 -> TDI (U304_A), ADBUS2 <- TDO (U306_A),
 *   ADBUS3 -> TMS, ADBUS4 -> U303_OE, ADBUS7 -> D301 (LED)
 * ACBUS (high byte):
 *   ACBUS1 -> RESET (nSRST), ACBUS2 -> TRST, ACBUS3 -> U307_A,
 *   ACBUS4 -> U304_OE, ACBUS5 -> U306_DIR/USR8, ACBUS7 -> U307_DIR
 */
static const struct ifxdap_pins mw_pins = {
	.adbus_val_normal = 0x80, .adbus_dir_normal = 0xDB,
	.adbus_val_trst   = 0x00, .adbus_dir_trst   = 0x4B,

	.acbus_val_output = 0x67, .acbus_dir_output = 0xF7,
	.acbus_val_input  = 0x57, .acbus_dir_input  = 0xF7,
	.acbus_val_trst   = 0x12, .acbus_dir_trst   = 0xF2,
	.acbus_val_reset  = 0x55, .acbus_dir_reset  = 0xF7,
};

/** Singleton DAP context - one FTDI device per session. */
static struct ifxdap_ctx *g_dap;
/* Set once the DAP handshake has completed in this process, so diagnostics
 * can connect on demand without redoing it after every 'ocmts init' failure. */
static bool g_dap_connected;
static bool g_is_tc4x; /* set by the first connect; the reset repeats its key exchange */

/*
 * Operation queue: we accumulate mw_op entries during queue_io_* calls,
 * then execute them all in one MPSSE burst when run() is called.
 * This mirrors the TAS client's queue-then-execute model.
 */
#define MW_MAX_OPS 4096

enum mw_op_type {
	MW_OP_SET_ADDRESS,
	MW_OP_WRITE_WORD,
	MW_OP_READ_WORD,
	MW_OP_WRITE_HWORD,
	MW_OP_READ_HWORD,
	MW_OP_WRITE_BYTE,
	MW_OP_READ_BYTE,
	MW_OP_WRITE_BLOCK,
	MW_OP_READ_BLOCK,
	MW_OP_SET_OJCONF,
	MW_OP_SET_ACR,
	MW_OP_WRITEREG0,      /* raw DAP writereg_0 (IOCONF) */
};

struct mw_op {
	enum mw_op_type type;
	uint32_t        addr;   /* set_address value */
	uint64_t        wdata;  /* write data (word/hword/byte/ojconf/acr) */
	void           *dst;    /* pointer to write read result */
	const void     *wbuf;   /* write_block source buffer */
	size_t          count;  /* block count in words */
};

static struct mw_op g_ops[MW_MAX_OPS];
static size_t g_nops;
static uint32_t g_cur_addr; /* tracks last selected address for block ops */

/*
 * g_ioconf_preamble: value written to DAP register 0 (IOCONF) at the start of
 * every transaction, mirroring ftdi_dap.py, which prefixes *every* memory
 * operation with set_io_client(1) + writereg_0(0xC1).
 *
 * This is the IO-client selection preamble, not a halt request: reads and
 * writes work unchanged while the core is running. Values without bit 7 set
 * (0x41, 0xC0, 0xC9) wedge the OCMTS memory interface so that every subsequent
 * read returns zero, so do not lower it.
 */
#define MW_IOCONF_PREAMBLE 0xC1
static uint32_t g_ioconf_preamble = MW_IOCONF_PREAMBLE;

/*
 * TC4x COMDATA key-exchange default key.
 *
 * TC4x devices need ifxdap_tc4x_key_exchange() to complete before their
 * OCMTS debug registers respond (see ocmts_init()'s TC4x address branch and
 * ifxdap_tc4x_key_exchange()'s own comment for the full rationale). No
 * board-specific key is known, so this uses Infineon's own publicly
 * documented 256-bit all-zero fallback for devices that require no debug
 * authorization: das_default_key256 / DAS_KEY_TYPE_256BIT_NOKEY (0xE1) in
 * /opt/Tools/DAS/8.3.0/include/das_api.h. This is a vendor-documented
 * default for unauthenticated devices, not a confirmed board-specific
 * secret.
 */
static const uint32_t mw_tc4x_default_key[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/* ---------------------------------------------------------------------- */
/* ocmts_ops implementation                                                 */
/* ---------------------------------------------------------------------- */

static int mw_op_connect(struct ocmts *ocmts)
{
	if (!g_dap) {
		LOG_ERROR("miniwiggler: DAP context not initialized");
		return ERROR_FAIL;
	}
	LOG_INFO("miniwiggler: connecting DAP (%.1f kHz slow, %.1f MHz fast)",
		 MW_SLOW_HZ / 1000.0f, MW_FAST_HZ / 1000000.0f);
	int ret = ifxdap_connect(g_dap, MW_SLOW_HZ, MW_FAST_HZ);
	if (ret < 0) {
		LOG_ERROR("miniwiggler: DAP connect failed");
		return ERROR_FAIL;
	}
	LOG_INFO("miniwiggler: DAP connected OK");
	g_dap_connected = true;

	/*
	 * TC4x needs an additional COMDATA key-exchange handshake before its
	 * OCMTS debug registers respond; see ifxdap_tc4x_key_exchange()'s
	 * comment. Hardware-validated on TC4D7 (serial LKBM1OL0, see
	 * tc4-work-2026/KEYEXCHANGE.md §8): a read-only probe of the PFLASH
	 * reset vector went from reading back all-zero to an exact,
	 * byte-for-byte match against the known-good backup, with this key
	 * exchange as the only change. A failure here is still logged but
	 * does not fail the whole connect, since the all-zero default key is
	 * only confirmed for that one board/serial so far: ocmts_init()'s own
	 * recovery path (ocmts_wait_io_client() / ocmts_unwedge_io_client())
	 * still runs afterwards and gives the clearest signal on other units.
	 * TC3x/TC2xx device types are unaffected -- this is a no-op for them.
	 */
	uint32_t dev_type = aurix_tap_device_type(ocmts->tap);
	g_is_tc4x = aurix_device_is_tc4x(dev_type);
	if (g_is_tc4x) {
		LOG_INFO("miniwiggler: TC4x detected, running COMDATA key exchange");
		if (ifxdap_tc4x_key_exchange(g_dap, 0, mw_tc4x_default_key,
					ARRAY_SIZE(mw_tc4x_default_key)) < 0)
			LOG_WARNING("miniwiggler: TC4x key exchange failed; "
				"OCMTS may remain unresponsive");
		else
			LOG_INFO("miniwiggler: TC4x key exchange OK");
	}

	return ERROR_OK;
}

static int mw_op_queue_set_address(struct ocmts *ocmts, uint32_t addr)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_SET_ADDRESS, .addr = addr };
	return ERROR_OK;
}

static int mw_op_queue_write_word(struct ocmts *ocmts, const void *data)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	uint32_t val;
	memcpy(&val, data, 4);
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_WRITE_WORD, .wdata = val };
	return ERROR_OK;
}

static int mw_op_queue_read_word(struct ocmts *ocmts, uint32_t *data)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_READ_WORD, .dst = data };
	return ERROR_OK;
}

static int mw_op_queue_write_hword(struct ocmts *ocmts, const void *data)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	uint16_t val;
	memcpy(&val, data, 2);
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_WRITE_HWORD, .wdata = val };
	return ERROR_OK;
}

static int mw_op_queue_read_hword(struct ocmts *ocmts, uint16_t *data)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_READ_HWORD, .dst = data };
	return ERROR_OK;
}

static int mw_op_queue_write_byte(struct ocmts *ocmts, const void *data)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	uint8_t val;
	memcpy(&val, data, 1);
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_WRITE_BYTE, .wdata = val };
	return ERROR_OK;
}

static int mw_op_queue_read_byte(struct ocmts *ocmts, uint8_t *data)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_READ_BYTE, .dst = data };
	return ERROR_OK;
}

static int mw_op_queue_write_block(struct ocmts *ocmts, const void *buffer,
				   size_t count)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){
		.type  = MW_OP_WRITE_BLOCK,
		.wbuf  = buffer,
		.count = count,
	};
	return ERROR_OK;
}

static int mw_op_queue_read_block(struct ocmts *ocmts, void *buffer,
				  size_t count)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){
		.type  = MW_OP_READ_BLOCK,
		.dst   = buffer,
		.count = count,
	};
	return ERROR_OK;
}

static int mw_op_queue_set_ojconf(struct ocmts *ocmts, uint16_t ojconf)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_SET_OJCONF, .wdata = ojconf };
	return ERROR_OK;
}

static int mw_op_queue_set_acr(struct ocmts *ocmts, uint8_t acr)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_SET_ACR, .wdata = acr };
	return ERROR_OK;
}

/*
 * queue_io_writereg0 - queue a raw DAP writereg_0 telegram.
 *
 * Used by the target layer to send halt (0xC1) or resume (0x41) commands
 * directly via the DAP control register without going through the memory bus.
 * The queued writereg_0 executes AFTER the preamble's own 0xC1, so 0x41
 * can override the preamble halt in the same exec batch.
 */
static int mw_op_queue_writereg0(struct ocmts *ocmts, uint8_t val)
{
	if (g_nops >= MW_MAX_OPS)
		return ERROR_FAIL;
	g_ops[g_nops++] = (struct mw_op){ .type = MW_OP_WRITEREG0, .wdata = val };
	return ERROR_OK;
}

/*
 * run() - translate queued operations to MPSSE telegrams and execute.
 *
 * Each run() prepends a session-refresh preamble (set_io_client + writereg_0)
 * to re-assert ownership of the DAP bus, matching the behavior seen in
 * ftdi_dap.py's DAPOperations where every read/write starts with this pair.
 */
static int mw_op_run(struct ocmts *ocmts)
{
	/* Without a DAP context there is no target to talk to. Reporting success
	 * here would leave every read destination untouched, i.e. hand the
	 * caller whatever happened to be in its buffer and pass it off as target
	 * memory. */
	if (!g_dap) {
		/* Drop the batch rather than keep it: the sequence it describes is
		 * already broken, and leaving it queued would have the next,
		 * unrelated run() replay it. */
		g_nops = 0;
		return ERROR_FAIL;
	}
	if (g_nops == 0)
		return ERROR_OK;

	/* Preamble: assert DAP IO client ownership and select the IO client. */
	ifxdap_set_io_client(g_dap, 1);
	ifxdap_writereg_0(g_dap, g_ioconf_preamble);

	for (size_t i = 0; i < g_nops; i++) {
		struct mw_op *op = &g_ops[i];
		switch (op->type) {
		case MW_OP_SET_ADDRESS:
			ifxdap_select_addr(g_dap, op->addr);
			g_cur_addr = op->addr;
			break;

		case MW_OP_WRITE_WORD:
			ifxdap_write32(g_dap, (uint32_t)op->wdata);
			break;

		case MW_OP_READ_WORD:
			ifxdap_read32(g_dap, (uint32_t *)op->dst);
			break;

		case MW_OP_WRITE_HWORD:
			ifxdap_write16(g_dap, (uint16_t)op->wdata);
			break;

		case MW_OP_READ_HWORD:
			ifxdap_read16(g_dap, (uint16_t *)op->dst);
			break;

		case MW_OP_WRITE_BYTE:
			ifxdap_write8(g_dap, (uint8_t)op->wdata);
			break;

		case MW_OP_READ_BYTE:
			ifxdap_read8(g_dap, (uint8_t *)op->dst);
			break;

		case MW_OP_WRITE_BLOCK: {
			/*
			 * Block write uses DAP cmdid=9 (dap_writemem): one
			 * telegram sets address+size, then each 32-bit word
			 * follows as a payload telegram.
			 *
			 * dap_writemem: cmdid=9, arglen=40,
			 *   arg = (size_bytes << 8) | addr_low8 for simple
			 *   or the full 40-bit :{addr[31:0], count_bytes[7:0]}
			 * From ftdi_dap.py dap_writemem(addr, size):
			 *   telegram(9, 40, addr | (size << 32), rspsize)
			 */
			uint32_t size_bytes = (uint32_t)(op->count * 4);
			uint64_t arg = (uint64_t)g_cur_addr |
						((uint64_t)size_bytes << 32);
			ifxdap_telegram(g_dap, IFXDAP_CMD_WRITEMEM, 40, arg,
					7, NULL, 0);
			const uint32_t *words = (const uint32_t *)op->wbuf;
			for (size_t w = 0; w < op->count; w++) {
				/* Each word: writereg(4, word, 32) */
				uint64_t warg = ((uint64_t)words[w] << 4) | 4;
				ifxdap_telegram(g_dap, IFXDAP_CMD_WRITEREG,
						36, warg, 7, NULL, 0);
			}
			break;
		}

		case MW_OP_READ_BLOCK: {
			/*
			 * Block read uses DAP cmdid=10 (dap_readmem): one
			 * telegram sets address+size, then each 32-bit word is
			 * returned. Response ends with a CRC32. We capture
			 * the result directly into the caller's buffer.
			 *
			 * dap_readmem(addr, size): telegram(10, 40, arg, rspsize)
			 * rspsize = (count * 4 + 4 + 7) bytes (value + crc32 + framing)
			 */
			uint32_t size_bytes = (uint32_t)(op->count * 4);
			uint64_t arg = (uint64_t)g_cur_addr |
						((uint64_t)size_bytes << 32);
			size_t rsp = op->count * 4 + 4 + 7; /* data + CRC32 + framing */
			if (!op->dst || op->count > (SIZE_MAX - 11) / 4) {
				LOG_ERROR("miniwiggler: bad block read request");
				break;
			}
			ifxdap_telegram(g_dap, IFXDAP_CMD_READMEM, 40, arg,
					rsp, (uint8_t *)op->dst,
					op->count * 4);
			break;
		}

		case MW_OP_SET_OJCONF:
			ifxdap_write_ojconf(g_dap, (uint16_t)op->wdata);
			break;

		case MW_OP_SET_ACR:
			/* ACR: writereg(0xA, acr, 8): cmdid=8, arglen=12, arg=(acr<<4)|0xA */
			ifxdap_telegram(g_dap, IFXDAP_CMD_WRITEREG, 12,
					((uint64_t)op->wdata << 4) | 0xA,
					7, NULL, 0);
			break;

		case MW_OP_WRITEREG0:
			/* Direct DAP register 0 (IOCONF) write */
			ifxdap_writereg_0(g_dap, (uint16_t)op->wdata);
			break;
		}
	}

	g_nops = 0;
	if (ifxdap_exec(g_dap) != 0)
		return ERROR_FAIL;

	/*
	 * Check the sticky OCMTS error latch.
	 *
	 * An aborted SRI transaction - reading erased PFLASH is the usual cause,
	 * since it raises an uncorrectable ECC error - sets IOINFO bit 2, after
	 * which every read returns 0x00000000 until the latch is cleared. Silently
	 * returning those zeros is what made a blank flash bank look like a
	 * permanent hardware read lockout, so report the failure to the caller and
	 * clear the latch so the session stays usable.
	 */
	uint16_t ioinfo = 0;
	if (ifxdap_read_ioinfo(g_dap, &ioinfo) != 0)
		return ERROR_FAIL;

	if (ioinfo & IFXDAP_IOINFO_ERROR) {
		LOG_DEBUG("miniwiggler: OCMTS transaction aborted (IOINFO=0x%04x), clearing", ioinfo);
		if (ifxdap_clear_error(g_dap) != 0) {
			LOG_ERROR("miniwiggler: failed to clear OCMTS error latch; "
					  "run 'miniwiggler reconnect' to recover");
			return ERROR_FAIL;
		}
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* ---------------------------------------------------------------------- */
/* adapter_driver callbacks                                                 */
/* ---------------------------------------------------------------------- */

static int mw_adapter_init(void)
{
	LOG_INFO("miniwiggler: opening FTDI device %04x:%04x (Channel A = DAP)",
		 mw_vid, mw_pid);
	g_dap = ifxdap_open(mw_vid, mw_pid, adapter_get_required_serial(),
		adapter_usb_get_location(), MW_CHANNEL, &mw_pins);
	if (!g_dap) {
		LOG_ERROR("miniwiggler: failed to open FTDI device %04x:%04x",
			  mw_vid, mw_pid);
		return ERROR_FAIL;
	}
	g_nops = 0;
	return ERROR_OK;
}

static int mw_adapter_quit(void)
{
	ifxdap_close(g_dap);
	g_dap = NULL;
	g_dap_connected = false;
	return ERROR_OK;
}

static int mw_adapter_reset(int srst, int trst)
{
	if (!g_dap)
		return ERROR_FAIL;

	/*
	 * The two arguments are not symmetric. For the ifxdap transport
	 * adapter_assert_reset() reaches the driver as reset(0, 1) and
	 * adapter_deassert_reset() as reset(0, 0) -- see adapter_system_reset()
	 * in src/jtag/core.c, which passes 0 as the first argument and the
	 * requested SRST state as the second. Only the second argument therefore
	 * distinguishes "assert" from "deassert". Hardware validation showed the
	 * assert path's full reset-and-handshake sequence is currently
	 * load-bearing for ocmts_recover_io_client(), so only strip the dead tail
	 * here; do not change that behaviour in this sweep.
	 */
	if (srst || trst) {
		/*
		 * The connect sequence, with its reset: measured on a TC4D7, it
		 * leaves the core halted on its first instruction every time (12 of
		 * 12, from a running application), while ifxdap_reset_and_run() let
		 * the application run first in about one reset out of four - a
		 * visible boot before the target layer got its halt. The target
		 * layer resumes the core for "reset run".
		 */
		if (ifxdap_connect_reset(g_dap, MW_SLOW_HZ, MW_FAST_HZ) < 0)
			return ERROR_FAIL;
		if (g_is_tc4x && ifxdap_tc4x_key_exchange(g_dap, 0, mw_tc4x_default_key,
					ARRAY_SIZE(mw_tc4x_default_key)) < 0)
			LOG_DEBUG("miniwiggler: TC4x key exchange after reset failed");
	}
	return ERROR_OK;
}

static int mw_adapter_speed(int speed)
{
	if (!g_dap)
		return ERROR_FAIL;
	ifxdap_set_clk_freq(g_dap, (uint32_t)speed * 1000);
	return ifxdap_exec(g_dap) == 0 ? ERROR_OK : ERROR_FAIL;
}

static int mw_adapter_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int mw_adapter_khz(int khz, int *speed)
{
	*speed = khz;
	return ERROR_OK;
}

static int mw_ifxdap_init(void)
{
	return ERROR_OK;
}

/* ---------------------------------------------------------------------- */
/* TCL command handlers                                                     */
/* ---------------------------------------------------------------------- */

/* miniwiggler probeocmts <addr> [<value>] -- read or write ONE 32-bit word
 * through the same CBS (DAP register) path that ocmts_init() drives, but with
 * no OCMTS unlock state machine, no work area and no dependence on any target.
 * It therefore still works after ocmts_init() has already failed, which is the
 * only way to tell two very different faults apart:
 *
 *   - an address reads 0x00000000 and IOINFO is clean  => the DAP reaches the
 *     chip, but the register being read is not the one we think it is (or OCDS
 *     is not enabled), i.e. a silicon-state / wrong-address problem;
 *   - reads error out with IOINFO bit 2 latched       => the DAP transaction
 *     itself is failing.
 *
 * DAP register IOINFO (0xB) is readable in both cases, so it is always printed.
 * This is a 32-bit access: use it only on 4-byte-aligned addresses.
 */
COMMAND_HANDLER(mw_cmd_probeocmts)
{
	uint32_t addr, val = 0;
	int do_write;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
	do_write = (CMD_ARGC == 2);
	if (do_write)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], val);

	if (!g_dap) {
		command_print(CMD, "miniwiggler: no DAP context (adapter not initialised)");
		return ERROR_FAIL;
	}

	/* 'ocmts init' may never have run, or may have failed after connecting,
	 * so make sure the DAP is up before touching a register. */
	if (!g_dap_connected) {
		if (ifxdap_connect(g_dap, MW_SLOW_HZ, MW_FAST_HZ) < 0) {
			command_print(CMD, "miniwiggler: DAP connect failed");
			return ERROR_FAIL;
		}
		g_dap_connected = true;
	}

	/* Same preamble mw_op_run() uses to take ownership of the DAP bus. */
	ifxdap_set_io_client(g_dap, 1);
	ifxdap_writereg_0(g_dap, g_ioconf_preamble);
	ifxdap_select_addr(g_dap, addr);
	if (do_write)
		ifxdap_write32(g_dap, val);
	else
		ifxdap_read32(g_dap, &val);

	int exec = ifxdap_exec(g_dap);

	/* Read IOINFO from a *fresh* batch, so it reflects the probe above and
	 * not a leftover telegram. */
	uint16_t ioinfo = 0;
	int ioinfo_rc = ifxdap_read_ioinfo(g_dap, &ioinfo);

	command_print(CMD, "miniwiggler: probeocmts %s 0x%08x = 0x%08x "
			  "(exec=%d ioinfo=%s0x%04x%s)",
			  do_write ? "wrote" : "read", addr, val, exec,
			  (ioinfo_rc == 0) ? "" : "unreadable ", ioinfo,
			  (ioinfo & IFXDAP_IOINFO_ERROR) ? " ERR-LATCH" : "");

	return ERROR_OK;
}

/* miniwiggler dapisc [<48-bit-key>|sync] -- run the DAPISC key exchange and
 * PRINT the response the driver normally throws away.
 *
 * ifxdap.c:562 clocks in 5 response bytes for every DAPISC telegram but passes
 * dst=NULL, so the key-exchange answer is never checked even though the comment
 * at ifxdap.c:627 says it is expected to be 0x400. This command makes that
 * answer observable. It re-runs the handshake from a clean DAP state, so it
 * resets the DAP; it is a diagnostic, not something to run mid-session.
 *
 *   (no arg)  driver's built-in key 0x4ABBAF530400
 *   sync      send ONLY the 16-bit 0xF00 sync telegram, no key exchange
 *   <n>       send the given 48-bit key instead, so a candidate alternative can
 *             be evaluated against the silicon without editing code
 *
 * IOINFO is sampled immediately before and after the DAPISC, so "the DAPISC
 * went unanswered" can be separated from "the DAP stopped answering altogether"
 * within a single session and a single DAP state. The key is only ever *sent*;
 * nothing here fabricates a "correct" one.
 */
COMMAND_HANDLER(mw_cmd_dapisc)
{
	uint64_t key = 0x4ABBAF530400ULL;
	bool send_key = true;

	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1) {
		if (!strcmp(CMD_ARGV[0], "sync"))
			send_key = false;
		else
			COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], key);
	}

	if (!g_dap) {
		command_print(CMD, "miniwiggler: no DAP context (adapter not initialised)");
		return ERROR_FAIL;
	}

	/* Establish a known-good IOINFO reading first, from the same DAP state
	 * the DAPISC telegram will be issued from. */
	uint16_t ioinfo_before = 0;
	if (ifxdap_read_ioinfo(g_dap, &ioinfo_before) != 0)
		command_print(CMD, "miniwiggler: dapisc: IOINFO before unreadable");
	else
		command_print(CMD, "miniwiggler: dapisc: IOINFO before = 0x%04x", ioinfo_before);

	/* Clean slate: same step 1 ifxdap_connect() uses, so the DAPISC answer is
	 * not a leftover from a previous session. */
	ifxdap_test_reset(g_dap);
	ifxdap_set_clk_freq(g_dap, MW_SLOW_HZ);
	ifxdap_reset(g_dap);
	if (ifxdap_exec(g_dap) < 0) {
		command_print(CMD, "miniwiggler: dapisc: reset/flush failed");
		return ERROR_FAIL;
	}

	/* 5 response bytes are captured into a block slot (valbytes > 4) so the
	 * full little-endian stream is visible, not a truncated scalar.
	 * An all-zero result means no framing '1' bit was clocked in at all:
	 * decode_find_payload() (ifxdap.c:185-197) returns nbits in that case,
	 * so this is "the DAP drove no response", not a decoding artefact. */
	uint8_t sync_rsp[8] = { 0 };
	uint8_t key_rsp[8] = { 0 };

	ifxdap_telegram(g_dap, IFXDAP_CMD_DAPISC, 16, 0xF00, 5, sync_rsp, 5);
	if (send_key)
		ifxdap_telegram(g_dap, IFXDAP_CMD_DAPISC, 48, key, 5, key_rsp, 5);

	if (ifxdap_exec(g_dap) < 0) {
		command_print(CMD, "miniwiggler: dapisc: exec failed");
		return ERROR_FAIL;
	}

	command_print(CMD, "miniwiggler: dapisc mode=%s key=0x%012" PRIx64
			  " sync_rsp=%02x%02x%02x%02x%02x key_rsp=%02x%02x%02x%02x%02x",
			  send_key ? "key" : "sync-only", key,
			  sync_rsp[0], sync_rsp[1], sync_rsp[2], sync_rsp[3], sync_rsp[4],
			  key_rsp[0], key_rsp[1], key_rsp[2], key_rsp[3], key_rsp[4]);

	/* The first four bytes little-endian are the value the protocol docs
	 * quote as 0x400 for an accepted key. */
	uint32_t word = (uint32_t)key_rsp[0] | ((uint32_t)key_rsp[1] << 8) |
			((uint32_t)key_rsp[2] << 16) | ((uint32_t)key_rsp[3] << 24);
	if (send_key)
		command_print(CMD, "miniwiggler: dapisc key_rsp[0..3] as u32le = 0x%08" PRIx32
				  "%s", word,
				  (word == 0x400) ? "  (matches the documented 0x400)" :
						"  (DOES NOT match the documented 0x400)");
	else
		command_print(CMD, "miniwiggler: dapisc sync_rsp[0..3] as u32le = 0x%08" PRIx32,
			word);

	/* Does the DAP still answer a normal register read afterwards? */
	uint16_t ioinfo_after = 0;
	if (ifxdap_read_ioinfo(g_dap, &ioinfo_after) != 0)
		command_print(CMD, "miniwiggler: dapisc: IOINFO after unreadable");
	else
		command_print(CMD, "miniwiggler: dapisc: IOINFO after = 0x%04x "
			  "(DAP still answers register reads: %s)",
			  ioinfo_after, "yes");

	/* Leave the DAP in the same state ifxdap_connect() would have left it,
	 * so a following probeocmts is comparable. */
	ifxdap_set_io_client(g_dap, 1);
	ifxdap_write_ojconf(g_dap, 0x4501);
	ifxdap_writereg_0(g_dap, g_ioconf_preamble);
	ifxdap_set_clk_freq(g_dap, MW_FAST_HZ);
	if (ifxdap_exec(g_dap) < 0) {
		command_print(CMD, "miniwiggler: dapisc: failed to restore normal DAP state");
		return ERROR_FAIL;
	}
	g_dap_connected = true;

	return ERROR_OK;
}

/* miniwiggler tc4x_keyexchange [<w0> <w1> ... <w7>] -- explicitly invoke
 * ifxdap_tc4x_key_exchange() and report its result directly, rather than
 * relying on mw_op_connect()'s own call, which only LOG_WARNINGs on failure
 * and always returns ERROR_OK (see that function's comment) precisely so a
 * failed exchange does not abort connect on boards with a real, unknown
 * provisioned key. This command exists to make the exchange's own outcome a
 * first-class, directly observable, scriptable result.
 *
 * Same spirit as 'dapisc': the key is only ever *sent*, nothing here
 * fabricates a "correct" one. Unlike 'dapisc' (whose 48-bit DAPISC key
 * exchange has no reported silicon side effect beyond the response word),
 * an INCORRECT TC4x COMDATA key's effect on silicon state is NOT
 * characterized by this project -- treat this the same as any other
 * authentication/credential operation against physical hardware, not as a
 * side-effect-free probe.
 *
 *   (no args)       the driver's built-in all-zero das_default_key256
 *                    (mw_tc4x_default_key, 8 words) -- the one combination
 *                    already hardware-validated (tc4-work-2026/KEYEXCHANGE.md)
 *   <w0> .. <w7>     exactly 8 32-bit words (das_key_256bit_t.key[8] shape),
 *                    MSB-first word order, to evaluate any other candidate
 *                    key against the silicon
 *
 * Requires the DAP to already be connected (run 'init' first): unlike
 * probeocmts/dapisc, this does not auto-connect, so the already-automatic
 * connect-time call (mw_op_connect(), TC4x only) always runs first and this
 * command's own result is never the very first handshake attempt of a
 * session.
 */
COMMAND_HANDLER(mw_cmd_tc4x_keyexchange)
{
	uint32_t key[8];
	const uint32_t *use_key = mw_tc4x_default_key;

	if (CMD_ARGC != 0 && CMD_ARGC != ARRAY_SIZE(key))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (ARRAY_SIZE(key) == CMD_ARGC) {
		for (unsigned int i = 0; i < ARRAY_SIZE(key); i++)
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i], key[i]);
		use_key = key;
	}

	if (!g_dap) {
		command_print(CMD, "miniwiggler: no DAP context (adapter not initialised)");
		return ERROR_FAIL;
	}
	if (!g_dap_connected) {
		command_print(CMD, "miniwiggler: tc4x_keyexchange: DAP not connected "
				  "(run 'init' first)");
		return ERROR_FAIL;
	}

	command_print(CMD, "miniwiggler: tc4x_keyexchange: sending 8-word key "
			  "%08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " "
			  "%08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32,
			  use_key[0], use_key[1], use_key[2], use_key[3],
			  use_key[4], use_key[5], use_key[6], use_key[7]);

	int rc = ifxdap_tc4x_key_exchange(g_dap, 0, use_key, ARRAY_SIZE(key));

	command_print(CMD, "miniwiggler: tc4x_keyexchange: %s",
			  (rc == 0) ? "OK (every word verified)" :
				"FAILED (see LOG_WARNING above for which word)");

	return (rc == 0) ? ERROR_OK : ERROR_FAIL;
}

/* miniwiggler probeocmtsbatch <oec_addr> <ostate_addr> <v1> <v2> [<v3> ...]
 *
 * Reproduces, in ONE ifxdap_exec() batch, exactly the operation sequence that
 * ocmts_init() sends (src/target/tricore/ocmts.c):
 *
 *   set_io_client(1); writereg_0(0xC1);   [preamble, as mw_op_run does]
 *   select_addr(oec_addr);
 *   write32(v1); write32(v2); ...        [the key words]
 *   select_addr(ostate_addr);
 *   read32(&ostate);
 *
 * Why this exists: 'miniwiggler probeocmts <addr> <value>' issues every write
 * as its OWN batch, so it cannot test whether the key has to be atomic within a
 * single DAP transaction. The driver sends it atomically; this makes that
 * observable. It is a faithful reproduction of existing driver behaviour, not a
 * new protocol.
 *
 * Also prints the OEC address read back immediately afterwards, so a write that
 * the chip echoed can be distinguished from one it ignored.
 */
COMMAND_HANDLER(mw_cmd_probeocmtsbatch)
{
	uint32_t oec, ostate_addr, ostate = 0, oec_readback = 0;
	unsigned int i;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], oec);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], ostate_addr);

	if (!g_dap) {
		command_print(CMD, "miniwiggler: no DAP context (adapter not initialised)");
		return ERROR_FAIL;
	}

	if (!g_dap_connected) {
		if (ifxdap_connect(g_dap, MW_SLOW_HZ, MW_FAST_HZ) < 0) {
			command_print(CMD, "miniwiggler: DAP connect failed");
			return ERROR_FAIL;
		}
		g_dap_connected = true;
	}

	/* Same preamble mw_op_run() uses to take ownership of the DAP bus. */
	ifxdap_set_io_client(g_dap, 1);
	ifxdap_writereg_0(g_dap, g_ioconf_preamble);
	ifxdap_select_addr(g_dap, oec);

	for (i = 2; i < CMD_ARGC; i++) {
		uint32_t v;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i], v);
		ifxdap_write32(g_dap, v);
	}

	ifxdap_select_addr(g_dap, ostate_addr);
	ifxdap_read32(g_dap, &ostate);

	int exec = ifxdap_exec(g_dap);

	/* Separate batch: read the OEC register back to see whether the key was
	 * echoed or ignored. Sentinel first so "no response" is distinguishable. */
	ifxdap_set_io_client(g_dap, 1);
	ifxdap_writereg_0(g_dap, g_ioconf_preamble);
	ifxdap_select_addr(g_dap, oec);
	ifxdap_read32(g_dap, &oec_readback);
	int exec2 = ifxdap_exec(g_dap);

	uint16_t ioinfo = 0;
	int ioinfo_rc = ifxdap_read_ioinfo(g_dap, &ioinfo);

	command_print(CMD, "miniwiggler: probeocmtsbatch %d words to 0x%08x in ONE batch: "
			  "OSTATE(0x%08x)=0x%08x OEC(0x%08x)=0x%08x "
			  "(exec=%d/%d ioinfo=%s0x%04x%s)",
			  CMD_ARGC - 2, oec, ostate_addr, ostate, oec, oec_readback,
			  exec, exec2, (ioinfo_rc == 0) ? "" : "unreadable ", ioinfo,
			  (ioinfo & IFXDAP_IOINFO_ERROR) ? " ERR-LATCH" : "");

	return ERROR_OK;
}

/* Milliseconds elapsed since @a start, both taken from timeval_ms().
 * timeval_ms() returns a negative value if the platform monotonic clock
 * could not be read; report that as 0 instead of wrapping around. */
static unsigned long mw_elapsed_ms(int64_t start)
{
	int64_t now = timeval_ms();

	return now < start ? 0UL : (unsigned long)(now - start);
}

/* miniwiggler wdtprobe <addr> <samples> <delay_ms> [reset]
 *
 * Time-resolved sampler: reset the chip, then read one address repeatedly,
 * printing an elapsed timestamp per sample. This exists to test the safety
 * watchdog hypothesis WITHOUT needing any watchdog register address.
 *
 * The hypothesis predicts a transient window: a watchdog that fires some fixed
 * time after reset would give a working OCDS immediately after the reset and a
 * dead one later. So the signature to look for is a non-zero value in the early
 * samples that decays to zero, or a value that is non-zero only on the first
 * sample. A flat all-zero trace with no time dependence argues against a simple
 * timeout and for a static condition.
 *
 * Addresses come from the user and must be real ones. Nothing here is a
 * substitute for sourcing them; this only samples whatever address is named.
 */
COMMAND_HANDLER(mw_cmd_wdtprobe)
{
	uint32_t addr, val = 0;
	unsigned int samples, i, delay_ms = 0;
	int do_reset = 0;
	int64_t t0;
	unsigned long ms;

	if (CMD_ARGC < 3 || CMD_ARGC > 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], addr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], samples);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], delay_ms);
	if (CMD_ARGC == 4)
		do_reset = !strcmp(CMD_ARGV[3], "reset");

	if (!g_dap) {
		command_print(CMD, "miniwiggler: no DAP context (adapter not initialised)");
		return ERROR_FAIL;
	}

	if (do_reset) {
		/* Same sequence ifxdap_connect() uses: TRST pulse, slow clock,
		 * hardware reset, DAPISC handshake. */
		if (ifxdap_connect(g_dap, MW_SLOW_HZ, MW_FAST_HZ) < 0) {
			command_print(CMD, "miniwiggler: wdtprobe: reset/connect failed");
			return ERROR_FAIL;
		}
		g_dap_connected = true;
		command_print(CMD, "miniwiggler: wdtprobe: reset done, sampling %u times "
			  "from 0x%08x every %u ms", samples, addr, delay_ms);
	} else {
		if (!g_dap_connected) {
			if (ifxdap_connect(g_dap, MW_SLOW_HZ, MW_FAST_HZ) < 0) {
				command_print(CMD, "miniwiggler: wdtprobe: connect failed");
				return ERROR_FAIL;
			}
			g_dap_connected = true;
		}
		command_print(CMD, "miniwiggler: wdtprobe: sampling %u times from "
			  "0x%08x every %u ms (no reset)", samples, addr, delay_ms);
	}

	t0 = timeval_ms();

	for (i = 0; i < samples; i++) {
		uint16_t ioinfo = 0;

		ifxdap_set_io_client(g_dap, 1);
		ifxdap_writereg_0(g_dap, g_ioconf_preamble);
		ifxdap_select_addr(g_dap, addr);
		ifxdap_read32(g_dap, &val);
		if (ifxdap_exec(g_dap) < 0) {
			command_print(CMD, "miniwiggler: wdtprobe: exec failed at sample %u", i);
			return ERROR_FAIL;
		}

		if (ifxdap_read_ioinfo(g_dap, &ioinfo) < 0) {
			command_print(CMD, "miniwiggler: wdtprobe: IOINFO read failed at sample %u", i);
			return ERROR_FAIL;
		}

		ms = mw_elapsed_ms(t0);

		command_print(CMD, "  wdtprobe[%3u] t=+%5lu ms  0x%08x = 0x%08x  ioinfo=0x%04x%s",
			i, ms, addr, val, ioinfo,
			(ioinfo & IFXDAP_IOINFO_ERROR) ? " ERR-LATCH" : "");

		if (delay_ms)
			alive_sleep(delay_ms);
	}

	return ERROR_OK;
}

/* miniwiggler poweronwatch <secs> <delay_ms> <serial|""> <addr1> [<addr2> ...]
 *
 * Long-running, timestamped, multi-address sampler built to survive a POWER
 * CYCLE of the board underneath it.
 *
 * A power cycle resets the FTDI chip as well as the target, so the USB handle
 * taken at adapter init goes stale and the device re-enumerates, possibly at a
 * DIFFERENT port path. A watcher that does not handle that dies at exactly the
 * moment it was meant to observe. So on any batch failure this re-opens the
 * device by USB serial with the location deliberately passed as NULL, then
 * re-runs the full DAP handshake, and keeps sampling.
 *
 * Why serial and not location: locations shuffle across a power cycle, which is
 * why this whole investigation is serial-addressed. The miniWiggler serials on
 * this host are distinct, so the serial identifies the board unambiguously.
 *
 * A single non-zero sample on ANY address is the positive result. All-zero
 * across the whole power-on window is the negative one.
 *
 * An extra log-file argument may be given as the 2nd argument, before <secs>:
 *   miniwiggler poweronwatch <logfile> <secs> <delay_ms> <serial|""> <addr...>
 * When present, every line is written there and fflush()ed immediately. That is
 * the form to use for a live test: OpenOCD's own stdout is buffered when it is
 * redirected to a file, so relying on it leaves the log empty until the process
 * exits, which is useless when a human is power-cycling the board in the
 * meantime.
 */
COMMAND_HANDLER(mw_cmd_poweronwatch)
{
	const char *serial = NULL;
	const char *logpath = NULL;
	unsigned int secs, delay_ms, i, a, argbase;
	unsigned int naddr;
	uint32_t *addrs;
	int64_t t0;
	unsigned long ms, last_report = 0;
	int connected = 0;
	FILE *lf = NULL;

	/* Optional leading log file. */
	if (CMD_ARGC >= 5 && !strncmp(CMD_ARGV[0], "/", 1)) {
		logpath = CMD_ARGV[0];
		argbase = 1;
	} else {
		argbase = 0;
	}

	if ((int)(CMD_ARGC - argbase) < 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[argbase + 0], secs);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[argbase + 1], delay_ms);
	if (CMD_ARGV[argbase + 2][0])
		serial = CMD_ARGV[argbase + 2];

	naddr = (unsigned int)(CMD_ARGC - argbase) - 3;
	addrs = calloc(naddr, sizeof(*addrs));
	if (!addrs)
		return ERROR_FAIL;

	for (a = 0; a < naddr; a++)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[argbase + 3 + a], addrs[a]);

	if (logpath) {
		lf = fopen(logpath, "w");
		if (!lf) {
			command_print(CMD, "miniwiggler: poweronwatch: cannot open %s",
				logpath);
			free(addrs);
			return ERROR_FAIL;
		}
	}

	command_print(CMD, "miniwiggler: poweronwatch for %u s, %u ms/round, %u addrs, "
			  "serial=%s, logfile=%s", secs, delay_ms, naddr,
			  serial ? serial : "(none)", logpath ? logpath : "(none)");

	t0 = timeval_ms();

	while (1) {
		ms = mw_elapsed_ms(t0);
		if (ms >= (unsigned long)secs * 1000UL)
			break;

		if (!connected) {
			/* (Re)open by serial, location NULL, then handshake. This
			 * is the path that survives re-enumeration. */
			if (g_dap) {
				ifxdap_close(g_dap);
				g_dap = NULL;
			}
			g_dap_connected = false;
			g_dap = ifxdap_open(mw_vid, mw_pid, serial, NULL,
					   MW_CHANNEL, &mw_pins);
			if (!g_dap) {
				command_print(CMD, "  POWERON t=+%6lu ms  open failed "
					  "(board off or re-enumerating)", ms);
				if (lf) {
					fprintf(lf, "  POWERON t=+%7lu ms  open failed "
						"(board off or re-enumerating)\n", ms);
					fflush(lf);
				}
				alive_sleep(delay_ms ? delay_ms : 50);
				continue;
			}
			if (ifxdap_connect(g_dap, MW_SLOW_HZ, MW_FAST_HZ) < 0) {
				command_print(CMD, "  POWERON t=+%6lu ms  connect failed",
					ms);
				if (lf) {
					fprintf(lf, "  POWERON t=+%7lu ms  connect failed\n",
						ms);
					fflush(lf);
				}
				alive_sleep(delay_ms ? delay_ms : 50);
				continue;
			}
			g_dap_connected = true;
			connected = 1;
			command_print(CMD, "  POWERON t=+%6lu ms  CONNECTED", ms);
			if (lf) {
				fprintf(lf, "  POWERON t=+%7lu ms  CONNECTED\n", ms);
				fflush(lf);
			}
		}

		/* One round: every address, plus IOINFO. */
		for (i = 0; i < naddr; i++) {
			uint32_t v = 0;
			uint16_t ioinfo = 0;

			ifxdap_set_io_client(g_dap, 1);
			ifxdap_writereg_0(g_dap, g_ioconf_preamble);
			ifxdap_select_addr(g_dap, addrs[i]);
			ifxdap_read32(g_dap, &v);
			if (ifxdap_exec(g_dap) < 0) {
				command_print(CMD, "  POWERON t=+%6lu ms  exec failed "
					  "on 0x%08x -> re-opening", ms, addrs[i]);
				if (lf) {
					fprintf(lf, "  POWERON t=+%7lu ms  exec failed on "
						"0x%08x -> re-opening\n", ms, addrs[i]);
					fflush(lf);
				}
				connected = 0;
				break;
			}
			if (ifxdap_read_ioinfo(g_dap, &ioinfo) < 0) {
				command_print(CMD, "  POWERON t=+%6lu ms  IOINFO read failed "
					"on 0x%08x -> re-opening", ms, addrs[i]);
				if (lf) {
					fprintf(lf, "  POWERON t=+%7lu ms  IOINFO read failed on "
						"0x%08x -> re-opening\n", ms, addrs[i]);
					fflush(lf);
				}
				connected = 0;
				break;
			}

			ms = mw_elapsed_ms(t0);

			/* Always log. A non-zero value is the positive, and the
			 * point is to have the whole window on record. */
			command_print(CMD,
				"  POWERON t=+%7lu ms  0x%08x = 0x%08x%s "
				"ioinfo=0x%04x", ms, addrs[i], v,
				v ? "  <=== NON-ZERO" : "",
				ioinfo);
			if (lf) {
				fprintf(lf,
					"  POWERON t=+%7lu ms  0x%08x = 0x%08x%s "
					"ioinfo=0x%04x\n", ms, addrs[i], v,
					v ? "  <=== NON-ZERO" : "", ioinfo);
				fflush(lf);
			}
		}

		if (connected) {
			/* Progress line, so a long run is visibly alive. */
			if (ms - last_report >= 1000UL) {
				last_report = ms;
				command_print(CMD, "  POWERON t=+%7lu ms  (still "
					"sampling)", ms);
				if (lf) {
					fprintf(lf,
						"  POWERON t=+%7lu ms  (still sampling)\n",
						ms);
					fflush(lf);
				}
			}
			alive_sleep(delay_ms);
		}
	}

	command_print(CMD, "miniwiggler: poweronwatch finished after %u s", secs);
	if (lf) {
		fprintf(lf, "  POWERON finished after %u s\n", secs);
		fclose(lf);
	}
	free(addrs);
	return ERROR_OK;
}

COMMAND_HANDLER(mw_cmd_vid_pid)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], mw_vid);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], mw_pid);
	LOG_INFO("miniwiggler: using VID:PID %04x:%04x", mw_vid, mw_pid);
	return ERROR_OK;
}

/* miniwiggler ioinfo - read the DAP IOINFO status register (bit 2 = error latch). */
COMMAND_HANDLER(mw_cmd_ioinfo)
{
	uint16_t ioinfo = 0;

	if (!g_dap || ifxdap_read_ioinfo(g_dap, &ioinfo) != 0) {
		command_print(CMD, "miniwiggler: IOINFO read failed");
		return ERROR_FAIL;
	}
	LOG_INFO("miniwiggler: IOINFO = 0x%04x%s", ioinfo,
			 (ioinfo & IFXDAP_IOINFO_ERROR) ? " (transaction error latched)" : "");
	return ERROR_OK;
}

/* miniwiggler reconnect - full DAP connect handshake; resets the chip. */
COMMAND_HANDLER(mw_cmd_reconnect)
{
	if (!g_dap)
		return ERROR_FAIL;

	if (ifxdap_connect(g_dap, MW_SLOW_HZ, MW_FAST_HZ) < 0) {
		command_print(CMD, "miniwiggler: reconnect failed");
		return ERROR_FAIL;
	}
	LOG_INFO("miniwiggler: DAP reconnected");
	return ERROR_OK;
}

/* miniwiggler clearerr - clear the sticky OCMTS error latch, without resetting. */
COMMAND_HANDLER(mw_cmd_clearerr)
{
	if (!g_dap)
		return ERROR_FAIL;

	if (ifxdap_clear_error(g_dap) != 0) {
		command_print(CMD, "miniwiggler: failed to clear OCMTS error latch");
		return ERROR_FAIL;
	}
	LOG_INFO("miniwiggler: OCMTS error latch cleared");
	return ERROR_OK;
}

static const struct command_registration mw_subcommand_handlers[] = {
	{
		.name    = "ioinfo",
		.handler = mw_cmd_ioinfo,
		.mode    = COMMAND_EXEC,
		.usage   = "",
		.help    = "Read the DAP IOINFO status register",
	},
	{
		.name    = "clearerr",
		.handler = mw_cmd_clearerr,
		.mode    = COMMAND_EXEC,
		.usage   = "",
		.help    = "Clear the sticky OCMTS error latch without resetting the chip",
	},
	{
		.name    = "reconnect",
		.handler = mw_cmd_reconnect,
		.mode    = COMMAND_EXEC,
		.usage   = "",
		.help    = "Redo the full DAP connect handshake",
	},
	{
		.name    = "probeocmts",
		.handler = mw_cmd_probeocmts,
		.mode    = COMMAND_EXEC,
		.usage   = "<addr> [<value>]",
		.help    = "Read/write one 32-bit word via the DAP CBS path, "
			   "without the OCMTS unlock state machine",
	},
	{
		.name    = "dapisc",
		.handler = mw_cmd_dapisc,
		.mode    = COMMAND_EXEC,
		.usage   = "[<48-bit-key>|sync]",
		.help    = "Run the DAPISC key exchange and print the response the "
			   "driver discards (default key 0x4ABBAF530400)",
	},
	{
		.name    = "tc4x_keyexchange",
		.handler = mw_cmd_tc4x_keyexchange,
		.mode    = COMMAND_EXEC,
		.usage   = "[<w0> <w1> ... <w7>]",
		.help    = "Explicitly run the TC4x COMDATA key exchange (default: "
			   "built-in all-zero key) and report its own pass/fail",
	},
	{
		.name    = "probeocmtsbatch",
		.handler = mw_cmd_probeocmtsbatch,
		.mode    = COMMAND_EXEC,
		.usage   = "<oec_addr> <ostate_addr> <v1> <v2> [<v3> ...]",
		.help    = "Send the ocmts_init() key sequence to <oec_addr> in a "
			   "single DAP batch, then read <ostate_addr> back",
	},
	{
		.name    = "wdtprobe",
		.handler = mw_cmd_wdtprobe,
		.mode    = COMMAND_EXEC,
		.usage   = "<addr> <samples> <delay_ms> [reset]",
		.help    = "Time-resolved read of <addr>, optionally after a reset; "
			   "looks for a transient window (safety-watchdog signature)",
	},
	{
		.name    = "poweronwatch",
		.handler = mw_cmd_poweronwatch,
		.mode    = COMMAND_EXEC,
		.usage   = "[<logfile>] <secs> <delay_ms> <serial|\"\"> <addr1> ...",
		.help    = "Long timestamped multi-address sampler that re-opens the "
			   "device by serial, so it survives a board power cycle",
	},
	{
		.name    = "vid_pid",
		.handler = mw_cmd_vid_pid,
		.mode    = COMMAND_CONFIG,
		.usage   = "<vid> <pid>",
		.help    = "Override USB VID:PID (default 058b:0043)",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration mw_command_handlers[] = {
	{
		.name  = "miniwiggler",
		.mode  = COMMAND_ANY,
		.help  = "miniWiggler V3 native FTDI adapter commands",
		.usage = "<cmd>",
		.chain = mw_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/* ---------------------------------------------------------------------- */
/* Public struct - referenced in src/jtag/drivers/driver.c                */
/* ---------------------------------------------------------------------- */

static const struct ocmts_ops mw_ocmts_ops = {
	.connect              = mw_op_connect,
	.queue_io_set_address = mw_op_queue_set_address,
	.queue_io_write_word  = mw_op_queue_write_word,
	.queue_io_read_word   = mw_op_queue_read_word,
	.queue_io_write_hword = mw_op_queue_write_hword,
	.queue_io_read_hword  = mw_op_queue_read_hword,
	.queue_io_write_byte  = mw_op_queue_write_byte,
	.queue_io_read_byte   = mw_op_queue_read_byte,
	.queue_io_write_block = mw_op_queue_write_block,
	.queue_io_read_block  = mw_op_queue_read_block,
	.queue_io_set_ojconf  = mw_op_queue_set_ojconf,
	.queue_io_set_acr     = mw_op_queue_set_acr,
	.queue_io_writereg0   = mw_op_queue_writereg0,
	.run                  = mw_op_run,
};

static struct ifxdap_driver mw_ifxdap_driver = {
	.init = mw_ifxdap_init,
};

struct adapter_driver miniwiggler_adapter_driver = {
	.name                  = "miniwiggler",
	.commands              = mw_command_handlers,
	.transport_ids         = TRANSPORT_IFXDAP,
	.transport_preferred_id = TRANSPORT_IFXDAP,
	.ocmts_ops             = &mw_ocmts_ops,
	.ifxdap_ops            = &mw_ifxdap_driver,
	.init                  = mw_adapter_init,
	.quit                  = mw_adapter_quit,
	.reset                 = mw_adapter_reset,
	.speed                 = mw_adapter_speed,
	.speed_div             = mw_adapter_speed_div,
	.khz                   = mw_adapter_khz,
};
