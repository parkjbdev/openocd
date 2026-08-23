// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   OCMTS Interface Module for Infineon AURIX                             *
 *   Copyright (C) 2026 Infineon Technologies AG                           *
 ***************************************************************************/

#include <inttypes.h>
#include <string.h>

#include <helper/command.h>
#include <helper/jim-nvp.h>
#include <helper/list.h>
#include <helper/log.h>
#include <helper/time_support.h>
#include <jim.h>
#include <jtag/adapter.h>
#include <jtag/ifxdap.h>
#include <jtag/interface.h>
#include <jtag/jtag.h>
#include <target/tricore/aurix.h>
#include <transport/transport.h>

#include "ocmts.h"

static OOCD_LIST_HEAD(all_ocmts);

struct ocmts *ocmts_by_jim_obj(Jim_Interp *interp, Jim_Obj *o)
{
	struct ocmts *ocmts;
	const char *name = Jim_GetString(o, NULL);

	list_for_each_entry(ocmts, &all_ocmts, lh) {
		if (strcmp(name, ocmts->name) == 0)
			return ocmts;
	}

	return NULL;
}

enum dap_cfg_param {
	CFG_CHAIN_POSITION,
};

static const struct jim_nvp nvp_config_opts[] = {{.name = "-chain-position", .value = CFG_CHAIN_POSITION},
												 {.name = NULL, .value = -1}};

static int ocmts_configure(struct jim_getopt_info *goi, struct ocmts *ocmts)
{
	struct jim_nvp *n;
	int e;

	/* parse config ... */
	while (goi->argc > 0) {
		Jim_SetEmptyResult(goi->interp);

		e = jim_getopt_nvp(goi, nvp_config_opts, &n);
		if (e != JIM_OK) {
			jim_getopt_nvp_unknown(goi, nvp_config_opts, 0);
			return e;
		}
		switch (n->value) {
		case CFG_CHAIN_POSITION: {
			Jim_Obj *o_t;
			e = jim_getopt_obj(goi, &o_t);
			if (e != JIM_OK)
				return e;

			struct jtag_tap *tap;
			tap = jtag_tap_by_jim_obj(goi->interp, o_t);
			if (!tap) {
				Jim_SetResultString(goi->interp, "-chain-position is invalid", -1);
				return JIM_ERR;
			}
			ocmts->tap = tap;
			/* loop for more */
			break;
		}
		default:
			break;
		}
	}

	return JIM_OK;
}

COMMAND_HANDLER(ocmts_create_cmd)
{
	int retval = ERROR_COMMAND_ARGUMENT_INVALID;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Create ocmts instance */
	struct ocmts *ocmts = calloc(1, sizeof(struct ocmts));
	if (!ocmts) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	ocmts->name = strdup(CMD_ARGV[0]);
	if (!ocmts->name) {
		LOG_ERROR("Out of memory");
		free(ocmts);
		return ERROR_FAIL;
	}

	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, CMD_CTX->interp, CMD_ARGC - 1, CMD_JIMTCL_ARGV + 1);
	int e = ocmts_configure(&goi, ocmts);
	if (e != JIM_OK) {
		int reslen;
		const char *result = Jim_GetString(Jim_GetResult(CMD_CTX->interp), &reslen);
		if (reslen > 0)
			command_print(CMD, "%s", result);
		goto err;
	}

	list_add_tail(&ocmts->lh, &all_ocmts);

	return ERROR_OK;
err:
	free(ocmts->name);
	free(ocmts);
	return retval;
}

/* Enabling debug means writing a four word key sequence to OEC and then seeing
 * bit 0 of OSTATE come up.
 *
 * The OCMTS IO client sometimes comes up wedged just after the DAP connects.
 * A wedged client does not report a transport error: it accepts every write
 * and answers every read with zero.  So the key sequence lands nowhere, OSTATE
 * reads back as 0x00000000, and the caller sees an intermittent "Failed to
 * enable debug".
 *
 * The existing recovery -- pulse reset with OJCONF=0x3 -- clears the condition,
 * but it used to be reachable only when the *transport* failed, which is
 * exactly what a wedged client never does.  An all-zero OSTATE is therefore
 * treated as "not accessible" here rather than as a valid status word, so the
 * recovery actually runs, and the unlock is retried around it. */
#define OCMTS_UNLOCK_TRIES  8
#define OCMTS_POLL_TRIES    20
#define OCMTS_POLL_DELAY_MS 5
/* The IO client needs a moment after the DAP comes up before it answers reads.
 * Running with -d2 slows the session down enough to hide the problem entirely,
 * which is what identified it as a wake-up race rather than a bad key. */
#define OCMTS_READY_TRIES    40
#define OCMTS_READY_DELAY_MS 5

/* Bit 0 set means debug enabled, bit 16 means the device is locked.  A word
 * with neither bit and no reserved bits at all is not a state the hardware
 * reports; it is what a wedged IO client returns for every read. */
#define OCMTS_OSTATE_ENABLED 0x00000001
#define OCMTS_OSTATE_LOCKED  0x00010000

/* How many reset + full-DAP-reconnect cycles to try before giving up. */
#define OCMTS_UNWEDGE_TRIES 8

static int ocmts_read_ostate(struct ocmts *ocmts, uint32_t ostate_addr, uint32_t *ostate)
{
	*ostate = 0;
	int ret = ocmts_queue_io_set_address(ocmts, ostate_addr);
	if (ret)
		return ret;
	ret = ocmts_queue_io_read_word(ocmts, ostate);
	if (ret)
		return ret;
	return ocmts_run(ocmts);
}

/* Wait for the IO client to start answering reads.  Returns ERROR_OK with
 * *ostate still zero if it never does, so the caller can escalate to a reset.
 *
 * A read can fail outright instead of just coming back zero: right after a
 * DAP reconnect, the first SRI transaction has been observed to abort and
 * latch the adapter's sticky IOINFO error bit (the same bit an erased-flash
 * ECC error sets). The adapter driver already clears that latch before
 * returning the failure, so by the time it is seen here the condition is
 * already gone - retry exactly like a zero OSTATE instead of giving up on a
 * transaction that has already healed itself. */
static int ocmts_wait_io_client(struct ocmts *ocmts, uint32_t ostate_addr,
				uint32_t *ostate)
{
	for (unsigned int i = 0; i < OCMTS_READY_TRIES; i++) {
		if (ocmts_read_ostate(ocmts, ostate_addr, ostate) == ERROR_OK && *ostate != 0)
			return ERROR_OK;
		alive_sleep(OCMTS_READY_DELAY_MS);
	}
	return ERROR_OK;
}

/* Pulse reset with OJCONF=0x3 to unwedge the IO client, then wait for it. */
static int ocmts_recover_io_client(struct ocmts *ocmts, uint32_t ostate_addr,
				   uint32_t *ostate)
{
	int err = adapter_assert_reset();
	if (err != ERROR_OK)
		return err;

	err = ocmts->ops->queue_io_set_ojconf(ocmts, 0x3);
	if (err != ERROR_OK)
		return err;

	err = adapter_deassert_reset();
	if (err != ERROR_OK)
		return err;

	return ocmts_wait_io_client(ocmts, ostate_addr, ostate);
}

/* A target reset cannot clear a wedge that lives in the adapter's IO client;
 * only redoing the DAP handshake from scratch can.  Escalate through both. */
static int ocmts_unwedge_io_client(struct ocmts *ocmts, uint32_t ostate_addr,
				   uint32_t *ostate)
{
	for (unsigned int attempt = 0; attempt < OCMTS_UNWEDGE_TRIES; attempt++) {
		int err = ocmts_recover_io_client(ocmts, ostate_addr, ostate);
		if (err)
			return err;
		if (*ostate != 0)
			return ERROR_OK;

		if (!ocmts->ops->connect) {
			LOG_ERROR("[OCMTS] UNWEDGE: no connect op available");
			break;
		}

		LOG_DEBUG("[OCMTS] still wedged, redoing DAP handshake (attempt %u)",
			  attempt + 1);
		err = ocmts->ops->connect(ocmts);
		if (err) {
			LOG_ERROR("[OCMTS] UNWEDGE: reconnect failed on attempt %u", attempt + 1);
			return err;
		}

		err = ocmts_wait_io_client(ocmts, ostate_addr, ostate);
		if (err)
			return err;
		if (*ostate != 0)
			return ERROR_OK;
	}
	LOG_ERROR("[OCMTS] UNWEDGE: exhausted %u reset+reconnect cycles", OCMTS_UNWEDGE_TRIES);
	return ERROR_OK;
}

/* The ifxdap transport never shifts a real IDCODE/BYPASS scan (its
 * transport .init is a no-op), so tap->idcode is always left at its
 * zero-initialized default. aurix_tap_device_type() falls back to the
 * device type configured on the TAP via 'newtap ... -expected-id', which is
 * the only value actually known to be correct. */
int ocmts_init(struct ocmts *ocmts)
{
	int err;
	uint32_t pat0 = 0xA1, pat1 = 0x5E, ostate;
	uint32_t oec_addr = 0xF0000478;
	uint32_t ostate_addr = 0xF0000480;
	/* aurix_tap_device_type(), not tap->idcode directly: the ifxdap transport
	 * never scans a real IDCODE, so tap->idcode is always 0 and testing it
	 * would make the TC4x branch below permanently unreachable. */
	uint32_t dev_type = aurix_tap_device_type(ocmts->tap);

	if (aurix_device_is_tc4x(dev_type)) {
		oec_addr = 0xFA180068;
		ostate_addr = 0xFA18006C;
	}

	/* Report which OCMTS register pair was selected and why, so that a zero
	 * OSTATE does not have to be diagnosed by inference. */
	LOG_INFO("[OCMTS] tap idcode=0x%08" PRIx32 " dev_type=0x%08" PRIx32
		 " tc4x=%s -> OEC=0x%08" PRIx32 " OSTATE=0x%08" PRIx32,
		 ocmts->tap->idcode, dev_type,
		 aurix_device_is_tc4x(dev_type) ? "yes" : "no",
		 oec_addr, ostate_addr);

	/* A zero OSTATE here is not evidence of a wedged client: on a device
	 * whose debug has never been enabled this power cycle (the common
	 * case on a fresh connect), OSTATE is *supposed* to read zero until
	 * the key sequence below is written. Escalating to a reset+reconnect
	 * recovery here -- before the key has ever been written even once --
	 * cannot ever succeed, since none of that recovery writes the key;
	 * it only ever changes a client that is actually wedged, which is a
	 * state only distinguishable from "not yet unlocked" by trying the
	 * key first. See the unlock loop below for the real wedge check. */
	err = ocmts_wait_io_client(ocmts, ostate_addr, &ostate);
	if (err)
		return err;
	if (ostate & OCMTS_OSTATE_ENABLED) {
		/* already initialized */
		return ERROR_OK;
	}

	if (ostate & OCMTS_OSTATE_LOCKED) {
		LOG_ERROR("[OCMTS] Device is still locked");
		return ERROR_FAIL;
	}

	for (unsigned int attempt = 0; attempt < OCMTS_UNLOCK_TRIES; attempt++) {
		err = ocmts_queue_io_set_address(ocmts, oec_addr);
		if (err != ERROR_OK)
			return err;
		err = ocmts_queue_io_write_word(ocmts, &pat0);
		if (err != ERROR_OK)
			return err;
		err = ocmts_queue_io_write_word(ocmts, &pat1);
		if (err != ERROR_OK)
			return err;
		err = ocmts_queue_io_write_word(ocmts, &pat0);
		if (err != ERROR_OK)
			return err;
		err = ocmts_queue_io_write_word(ocmts, &pat1);
		if (err != ERROR_OK)
			return err;
		err = ocmts_queue_io_set_address(ocmts, ostate_addr);
		if (err != ERROR_OK)
			return err;
		err = ocmts_queue_io_read_word(ocmts, &ostate);
		if (err != ERROR_OK)
			return err;
		err = ocmts->ops->run(ocmts);
		if (err)
			return err;

		/* The status bit is not guaranteed to have settled by the time a
		 * read queued behind the key is shifted back out. */
		for (unsigned int poll = 0;
			(ostate & OCMTS_OSTATE_ENABLED) == 0 && poll < OCMTS_POLL_TRIES;
			poll++) {
			alive_sleep(OCMTS_POLL_DELAY_MS);
			err = ocmts_read_ostate(ocmts, ostate_addr, &ostate);
			if (err)
				return err;
		}

		if (ostate & OCMTS_OSTATE_ENABLED) {
			if (attempt)
				LOG_DEBUG("[OCMTS] debug enabled after %u retries", attempt);
			return ERROR_OK;
		}

		/* A locked device will never unlock, however many keys it is sent. */
		if (ostate & OCMTS_OSTATE_LOCKED) {
			LOG_ERROR("[OCMTS] Device is still locked");
			return ERROR_FAIL;
		}

		/* Still all zeros after a real key write: *now* the client is
		 * known to be wedged rather than simply not-yet-unlocked, so
		 * escalate through reset and, if that alone is not enough, a
		 * full DAP reconnect, before looping around to try the key
		 * again. */
		if (ostate == 0) {
			err = ocmts_unwedge_io_client(ocmts, ostate_addr, &ostate);
			if (err) {
				LOG_ERROR("[OCMTS] is not accessible via IO client after adapter reset");
				return err;
			}
			if (ostate & OCMTS_OSTATE_ENABLED) {
				LOG_DEBUG("[OCMTS] debug enabled after %u retries", attempt);
				return ERROR_OK;
			}
		}
	}

	LOG_ERROR("[OCMTS] Failed to enable debug (OSTATE 0x%08" PRIx32 " after %d attempts)",
		  ostate, OCMTS_UNLOCK_TRIES);
	return ERROR_FAIL;
}

COMMAND_HANDLER(ocmts_init_cmd)
{
	struct ocmts *ocmts;

	list_for_each_entry(ocmts, &all_ocmts, lh) {
		/* skip taps that are disabled */
		if (!ocmts->tap->enabled)
			continue;

		if (!transport_is_ifxdap()) {
			LOG_ERROR("OCMTS requires the ifxdap transport");
			return ERROR_FAIL;
		}

		if (!adapter_driver || !adapter_driver->ocmts_ops) {
			LOG_ERROR("selected adapter does not provide OCMTS support");
			return ERROR_FAIL;
		}

		ocmts->ops = adapter_driver->ocmts_ops;

		int err = ocmts->ops->connect(ocmts);
		if (err)
			return err;

		err = ocmts_init(ocmts);
		if (err)
			return err;
	}

	return ERROR_OK;
}

static const struct command_registration ocmts_subcommand_handlers[] = {
	{
		.name = "init",
		.mode = COMMAND_ANY,
		.handler = ocmts_init_cmd,
		.usage = "",
		.help = "Initialize all OCMTS systems",
	},
	{
		.name = "create",
		.mode = COMMAND_ANY,
		.handler = ocmts_create_cmd,
		.usage = "name '-chain-position' name",
		.help = "Create a new OCMTS instance",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ocmts_commands[] = {
	{
		.name = "ocmts",
		.mode = COMMAND_CONFIG,
		.help = "OCMTS commands",
		.chain = ocmts_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

int ocmts_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, ocmts_commands);
}

void ocmts_free_all(void)
{
	struct ocmts *ocmts, *next;

	list_for_each_entry_safe(ocmts, next, &all_ocmts, lh) {
		list_del(&ocmts->lh);
		free(ocmts->name);
		free(ocmts);
	}
}
