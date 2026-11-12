// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Infineon DAP (ifxdap) protocol library over FTDI MPSSE
 *
 * Protocol reverse-engineered by Enrico Pozzobon (epozzobon/tricore-things,
 * HackInBo 2025). This file ports his Python implementation to C, on top of
 * the generic FTDI MPSSE helpers in jtag/drivers/mpsse.[ch] rather than
 * libftdi, so any adapter built around an FTDI MPSSE channel can use it, not
 * just the Infineon miniWiggler.
 *
 * Copyright (C) 2026 Rafael J. Cruz <rafa.junio@hardenedlinux.org>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/ifxdap.h>

#include "drivers/mpsse.h"

#include <helper/log.h>
#include <helper/types.h>

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* -------------------------------------------------------------------------
 * AURIX DAP command IDs (from ftdi_dap.py DAPBatch class) not already
 * exposed in ifxdap.h for callers building their own telegrams.
 * ------------------------------------------------------------------------- */
/* DAP register addresses for read/write operations */
#define DAP_REG_IOADDR    0x1  /* IO address register (select_addr)  */
#define DAP_REG_READ8     0x9  /* Read 1 byte from IOADDR            */
#define DAP_REG_READ16    0x7  /* Read 2 bytes from IOADDR           */
#define DAP_REG_READ32    0x5  /* Read 4 bytes from IOADDR           */
#define DAP_REG_IOINFO    0xB  /* IOINFO status register             */
#define DAP_REG_OJCONF    0xE  /* OJCONF debug configuration         */

/* Size encoding added to register number for dap_readreg */
#define DAP_SIZE2  0x40  /* OR into reg for 2-byte read */
#define DAP_SIZE4  0x50  /* OR into reg for 4-byte read */

/* -------------------------------------------------------------------------
 * Batch context
 *
 * DAP telegram responses are clocked in raw (LSB first, framing bit and
 * padding included) into a scratch buffer, and decoded into the caller's
 * destination only once ifxdap_exec() confirms the whole batch, including
 * every queued read, was clocked in successfully.
 * ------------------------------------------------------------------------- */
#define IFXDAP_RAW_MAX      (64 * 1024)  /* scratch space for raw responses */
#define IFXDAP_RSP_SLOTS_MAX  512        /* max pending responses per batch */

struct ifxdap_rsp_slot {
	uint8_t *raw;     /* scratch buffer holding the raw clocked-in bytes  */
	size_t  nbytes;   /* length of the raw buffer                        */
	uint8_t *dst;     /* where to write the decoded response (NULL=discard) */
	size_t  valbytes; /* how many bytes of decoded value to keep          */
};

struct ifxdap_ctx {
	struct mpsse_ctx *mpsse;
	struct ifxdap_pins pins;

	uint8_t raw[IFXDAP_RAW_MAX];
	size_t  raw_len;

	struct ifxdap_rsp_slot rsp[IFXDAP_RSP_SLOTS_MAX];
	size_t rsp_count;

	/* Set when a queued response did not fit in raw[]/rsp[]. ifxdap_exec()
	 * fails the whole batch instead of decoding a truncated response. */
	bool overflow;

	/* CBS "max busy bits" budget and the 16-bit DAPISC selector derived
	 * from it, ((bits + 3) >> 3) << 8.  DAS keeps the same pair in
	 * CSelDevIfTc+0x74 and CSelDevIfTc+0x68. */
	unsigned int busy_bits;
	uint16_t      dapisc_sel;

	/* Post-reset settle time in milliseconds.  DAS stores this in
	 * CSelDevIfTc+0x64, initialises it to 400 in CSelDevIfTc::init()
	 * @ 0x2d83f, and varies it per device family. */
	unsigned int post_reset_delay_ms;
};

/* ---------------------------------------------------------------------- */
/* CRC6                                                                     */
/* ---------------------------------------------------------------------- */

/**
 * Compute 6-bit CRC over an LSB-first bit array.
 *
 * Direct port of:
 *   def compute_crc6(msg: bitarray) -> int:
 *       crc = 0x3F
 *       for bit in msg:
 *           b = (bit ^ crc ^ (crc >> 1)) & 1
 *           crc = (b << 5) | (crc >> 1)
 *       return crc ^ (crc >> 1)
 */
uint8_t ifxdap_crc6(const uint8_t *bits, size_t nbit)
{
	uint8_t crc = 0x3F;

	for (size_t i = 0; i < nbit; i++) {
		uint8_t b = (bits[i] ^ crc ^ (crc >> 1)) & 1;
		crc = (uint8_t)((b << 5) | (crc >> 1));
	}
	return crc ^ (crc >> 1);
}

/* ---------------------------------------------------------------------- */
/* Telegram encoding                                                        */
/* ---------------------------------------------------------------------- */

/*
 * DAP telegram wire format (LSB first, per dap_telegram() + dap2_encode()):
 *   [1][cmdid:5][arglen:6][arg:arglen][crc6:6]
 *   optionally appended with 0 if last bit is 1
 *   then MSB-zero-padded to byte boundary
 *
 * Returns number of encoded bytes written to `out`.
 * `out` must have at least 16 bytes of space.
 */
static size_t encode_telegram(uint8_t cmdid, uint8_t arglen, uint64_t arg,
					uint8_t *out)
{
	/* 1 start + 5 cmdid + 6 arglen + arglen arg + 6 crc + 1 pad, rounded up
	 * to a byte boundary, has to fit the scratch array. The wire format
	 * carries arglen in 6 bits and no caller needs more than 40. */
	uint8_t bits[128];
	size_t nb = 0;

	if (arglen > 63) {
		LOG_ERROR("ifxdap: telegram argument of %u bits does not fit the wire format",
			  arglen);
		return 0;
	}

	/* Start bit */
	bits[nb++] = 1;

	/* 5-bit cmdid, LSB first */
	for (int i = 0; i < 5; i++)
		bits[nb++] = (cmdid >> i) & 1;

	/* 6-bit arglen, LSB first */
	for (int i = 0; i < 6; i++)
		bits[nb++] = (arglen >> i) & 1;

	/* arglen bits of arg, LSB first */
	for (int i = 0; i < arglen; i++)
		bits[nb++] = (uint8_t)((arg >> i) & 1);

	/* CRC6 over all bits so far */
	uint8_t crc = ifxdap_crc6(bits, nb);

	/* Append 6-bit CRC, LSB first */
	for (int i = 0; i < 6; i++)
		bits[nb++] = (crc >> i) & 1;

	/* dap2_encode: if last bit is 1, append 0 */
	if (bits[nb - 1] == 1)
		bits[nb++] = 0;

	/* Pad at front (prepend 0-bits) to align to byte boundary */
	size_t pad = (8 - (nb % 8)) % 8;
	if (pad > 0) {
		memmove(&bits[pad], &bits[0], nb);
		memset(&bits[0], 0, pad);
		nb += pad;
	}

	/* Convert bit array to bytes (bit j -> byte bit j, LSB = bit 0) */
	size_t nbytes = nb / 8;
	for (size_t i = 0; i < nbytes; i++) {
		out[i] = 0;
		for (int j = 0; j < 8; j++)
			out[i] |= (uint8_t)(bits[i * 8 + j] << j);
	}
	return nbytes;
}

/* ---------------------------------------------------------------------- */
/* Response decoding                                                        */
/* ---------------------------------------------------------------------- */

/**
 * Locate the first payload bit of a DAP response.
 *
 * The response format is:
 *   [0...0][1][value_bits:valbytes*8][crc6:6][0...0_pad]
 *
 * Skips the leading zero padding and the framing '1' bit and returns the
 * index of the first value bit. Returns nbits when no framing bit is found,
 * which makes the extraction below yield zeros.
 */
static size_t decode_find_payload(const uint8_t *raw, size_t nbits)
{
	size_t start = 0;

	while (start < nbits && ((raw[start / 8] >> (start % 8)) & 1) == 0)
		start++;

	/* Skip the framing '1' bit */
	if (start < nbits)
		start++;

	return start;
}

/**
 * Decode a raw MPSSE-clocked-in byte buffer into a little-endian byte stream.
 *
 * The FTDI clocks bits in LSB-first, so the value bits are extracted LSB
 * first into consecutive destination bytes. Exactly @a valbytes bytes are
 * written to @a dst; bits past the end of the raw buffer decode as zero.
 *
 * Unlike a fixed-width decode this handles responses of any length, which is
 * what a block read needs.
 */
static void decode_response_buf(const uint8_t *raw, size_t rawbytes,
				uint8_t *dst, size_t valbytes)
{
	size_t nbits = rawbytes * 8;
	size_t start = decode_find_payload(raw, nbits);

	for (size_t byte = 0; byte < valbytes; byte++) {
		uint8_t b = 0;

		for (unsigned int bit = 0; bit < 8; bit++) {
			size_t idx = start + byte * 8 + bit;

			if (idx < nbits)
				b |= (uint8_t)(((raw[idx / 8] >> (idx % 8)) & 1) << bit);
		}
		dst[byte] = b;
	}
}

/**
 * Decode a raw MPSSE-clocked-in byte buffer into a scalar value.
 *
 * Returns the decoded value (up to 4 bytes).
 * If the buffer is all zeros (no response), returns 0.
 */
static uint32_t decode_response(const uint8_t *raw, size_t rawbytes,
				size_t valbytes)
{
	uint8_t buf[4] = { 0 };

	if (valbytes > sizeof(buf))
		valbytes = sizeof(buf);

	decode_response_buf(raw, rawbytes, buf, valbytes);

	return le_to_h_u32(buf);
}

/* ---------------------------------------------------------------------- */
/* Context lifecycle                                                        */
/* ---------------------------------------------------------------------- */

struct ifxdap_ctx *ifxdap_open(uint16_t vid, uint16_t pid, const char *serial,
				const char *location, int channel,
				const struct ifxdap_pins *pins)
{
	struct ifxdap_ctx *ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
		return NULL;

	uint16_t vids[2] = { vid, 0 };
	uint16_t pids[2] = { pid, 0 };

	/*
	 * serial and location are normally NULL, matching any device with the
	 * right VID:PID. They only need to be set when more than one such
	 * device is enumerated at once - e.g. two miniWigglers, or a single
	 * miniWiggler that shows up twice because a prior USB enumeration
	 * failed to configure (Windows/Linux both leave the stale, unusable
	 * device node in place instead of removing it). Since every
	 * miniWiggler V3 reports the same serial number, "adapter usb
	 * location" is the only way to disambiguate that case; "adapter
	 * serial" is provided for symmetry with the generic ftdi driver.
	 */
	ctx->mpsse = mpsse_open(vids, pids, NULL, serial, location, channel);
	if (!ctx->mpsse) {
		free(ctx);
		return NULL;
	}

	ctx->pins = *pins;
	ifxdap_set_max_busy_bits(ctx, IFXDAP_BUSY_BITS_DEFAULT);
	/* DAS's CSelDevIfTc::init() default is 400 ms @ 0x2d83f, but that value
	 * does not transfer to our driver: measured on a TC375 Lite Kit, keeping
	 * the 400 ms stall here breaks attach reliably (0/8), while 0 restores it
	 * (8/8), with an identical object checksum across three replications
	 * (ab-test/ROOT-CAUSE.md). Inferred, not measured: DAS's vendor stack
	 * services the AURIX safety watchdog during that stall; ours does not, so
	 * the wait likely lets the watchdog expire and the OCDS alarm it raises
	 * makes the subsequent IO-client handshake fail. Left at 0 until that is
	 * instrumented directly.
	 */
	ctx->post_reset_delay_ms = 0;

	/* Idle state: DAP bus released (input), normal ADBUS levels. */
	mpsse_set_data_bits_high_byte(ctx->mpsse, pins->acbus_val_input, pins->acbus_dir_input);
	mpsse_set_data_bits_low_byte(ctx->mpsse, pins->adbus_val_normal, pins->adbus_dir_normal);
	if (mpsse_flush(ctx->mpsse) != ERROR_OK) {
		mpsse_close(ctx->mpsse);
		free(ctx);
		return NULL;
	}

	return ctx;
}

void ifxdap_close(struct ifxdap_ctx *ctx)
{
	if (!ctx)
		return;
	mpsse_close(ctx->mpsse);
	free(ctx);
}

void ifxdap_set_clk_freq(struct ifxdap_ctx *ctx, uint32_t freq_hz)
{
	mpsse_set_frequency(ctx->mpsse, (int)freq_hz);
}

/* ---------------------------------------------------------------------- */
/* Response scratch-buffer bookkeeping                                      */
/* ---------------------------------------------------------------------- */

static uint8_t *ifxdap_raw_alloc(struct ifxdap_ctx *ctx, size_t nbytes)
{
	if (ctx->overflow)
		return NULL;

	if (ctx->raw_len + nbytes > sizeof(ctx->raw) || ctx->rsp_count >= IFXDAP_RSP_SLOTS_MAX) {
		ctx->overflow = true;
		return NULL;
	}

	uint8_t *p = &ctx->raw[ctx->raw_len];
	ctx->raw_len += nbytes;
	return p;
}

static void ifxdap_queue_response(struct ifxdap_ctx *ctx, uint8_t *raw, size_t nbytes,
					uint8_t *dst, size_t valbytes)
{
	if (!raw)
		return;

	struct ifxdap_rsp_slot *s = &ctx->rsp[ctx->rsp_count++];
	s->raw = raw;
	s->nbytes = nbytes;
	s->dst = dst;
	s->valbytes = valbytes;
}

int ifxdap_exec(struct ifxdap_ctx *ctx)
{
	if (ctx->overflow) {
		LOG_ERROR("ifxdap: batch buffer overflow, aborting batch");
		mpsse_discard_queued_commands(ctx->mpsse);
		ctx->raw_len = 0;
		ctx->rsp_count = 0;
		ctx->overflow = false;
		return -1;
	}

	if (mpsse_flush(ctx->mpsse) != ERROR_OK) {
		ctx->raw_len = 0;
		ctx->rsp_count = 0;
		return -1;
	}

	/* Dispatch responses to their destination slots. mpsse_flush() above
	 * already failed the whole batch on a short read, so every raw
	 * scratch buffer here is known to hold real clocked-in data. */
	for (size_t i = 0; i < ctx->rsp_count; i++) {
		struct ifxdap_rsp_slot *s = &ctx->rsp[i];

		if (!s->dst)
			continue;

		if (s->valbytes <= sizeof(uint32_t)) {
			/*
			 * Scalar slot: the destination is a host
			 * uint32_t/uint16_t/uint8_t, so store in host byte
			 * order.
			 */
			uint32_t val = decode_response(s->raw, s->nbytes, s->valbytes);
			memcpy(s->dst, &val, s->valbytes);
		} else {
			/*
			 * Block slot: the destination is a raw memory
			 * buffer, so store the decoded little-endian byte
			 * stream directly. Decoding into a scalar here would
			 * overread the temporary.
			 */
			decode_response_buf(s->raw, s->nbytes, s->dst, s->valbytes);
		}
	}

	ctx->raw_len = 0;
	ctx->rsp_count = 0;
	return 0;
}

/* ---------------------------------------------------------------------- */
/* GPIO sequences                                                           */
/* ---------------------------------------------------------------------- */

void ifxdap_test_reset(struct ifxdap_ctx *ctx)
{
	mpsse_set_data_bits_high_byte(ctx->mpsse, ctx->pins.acbus_val_trst, ctx->pins.acbus_dir_trst);
	mpsse_set_data_bits_low_byte(ctx->mpsse, ctx->pins.adbus_val_trst, ctx->pins.adbus_dir_trst);
	mpsse_set_data_bits_low_byte(ctx->mpsse, ctx->pins.adbus_val_normal, ctx->pins.adbus_dir_normal);
}

void ifxdap_reset(struct ifxdap_ctx *ctx)
{
	mpsse_set_data_bits_high_byte(ctx->mpsse, ctx->pins.acbus_val_reset, ctx->pins.acbus_dir_reset);
	mpsse_set_data_bits_low_byte(ctx->mpsse, ctx->pins.adbus_val_normal, ctx->pins.adbus_dir_normal);
}

/*
 * ifxdap_release_reset - best-effort return to the idle pin state.
 *
 * ifxdap_connect() and ifxdap_reset_and_run() assert TRST and nSRST as their
 * first action. If a USB transfer fails between that assertion and the
 * explicit release, the target would be left held in reset for the rest of
 * the session with no further command coming its way. Every error return
 * from those two functions therefore routes through here, which restores the
 * idle DAP bus state and tries to flush it.
 *
 * Deliberately fire-and-forget: it runs on an error path where the transport
 * is already known to be unreliable, so it must not be able to change the
 * caller's exit status.
 */
static void ifxdap_release_reset(struct ifxdap_ctx *ctx)
{
	if (!ctx)
		return;

	mpsse_set_data_bits_high_byte(ctx->mpsse, ctx->pins.acbus_val_input, ctx->pins.acbus_dir_input);
	mpsse_set_data_bits_low_byte(ctx->mpsse, ctx->pins.adbus_val_trst, ctx->pins.adbus_dir_trst);
	mpsse_set_data_bits_low_byte(ctx->mpsse, ctx->pins.adbus_val_normal, ctx->pins.adbus_dir_normal);
	mpsse_flush(ctx->mpsse);

	/* Drop the response bookkeeping too: the pins were re-driven underneath
	 * it, so anything queued against the old pin state is meaningless. */
	ctx->raw_len = 0;
	ctx->rsp_count = 0;
	ctx->overflow = false;
}

/* ---------------------------------------------------------------------- */
/* DAP telegram transmission                                                */
/* ---------------------------------------------------------------------- */

void ifxdap_telegram(struct ifxdap_ctx *ctx, uint8_t cmdid, uint8_t arglen,
				uint64_t arg, size_t rsplen,
				uint8_t *dst, size_t valbytes)
{
	uint8_t enc[16];
	size_t enc_len = encode_telegram(cmdid, arglen, arg, enc);
	if (enc_len == 0)
		return;

	/* Drive the DAP bus towards the target before clocking the telegram
	 * out, then release it so the target can drive the response back. */
	mpsse_set_data_bits_high_byte(ctx->mpsse, ctx->pins.acbus_val_output, ctx->pins.acbus_dir_output);
	mpsse_clock_data_out(ctx->mpsse, enc, 0, enc_len * 8, NEG_EDGE_OUT | LSB_FIRST);
	mpsse_set_data_bits_high_byte(ctx->mpsse, ctx->pins.acbus_val_input, ctx->pins.acbus_dir_input);

	if (rsplen > 0) {
		uint8_t *raw = ifxdap_raw_alloc(ctx, rsplen);

		if (raw)
			mpsse_clock_data_in(ctx->mpsse, raw, 0, rsplen * 8, POS_EDGE_IN | LSB_FIRST);
		ifxdap_queue_response(ctx, raw, rsplen, dst, valbytes);
	}
}

/* ---------------------------------------------------------------------- */
/* High-level DAP operations                                                */
/* ---------------------------------------------------------------------- */

void ifxdap_set_io_client(struct ifxdap_ctx *ctx, uint8_t client_id)
{
	/* dap_telegram(28, 3, client_id, 3) - response 3 bytes discarded */
	ifxdap_telegram(ctx, IFXDAP_CMD_SET_IO_CLIENT, 3, client_id, 3, NULL, 0);
}

void ifxdap_write_ojconf(struct ifxdap_ctx *ctx, uint16_t ojconf)
{
	/* dap_writereg(0xE, ojconf, 16): cmdid=8, arglen=20, arg=(ojconf<<4)|0xE */
	uint64_t arg = ((uint64_t)ojconf << 4) | DAP_REG_OJCONF;
	ifxdap_telegram(ctx, IFXDAP_CMD_WRITEREG, 20, arg, 7, NULL, 0);
}

void ifxdap_writereg_0(struct ifxdap_ctx *ctx, uint16_t val)
{
	/* dap_writereg(0, val, 12): cmdid=8, arglen=16, arg=(val<<4)|0 */
	uint64_t arg = (uint64_t)val << 4;
	ifxdap_telegram(ctx, IFXDAP_CMD_WRITEREG, 16, arg, 7, NULL, 0);
}

void ifxdap_select_addr(struct ifxdap_ctx *ctx, uint32_t addr)
{
	/* dap_writereg(1, addr, 32): cmdid=8, arglen=36, arg=(addr<<4)|1 */
	uint64_t arg = ((uint64_t)addr << 4) | DAP_REG_IOADDR;
	ifxdap_telegram(ctx, IFXDAP_CMD_WRITEREG, 36, arg, 7, NULL, 0);
}

void ifxdap_write32(struct ifxdap_ctx *ctx, uint32_t val)
{
	/* dap_writereg(4, val, 32): cmdid=8, arglen=36, arg=(val<<4)|4 */
	uint64_t arg = ((uint64_t)val << 4) | 4;
	ifxdap_telegram(ctx, IFXDAP_CMD_WRITEREG, 36, arg, 7, NULL, 0);
}

void ifxdap_write16(struct ifxdap_ctx *ctx, uint16_t val)
{
	/* dap_writereg(6, val, 16): cmdid=8, arglen=20, arg=(val<<4)|6 */
	uint64_t arg = ((uint64_t)val << 4) | 6;
	ifxdap_telegram(ctx, IFXDAP_CMD_WRITEREG, 20, arg, 7, NULL, 0);
}

void ifxdap_write8(struct ifxdap_ctx *ctx, uint8_t val)
{
	/* dap_writereg(8, val, 8): cmdid=8, arglen=12, arg=(val<<4)|8 */
	uint64_t arg = ((uint64_t)val << 4) | 8;
	ifxdap_telegram(ctx, IFXDAP_CMD_WRITEREG, 12, arg, 7, NULL, 0);
}

void ifxdap_read32(struct ifxdap_ctx *ctx, uint32_t *dst)
{
	/* dap_readreg(5, 4): cmdid=26, arglen=7, arg=(5|0x50)=0x55, rsp=9 bytes */
	ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7, DAP_REG_READ32 | DAP_SIZE4,
			9, (uint8_t *)dst, 4);
}

void ifxdap_read16(struct ifxdap_ctx *ctx, uint16_t *dst)
{
	/* dap_readreg(7, 2): cmdid=26, arglen=7, arg=(7|0x40)=0x47, rsp=7 bytes */
	ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7, DAP_REG_READ16 | DAP_SIZE2,
			7, (uint8_t *)dst, 2);
}

void ifxdap_read8(struct ifxdap_ctx *ctx, uint8_t *dst)
{
	/* dap_readreg(9, 4): cmdid=26, arglen=7, arg=(9|0x50)=0x59, rsp=9 bytes */
	/* Note: 8-bit reads return a 32-bit value with the byte replicated.
	 * Keeping valbytes=1 makes the response dispatcher store just the low
	 * byte directly into the caller's buffer once the batch is executed. */
	ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7, DAP_REG_READ8 | DAP_SIZE4,
			9, dst, 1);
}

void ifxdap_write_comdata(struct ifxdap_ctx *ctx, uint32_t val)
{
	/* Keep the named COMDATA helper even though the current wire encoding is
	 * identical to a normal 32-bit write: vendor tooling uses register 4 as a
	 * distinct protocol surface for the TC4x key exchange. */
	uint64_t arg = ((uint64_t)val << 4) | IFXDAP_REG_COMDATA;
	ifxdap_telegram(ctx, IFXDAP_CMD_WRITEREG, 36, arg, 7, NULL, 0);
}

void ifxdap_dapisc(struct ifxdap_ctx *ctx, uint8_t sz, uint64_t data)
{
	/* dap_telegram(17, sz, data, 5): DAPISC opcode */
	ifxdap_telegram(ctx, IFXDAP_CMD_DAPISC, sz, data, 5, NULL, 0);
}

/*
 * ifxdap_dap_sync - DAP opcode 16 "sync".
 *
 * DAS sends this as the first telegram of every DAPISC handshake, with a
 * zero-length argument and no response:
 *
 *   2ccb9: mov  rdi,QWORD PTR [rbx+0x78]
 *   2ccbd: xor  r8d,r8d            ; arg = 0
 *   2ccc0: xor  ecx,ecx            ; arglen = 0 bits
 *   2ccc2: mov  edx,0x20           ; response field = 32
 *   2ccc7: mov  esi,0x10           ; cmd = 16 = "sync"
 *   2cccf: call QWORD PTR [rax+0x118]
 *
 * (which cs_add_tgr() parameter is the argument width and which the
 * response width is argued in evidence/notes/46_csfAddDapCmd_arglen.txt;
 * either reading makes the encoded telegram for (16, 0-bit, 0) the same
 * three bytes this produces).
 *
 * The opcode name comes from das_dap_cmd_ifx_name[16] == "sync" @ 0x812a0.
 */
void ifxdap_dap_sync(struct ifxdap_ctx *ctx)
{
	ifxdap_telegram(ctx, IFXDAP_CMD_DAP_SYNC, 0, 0, 0, NULL, 0);
}

/*
 * ifxdap_idle_clocks - generate n bare DAP clock cycles.
 *
 * DAS clocks 64 idle cycles right before the DAPISC pair on the AURIX
 * connect and reset paths:
 *
 *   mCbsConnectAndConfig() @ 0x30380-0x3038c
 *     30380: mov  rdi,QWORD PTR [rbx+0x78]
 *     30384: mov  esi,0x40              ; 64 cycles
 *     3038c: mov  rdx,QWORD PTR [rax+0xb8]   ; cs_add_clock_cycles
 *     303a2: call rdx
 *
 * The bus must be released while this runs: no data is driven and nothing
 * is clocked back in, so the DAP sees only a train of clock edges.
 */
void ifxdap_idle_clocks(struct ifxdap_ctx *ctx, unsigned int n)
{
	static const uint8_t zeros[16] = { 0 };

	if (n == 0)
		return;

	/* Release the bus first so the clock edges are seen as idle. */
	mpsse_set_data_bits_high_byte(ctx->mpsse, ctx->pins.acbus_val_input, ctx->pins.acbus_dir_input);

	/* MPSSE clocks at most 8 bits per byte written; emit whole bytes and
	 * then a remainder.  n is small (<= a few hundred) at every DAS call
	 * site. */
	while (n >= 8) {
		mpsse_clock_data_out(ctx->mpsse, zeros, 0, 8, NEG_EDGE_OUT | LSB_FIRST);
		n -= 8;
	}
	if (n)
		mpsse_clock_data_out(ctx->mpsse, zeros, 0, n, NEG_EDGE_OUT | LSB_FIRST);
}

/*
 * ifxdap_set_max_busy_bits - set the CBS wait budget behind the 48-bit
 * DAPISC selector.
 *
 * DAS keeps the budget in CSelDevIfTc+0x74 and recomputes the selector in
 * CSelDevIfTc+0x68 as ((bits + 3) >> 3) << 8:
 *
 *   mCbsSetMaxBusyBits() @ 0x2f3d0-0x2f3ec   clamp to [12, 248]
 *   mCbsSetMaxBusyBits() @ 0x2f413-0x2f41f   sel = ((bits + 3) >> 3) << 8
 *   init()             @ 0x2d7d3,0x2d80f-0x2d818  default bits = 34, sel = 0x0400
 *
 * and the 48-bit DAPISC argument is then 0x4ABBAF530000 | sel:
 *
 *   mSetDapisc() [clone .part.0] @ 0x2ccfb-0x2cd1f
 *     2ccfb: movzx r8d,WORD PTR [rbx+0x68]
 *     2cd00: movabs rdx,0x4abbaf530000
 *     2cd17: or   r8,rdx
 *
 * With the default 34 that is exactly the 0x4ABBAF530400 this driver used
 * to hardcode, so this refactor is behaviour preserving by construction.
 */
void ifxdap_set_max_busy_bits(struct ifxdap_ctx *ctx, unsigned int bits)
{
	if (bits < IFXDAP_BUSY_BITS_MIN)
		bits = IFXDAP_BUSY_BITS_MIN;
	if (bits > IFXDAP_BUSY_BITS_MAX)
		bits = IFXDAP_BUSY_BITS_MAX;

	ctx->busy_bits = bits;
	/* ((bits + 3) >> 3) << 8, the low 16 bits of the DAPISC argument. */
	ctx->dapisc_sel = (uint16_t)(((bits + 3) >> 3) << 8);
}

/*
 * ifxdap_set_post_reset_delay - record the post-reset settle time.
 *
 * DAS makes the post-reset delay an explicit, device-configurable quantity
 * and applies it on every reset path, rather than folding it into a fixed
 * sleep.  CSelDevIfTc+0x64 holds the delay in milliseconds and is set per
 * device family in CSelDevIfTc::init():
 *
 *   2d83f: mov DWORD PTR [rbx+0x64],0x190   ; 400 ms, the default
 *   2d8a6: mov DWORD PTR [rbx+0x64],0xc8     ; 200 ms
 *   2d8e1: mov DWORD PTR [rbx+0x64],0x14     ;  20 ms
 *   2d95b: mov DWORD PTR [rbx+0x64],0x12c    ; 300 ms
 *   2d974: mov DWORD PTR [rbx+0x64],0x64     ; 100 ms
 *   2daf1: mov DWORD PTR [rbx+0x64],0x28     ;  40 ms
 *   2db4f: mov DWORD PTR [rbx+0x64],0xaf     ; 175 ms
 *   2db74: mov DWORD PTR [rbx+0x64],0x78     ; 120 ms
 *   2dc90: mov DWORD PTR [rbx+0x64],0x12c    ; 300 ms
 *
 * and it is applied as a nanosleep in
 * mResetDeviceAurixDap_PostResetDelay() @ 0x2f5cd-0x2f62a.  This driver
 * instead hardcoded usleep(100000) (100 ms), which is neither DAS's default
 * nor derived from anything device specific.
 */
void ifxdap_set_post_reset_delay(struct ifxdap_ctx *ctx, unsigned int ms)
{
	ctx->post_reset_delay_ms = ms;
}

/* ---------------------------------------------------------------------- */
/* Full DAP connect sequence                                                */
/* ---------------------------------------------------------------------- */

/*
 * ifxdap_clear_error - clear the sticky OCMTS error latch without resetting.
 *
 * An aborted SRI transaction (most commonly reading erased PFLASH, which
 * raises an uncorrectable ECC error) sets IOINFO bit 2. While that bit is set
 * the OCMTS answers *every* subsequent read with 0x00000000, which historically
 * led to the false conclusion that PFLASH0 had a hardware read lockout.
 *
 * Re-running the DAPISC handshake and reprogramming the IO client clears the
 * latch. Unlike ifxdap_connect() this performs no TRST/SRST pulse and no clock
 * change, so target state and the current halt/run mode are preserved.
 *
 * Returns 0 if the interface is healthy afterwards, -1 otherwise.
 */
/* The 48-bit DAPISC key exchange argument prefix: 0x4ABBAF53_0000.
 * OR'd with the max-busy-bits selector; see
 * ifxdap_set_max_busy_bits(). */
#define IFXDAP_DAPISC48_MAGIC  0x4ABBAf530000ULL

int ifxdap_clear_error(struct ifxdap_ctx *ctx)
{
	uint16_t ioinfo = 0;

	ifxdap_idle_clocks(ctx, 64);
	ifxdap_dap_sync(ctx);
	ifxdap_dapisc(ctx, 16, 0xF00);
	ifxdap_telegram(ctx, IFXDAP_CMD_DAPISC, 48,
			IFXDAP_DAPISC48_MAGIC | ctx->dapisc_sel, 5, NULL, 0);
	ifxdap_set_io_client(ctx, 1);
	ifxdap_write_ojconf(ctx, 0x4501);
	ifxdap_writereg_0(ctx, 0xC1);
	if (ifxdap_exec(ctx) < 0)
		return -1;

	return ifxdap_read_ioinfo(ctx, &ioinfo) == 0 && !(ioinfo & IFXDAP_IOINFO_ERROR) ? 0 : -1;
}

/*
 * ifxdap_read_ioinfo - read the DAP IOINFO status register (register 0xB).
 */
int ifxdap_read_ioinfo(struct ifxdap_ctx *ctx, uint16_t *out)
{
	uint16_t ioinfo = 0;

	ifxdap_set_io_client(ctx, 1);
	ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7, DAP_REG_IOINFO | DAP_SIZE2, 7,
					(uint8_t *)&ioinfo, 2);
	if (ifxdap_exec(ctx) < 0)
		return -1;

	*out = ioinfo;
	return 0;
}

#define IFXDAP_RESET_PULSE_MS 20

static int ifxdap_connect_seq(struct ifxdap_ctx *ctx, uint32_t slow_hz, uint32_t fast_hz, bool with_reset);

int ifxdap_connect(struct ifxdap_ctx *ctx, uint32_t slow_hz, uint32_t fast_hz)
{
	/*
	 * AURIX_DAP_ATTACH set: connect without the hardware reset. The DAP
	 * handshake works on a chip whose debug interface is already up (any
	 * earlier session since power-on), and the application keeps running;
	 * with the reset every start of OpenOCD rebooted the board. When the
	 * handshake fails, start again without the variable.
	 */
	const char *attach = getenv("AURIX_DAP_ATTACH");

	return ifxdap_connect_seq(ctx, slow_hz, fast_hz, !attach || !*attach);
}

int ifxdap_connect_reset(struct ifxdap_ctx *ctx, uint32_t slow_hz, uint32_t fast_hz)
{
	return ifxdap_connect_seq(ctx, slow_hz, fast_hz, true);
}

static int ifxdap_connect_seq(struct ifxdap_ctx *ctx, uint32_t slow_hz, uint32_t fast_hz, bool with_reset)
{
	/* Force plain 2-phase MPSSE clocking. A prior tool session (e.g.
	 * Infineon's own DAS/TAS tooling) may have left the FTDI chip in
	 * 3-phase mode; the single-wire DAP protocol assumes 2-phase. */
	mpsse_three_phase_clocking_config(ctx->mpsse, false);

	/* Step 1: test reset + slow clock + hardware reset */
	ifxdap_test_reset(ctx);
	ifxdap_set_clk_freq(ctx, slow_hz);
	if (with_reset)
		ifxdap_reset(ctx);
	if (ifxdap_exec(ctx) < 0) {
		ifxdap_release_reset(ctx);
		return -1;
	}
	/*
	 * Hold the reset. Released in the very next USB transaction (about a
	 * millisecond later) the reset was not dependable: measured on a TC4D7,
	 * the halt after reset request below was missed in one reset out of three
	 * or four and the application ran. With 20 ms: 24 clean halts out of 24.
	 */
	if (with_reset)
		usleep(IFXDAP_RESET_PULSE_MS * 1000);

	/*
	 * Release reset as its own isolated USB transaction, then give the
	 * chip's OCDS/DAP interface time to come up before starting the
	 * DAPISC handshake below. Releasing reset only as an implicit side
	 * effect of the first handshake telegram's own ACBUS output-mode
	 * write -- i.e. in the very same batch, microseconds after release --
	 * is too fast: the chip is not yet ready and the handshake comes back
	 * all-zero as if nothing were connected.
	 */
	mpsse_set_data_bits_high_byte(ctx->mpsse, ctx->pins.acbus_val_input, ctx->pins.acbus_dir_input);
	if (ifxdap_exec(ctx) < 0) {
		ifxdap_release_reset(ctx);
		return -1;
	}
	usleep((useconds_t)ctx->post_reset_delay_ms * 1000);

	/*
	 * Step 2: DAPISC handshake.
	 *
	 * DAS's sequence is four parts, all queued as one batch
	 * (mSetDapisc() [clone .part.0] @ 0x2ccb1-0x2cd1f in
	 * /opt/Tools/DAS/8.3.0/bin/tas_server, and
	 * mCbsConnectAndConfig() @ 0x303a4-0x303ee which omits only the
	 * "sync"):
	 *
	 *   cs_add_clock_cycles(0x40)   64 idle DAP clocks
	 *   cmd 16 "sync"   arglen 0   arg 0
	 *   cmd 17 "dapisc" arglen 16  arg 0x00000f00
	 *   cmd 17 "dapisc" arglen 48  arg 0x4abba530000 | maxwait selector
	 *
	 * The low 16 bits of the 48-bit argument are the "max busy bits"
	 * selector; with DAS's default budget of 34 that is 0x0400, i.e. the
	 * literal 0x4ABBAF530400 this driver used to hardcode.  The response
	 * of the 48-bit DAPISC is expected to be 0x0400.
	 */
	uint16_t keyex_rsp = 0;
	ifxdap_idle_clocks(ctx, 64);
	ifxdap_dap_sync(ctx);
	ifxdap_dapisc(ctx, 16, 0xF00);
	ifxdap_telegram(ctx, IFXDAP_CMD_DAPISC, 48,
			IFXDAP_DAPISC48_MAGIC | ctx->dapisc_sel, 5,
			(uint8_t *)&keyex_rsp, 2);
	ifxdap_set_io_client(ctx, 1);

	/* Read IOINFO to check state (register 0xB, 2 bytes) */
	uint16_t ioinfo = 0;
	ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7,
			DAP_REG_IOINFO | DAP_SIZE2, 7,
			(uint8_t *)&ioinfo, 2);

	ifxdap_write_ojconf(ctx, 0x503);
	if (ifxdap_exec(ctx) < 0) {
		ifxdap_release_reset(ctx);
		return -1;
	}

	if (keyex_rsp != 0x0400)
		LOG_WARNING("ifxdap: DAPISC key-exchange handshake returned 0x%04x (expected 0x0400); "
			"debug interface may not be responding", keyex_rsp);

	/*
	 * Allow debug state to settle.
	 *
	 * The handshake sometimes leaves the IO client with its error flag set.
	 * That state is sticky: the client then answers every subsequent read
	 * with zero instead of reporting a transport failure, so OCMTS sees an
	 * all-zero OSTATE and reports "Failed to enable debug".  Stop as soon as
	 * IOINFO comes back clean rather than burning a fixed number of reads
	 * and ignoring what they returned.
	 */
	bool healthy = false;
	for (int i = 0; i < 10 && !healthy; i++) {
		ioinfo = 0;
		ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7,
				DAP_REG_IOINFO | DAP_SIZE2, 7,
				(uint8_t *)&ioinfo, 2);
		if (ifxdap_exec(ctx) < 0) {
			ifxdap_release_reset(ctx);
		return -1;
		}
		healthy = !(ioinfo & IFXDAP_IOINFO_ERROR);
	}

	if (!healthy) {
		LOG_DEBUG("ifxdap: IO client error after handshake (IOINFO 0x%04x), clearing",
			  ioinfo);
		if (ifxdap_clear_error(ctx) < 0) {
			LOG_DEBUG("ifxdap: IO client stuck in error state");
			ifxdap_release_reset(ctx);
		return -1;
		}
	}

	/*
	 * Step 3: Configure IO client for full access. Bit 1 - the halt after
	 * reset request, it reads back as CBS_OSTATE.HARR - stays set: step 2 set
	 * it (0x503), and clearing it here (0x4501) took the request away again
	 * before the boot firmware had looked at it. The firmware arms a trigger
	 * on the start address and the core halts there before its first
	 * instruction; the target layer withdraws the request afterwards
	 * (tricore_init_debug_access()) and resumes the core for "reset run".
	 */
	ifxdap_set_io_client(ctx, 1);
	ifxdap_write_ojconf(ctx, 0x4503);
	ifxdap_writereg_0(ctx, 0xC1);
	if (ifxdap_exec(ctx) < 0) {
		ifxdap_release_reset(ctx);
		return -1;
	}

	/* Step 4: Switch to fast clock */
	ifxdap_set_clk_freq(ctx, fast_hz);
	return ifxdap_exec(ctx);
}

/*
 * ifxdap_reset_and_run - hardware reset with CPU released to run freely.
 *
 * Same as ifxdap_connect but ends with OJCONF=0x0501 (bit 14 clear) instead
 * of 0x4501. Bit 14 (HARR) is the "halt-after-reset" flag; clearing it lets
 * the CPU execute firmware immediately after reset without debugger halt.
 */
int ifxdap_reset_and_run(struct ifxdap_ctx *ctx, uint32_t slow_hz, uint32_t fast_hz)
{
	/* Step 1: hardware reset pulse + slow clock */
	ifxdap_set_clk_freq(ctx, slow_hz);
	ifxdap_reset(ctx);
	if (ifxdap_exec(ctx) < 0) {
		ifxdap_release_reset(ctx);
		return -1;
	}

	/* Explicit reset release as its own isolated USB transaction, followed
	 * by a settle delay.  See ifxdap_connect() for the full rationale: the
	 * chip's OCDS/DAP interface needs real time after reset release before
	 * it can process the DAPISC handshake; without this delay the
	 * handshake intermittently returns a bogus all-zero response. */
	mpsse_set_data_bits_high_byte(ctx->mpsse, ctx->pins.acbus_val_input, ctx->pins.acbus_dir_input);
	if (ifxdap_exec(ctx) < 0) {
		ifxdap_release_reset(ctx);
		return -1;
	}
	usleep((useconds_t)ctx->post_reset_delay_ms * 1000);

	/* Step 2: DAPISC handshake - re-establish DAP after chip reset.
	 * Same four parts as ifxdap_connect(): 64 idle clocks, "sync", the
	 * 16-bit DAPISC 0x0F00, and the 48-bit DAPISC
	 * 0x4ABBAF53_0000 | maxwait selector. */
	ifxdap_idle_clocks(ctx, 64);
	ifxdap_dap_sync(ctx);
	ifxdap_dapisc(ctx, 16, 0xF00);
	ifxdap_telegram(ctx, IFXDAP_CMD_DAPISC, 48,
			IFXDAP_DAPISC48_MAGIC | ctx->dapisc_sel, 5, NULL, 0);
	ifxdap_set_io_client(ctx, 1);

	uint16_t ioinfo = 0;
	ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7,
			DAP_REG_IOINFO | DAP_SIZE2, 7,
			(uint8_t *)&ioinfo, 2);

	ifxdap_write_ojconf(ctx, 0x503);
	if (ifxdap_exec(ctx) < 0) {
		ifxdap_release_reset(ctx);
		return -1;
	}

	/*
	 * Allow debug state to settle.  Identical to ifxdap_connect(): the
	 * handshake can leave the IO client's sticky error flag set, after
	 * which every read answers zero and OCMTS reports "Failed to enable
	 * debug".  Stop as soon as IOINFO is clean instead of discarding the
	 * results of a fixed number of reads.
	 */
	bool healthy = false;
	for (int i = 0; i < 10 && !healthy; i++) {
		ioinfo = 0;
		ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7,
				DAP_REG_IOINFO | DAP_SIZE2, 7,
				(uint8_t *)&ioinfo, 2);
		if (ifxdap_exec(ctx) < 0) {
			ifxdap_release_reset(ctx);
		return -1;
		}
		healthy = !(ioinfo & IFXDAP_IOINFO_ERROR);
	}

	if (!healthy) {
		LOG_DEBUG("ifxdap: IO client error after reset handshake (IOINFO 0x%04x), clearing",
			  ioinfo);
		if (ifxdap_clear_error(ctx) < 0) {
			LOG_DEBUG("ifxdap: IO client stuck in error state");
			ifxdap_release_reset(ctx);
		return -1;
		}
	}

	/* Step 3: OJCONF=0x0501 - debug enabled, NO halt-after-reset (bit 14=0) */
	ifxdap_set_io_client(ctx, 1);
	ifxdap_write_ojconf(ctx, 0x0501);
	if (ifxdap_exec(ctx) < 0) {
		ifxdap_release_reset(ctx);
		return -1;
	}

	/* Step 4: fast clock */
	ifxdap_set_clk_freq(ctx, fast_hz);
	return ifxdap_exec(ctx);
}

/*
 * ifxdap_tc4x_key_exchange - TC4x COMDATA key-exchange handshake.
 *
 * TC4x devices do not expose their OCMTS debug registers (every read comes
 * back zero, see ocmts_init()'s TC4x address branch) until a word-at-a-time
 * COMDATA handshake completes on top of the DAPISC handshake that
 * ifxdap_connect() already performs. This is a distinct interface -- DAS's
 * CSelDevIfTc::mTc4xTmKeyExchange(unsigned int), 0x2fd10-0x30340 in
 * /opt/Tools/DAS/8.3.0/bin/tas_server -- not the DAPISC key exchange and not
 * the separate OCMTS-local 0xA1/0x5E unlock pattern ocmts_init() already
 * writes.
 *
 * Traced structure (see tc4-work-2026/KEYEXCHANGE.md for the full
 * derivation) for an n_words-word key:
 *
 *   Phase A: write key[0] to COMDATA (register 4), one batch
 *            (0x2fd79-0x2fd9a).
 *   Phase B: poll IOINFO + read register 5 up to 100 times, until the
 *            readback equals key[0] (0x2fdea-0x2fe9c).
 *   Phase C: write key[1..n_words-1] to COMDATA, one batch -- true only for
 *            n_words <= 16 (our 256-bit/8-word default key, 0x30057-
 *            0x300fc). Above 16 words the vendor batches pushes in groups
 *            of 16 with a "retry minus 16" backoff (0x2ff00-0x2ffb3),
 *            which is not reproduced here since it is out of scope for
 *            this key size.
 *   Phase D: for each of key[1..n_words-1], poll IOINFO + read register 5,
 *            up to 5 retries per word (0x3026d-0x30304). The >16-word path
 *            instead uses a 16-outer x 5-inner = 80 retry budget
 *            (0x30180-0x30235); that figure does not apply to this key
 *            size.
 *
 * Both the vendor's "set IO path" call (mJdaCsAddSetIopath(), 0x3007c) and
 * the 5 extra IOINFO/COMDATA padding accesses bracketing every Phase C/D
 * word are omitted: neither has a disassembled body anywhere in this
 * project (KEYEXCHANGE.md items 6 and 10), so there is nothing sourced to
 * reproduce. @iopath is accepted and logged for future use but not
 * otherwise acted on. Likewise, no IOADDR (register 1) select precedes the
 * COMDATA writes/reads below -- the vendor binary never issues one either
 * (confirmed at every one of the addresses cited above), so none is added
 * here.
 *
 * The vendor binary's own pass/fail check only inspects the *last* word's
 * verification outcome (its "ok" flag is reset to 0 at the top of every
 * Phase D outer iteration, 0x302ab, and only its final value is tested at
 * 0x3030f-0x3016f) -- an earlier word can silently fail to verify without
 * failing the overall call. This implementation deliberately requires
 * every word to verify instead, logging each failure individually: a
 * from-scratch reimplementation should not mask a partial handshake
 * failure for the sake of bug-for-bug fidelity to what looks like an
 * incidental artifact of the vendor's source.
 */
int ifxdap_tc4x_key_exchange(struct ifxdap_ctx *ctx, unsigned int iopath,
			const uint32_t *key, size_t n_words)
{
	if (n_words == 0) {
		LOG_ERROR("ifxdap: tc4x key exchange: zero-length key");
		return -1;
	}

	LOG_DEBUG("ifxdap: tc4x key exchange: starting (%zu words, iopath=%u)",
		  n_words, iopath);

	/* Phase A: push key[0] (0x2fd79-0x2fd9a). */
	ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7, DAP_REG_IOINFO | DAP_SIZE2,
			7, NULL, 2);
	ifxdap_write_comdata(ctx, key[0]);
	if (ifxdap_exec(ctx) < 0) {
		LOG_ERROR("ifxdap: tc4x key exchange: phase A transport error");
		return -1;
	}

	/* Phase B: poll for key[0] to be echoed back, up to 100 tries
	 * (0x2fdf4-0x2fe9c). */
	bool word0_ok = false;
	for (int i = 0; i < 100 && !word0_ok; i++) {
		uint16_t ioinfo = 0;
		uint32_t got = 0xDEADBEEF;

		ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7,
				DAP_REG_IOINFO | DAP_SIZE2, 7,
				(uint8_t *)&ioinfo, 2);
		ifxdap_read32(ctx, &got);
		if (ifxdap_exec(ctx) == 0)
			word0_ok = (got == key[0]);
	}
	if (!word0_ok) {
		LOG_WARNING("ifxdap: tc4x key exchange: key[0] was not acknowledged "
			"after 100 tries (vendor error 0xE010)");
		return -1;
	}

	if (n_words == 1) {
		LOG_DEBUG("ifxdap: tc4x key exchange: done (1-word key)");
		return 0;
	}

	/* Phase C: push key[1..n_words-1] as one batch -- valid for
	 * n_words <= 16, see the function comment (0x30065-0x300fc). */
	for (size_t i = 1; i < n_words; i++) {
		ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7,
				DAP_REG_IOINFO | DAP_SIZE2, 7, NULL, 2);
		ifxdap_write_comdata(ctx, key[i]);
	}
	if (ifxdap_exec(ctx) < 0) {
		LOG_ERROR("ifxdap: tc4x key exchange: phase C transport error");
		return -1;
	}

	/* Phase D: verify key[1..n_words-1], up to 5 retries per word
	 * (0x3026d-0x30304). Unlike the vendor binary (see function
	 * comment), every word must verify for this call to succeed. */
	bool all_ok = true;
	for (size_t i = 1; i < n_words; i++) {
		bool word_ok = false;

		for (int attempt = 0; attempt < 5 && !word_ok; attempt++) {
			uint16_t ioinfo = 0;
			uint32_t got = 0xDEADBEEF;

			ifxdap_telegram(ctx, IFXDAP_CMD_READREG, 7,
					DAP_REG_IOINFO | DAP_SIZE2, 7,
					(uint8_t *)&ioinfo, 2);
			ifxdap_read32(ctx, &got);
			if (ifxdap_exec(ctx) == 0)
				word_ok = (got == key[i]);
		}
		if (!word_ok) {
			LOG_WARNING("ifxdap: tc4x key exchange: key[%zu] did not verify "
				"after 5 tries (vendor error 0xE010)", i);
			all_ok = false;
		}
	}

	if (!all_ok)
		return -1;

	LOG_DEBUG("ifxdap: tc4x key exchange: completed OK (%zu words)", n_words);
	return 0;
}
