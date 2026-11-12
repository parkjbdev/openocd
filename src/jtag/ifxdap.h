/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * DAP-over-MPSSE protocol library
 *
 * Protocol reverse-engineered by Enrico Pozzobon (epozzobon/tricore-things,
 * HackInBo 2025). Implements the DAP telegram framing, CRC6 and the OCDS
 * register-level handshake on top of the generic FTDI MPSSE helpers in
 * jtag/drivers/mpsse.[ch], so it works with any adapter built around an
 * FTDI MPSSE channel, not just the Infineon miniWiggler.
 *
 * This layer only knows the DAP wire protocol. It has no notion of the
 * OCMTS memory transaction semantics built on top of it (see
 * target/tricore/ocmts.h) and no notion of a specific adapter's USB
 * descriptor; both of those are supplied by the adapter driver.
 *
 * Copyright (C) 2026 Rafael J. Cruz <rafa.junio@hardenedlinux.org>
 */

#ifndef OPENOCD_JTAG_IFXDAP_H
#define OPENOCD_JTAG_IFXDAP_H

#include <jtag/interface.h>

struct ifxdap_driver {
	int (*init)(void);
};

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

struct ifxdap_ctx;

/*
 * GPIO states needed to drive the DAP bus, expressed as MPSSE "Set Data
 * Bits Low/High Byte" (value, direction) pairs. The adapter driver fills
 * these in from its own schematic; the library only ever asserts one of
 * these four fixed states, it never computes pin numbers itself.
 */
struct ifxdap_pins {
	/* ADBUS (low byte): idle/normal state */
	uint8_t adbus_val_normal;
	uint8_t adbus_dir_normal;
	/* ADBUS: test-reset (TRST) assert state */
	uint8_t adbus_val_trst;
	uint8_t adbus_dir_trst;

	/* ACBUS (high byte): DAP bus driven towards the target (telegram out) */
	uint8_t acbus_val_output;
	uint8_t acbus_dir_output;
	/* ACBUS: DAP bus released to receive (telegram response in) */
	uint8_t acbus_val_input;
	uint8_t acbus_dir_input;
	/* ACBUS: TRST asserted */
	uint8_t acbus_val_trst;
	uint8_t acbus_dir_trst;
	/* ACBUS: nSRST asserted */
	uint8_t acbus_val_reset;
	uint8_t acbus_dir_reset;
};

/**
 * Open the MPSSE channel and set the DAP bus to its idle state.
 *
 * @param serial Match only a device with this USB serial string, or NULL
 * for any. Every miniWiggler V3 reports the same serial number, so this
 * cannot disambiguate two of them; it is provided for symmetry with the
 * generic ftdi driver's "adapter serial" command.
 * @param location Match only a device at this USB bus/port path (e.g.
 * "1-2"), or NULL for any. This is the only way to select a specific
 * device when more than one matching VID:PID is enumerated at once.
 * @param vid USB vendor ID to open
 * @param pid USB product ID to open
 * @param channel MPSSE channel to use, 0-based (channel A on an FT2232H)
 * @param pins GPIO pin assignment of this adapter; must not be NULL, the
 * assignment is schematic specific and there is no default
 */
struct ifxdap_ctx *ifxdap_open(uint16_t vid, uint16_t pid, const char *serial,
				const char *location, int channel,
				const struct ifxdap_pins *pins);

/** Close the MPSSE channel and free the context. */
void ifxdap_close(struct ifxdap_ctx *ctx);

/** Queue an MPSSE clock divisor change for the given frequency in Hz. */
void ifxdap_set_clk_freq(struct ifxdap_ctx *ctx, uint32_t freq_hz);

/**
 * Flush all queued MPSSE commands, read back any pending telegram
 * responses and decode them into the destinations given to
 * ifxdap_telegram(). Returns 0 on success, -1 on error; on error no
 * response destination is written; a short read is a hard error, not a
 * partial one.
 */
int ifxdap_exec(struct ifxdap_ctx *ctx);

/** Assert test-reset (TRST GPIO sequence). */
void ifxdap_test_reset(struct ifxdap_ctx *ctx);

/** Assert hardware reset (nSRST GPIO sequence). */
void ifxdap_reset(struct ifxdap_ctx *ctx);

/**
 * Encode a DAP telegram and queue it for clocking out, queuing the
 * response (if any) for decoding by the next ifxdap_exec().
 *
 * @param ctx     DAP context
 * @param cmdid   5-bit command ID (IFXDAP_CMD_*)
 * @param arglen  number of argument bits (0-63)
 * @param arg     argument value (arglen bits, LSB first)
 * @param rsplen  number of bytes to clock in as response (0 = no response)
 * @param dst     destination buffer for decoded response (NULL = discard)
 * @param valbytes number of significant value bytes in response
 */
void ifxdap_telegram(struct ifxdap_ctx *ctx, uint8_t cmdid, uint8_t arglen,
		uint64_t arg, size_t rsplen, uint8_t *dst, size_t valbytes);

/* High-level DAP register operations */

/** Set IO client (must precede every memory access). */
void ifxdap_set_io_client(struct ifxdap_ctx *ctx, uint8_t client_id);

/** Set OJCONF debug configuration register. */
void ifxdap_write_ojconf(struct ifxdap_ctx *ctx, uint16_t ojconf);

/** Write to DAP register 0 (misc control / halt-resume). */
void ifxdap_writereg_0(struct ifxdap_ctx *ctx, uint16_t val);

/** Select target memory address (IOADDR). */
void ifxdap_select_addr(struct ifxdap_ctx *ctx, uint32_t addr);

/** Write a 32-bit word to the currently-selected address. */
void ifxdap_write32(struct ifxdap_ctx *ctx, uint32_t val);

/** Write a 16-bit half-word to the currently-selected address. */
void ifxdap_write16(struct ifxdap_ctx *ctx, uint16_t val);

/** Write an 8-bit byte to the currently-selected address. */
void ifxdap_write8(struct ifxdap_ctx *ctx, uint8_t val);

/** Queue a 32-bit read from the currently-selected address, store to *dst. */
void ifxdap_read32(struct ifxdap_ctx *ctx, uint32_t *dst);

/** Queue a 16-bit read from the currently-selected address, store to *dst. */
void ifxdap_read16(struct ifxdap_ctx *ctx, uint16_t *dst);

/** Queue an 8-bit read from the currently-selected address, store to *dst. */
void ifxdap_read8(struct ifxdap_ctx *ctx, uint8_t *dst);

/** Write COMDATA register (key exchange / password slot). */
void ifxdap_write_comdata(struct ifxdap_ctx *ctx, uint32_t val);

/** DAPISC handshake telegram (used for ISC unlock). */
void ifxdap_dapisc(struct ifxdap_ctx *ctx, uint8_t sz, uint64_t data);

/**
 * DAP "sync" telegram (DAP opcode 16, zero argument bits, no response).
 *
 * DAS emits this as the first telegram of every DAPISC handshake:
 * CSelDevIfTc::mSetDapisc() [clone .part.0] @ 0x2ccb9-0x2ccf1 in
 * /opt/Tools/DAS/8.3.0/bin/tas_server issues
 * cs_add_tgr(cmd=0x10, 0x20, 0x00, 0) before the two cmd=0x11 telegrams.
 * The opcode name "sync" comes from the table das_dap_cmd_ifx_name
 * @ 0x812a0, index 16.  The purpose is to hand the DAP state machine a
 * short, self-delimiting, CRC-checked bitstream to resynchronise on
 * before the extended DAPISC exchange starts.
 */
void ifxdap_dap_sync(struct ifxdap_ctx *ctx);

/**
 * Generate @a n idle DAP clock cycles with the DAP bus released: no
 * telegram, no response, just clock edges.
 *
 * DAS clocks 64 idle cycles immediately before the DAPISC pair on the
 * AURIX connect and reset paths: CSelDevIfTc::mCbsConnectAndConfig()
 * @ 0x30384 calls cs_add_clock_cycles(0x40), and
 * mResetDeviceAurixDap_PostResetDelay() @ 0x2f696 calls
 * cs_add_clock_cycles(this->+0x64) followed by cs_add_clock_cycles(4)
 * @ 0x2f6a0.
 */
void ifxdap_idle_clocks(struct ifxdap_ctx *ctx, unsigned int n);

/**
 * Set the "CBS max busy bits" budget that determines the low 16 bits of
 * the 48-bit DAPISC argument.
 *
 * DAS keeps the budget in CSelDevIfTc+0x74 (initialised to 34 by
 * CSelDevIfTc::init() @ 0x2d7d3) and derives the DAPISC selector in
 * CSelDevIfTc+0x68 as ((bits + 3) >> 3) << 8
 * (CSelDevIfTc::init() @ 0x2d80f-0x2d818, and
 * CSelDevIfTc::mCbsSetMaxBusyBits() @ 0x2f3d0, which also clamps the
 * budget to [12, 248]).  The 48-bit DAPISC argument is then
 * 0x4ABBAF530000 | selector (mSetDapisc() @ 0x2ccfb-0x2cd1f).
 *
 * The default of 34 yields selector 0x0400, i.e. the literal
 * 0x4ABBAF530400 this driver hardcodes.  Making the budget explicit
 * lets it be adjusted without editing a magic constant; as in DAS, a
 * change only takes effect on the next DAPISC handshake.
 */
void ifxdap_set_max_busy_bits(struct ifxdap_ctx *ctx, unsigned int bits);

/**
 * Set the post-reset settle time, in milliseconds.
 *
 * DAS makes this an explicit, per-device-family quantity rather than a
 * fixed constant.  It is stored in CSelDevIfTc+0x64, initialised to 400 in
 * CSelDevIfTc::init() @ 0x2d83f, and re-assigned for each supported family
 * (20, 40, 100, 120, 175, 200, 300 ms; see
 * evidence/notes/50_post_reset_delay_table.txt).  It is applied as a
 * nanosleep() on every reset path in
 * CSelDevIfTc::mResetDeviceAurixDap_PostResetDelay() @ 0x2f5cd-0x2f62a:
 *
 *   2f5df: mul  rdx                    ; rdx = 0x431bde82d7b634db
 *   2f5e2: shr  rdx,0x12               ; rdx = ms / 1000  -> tv_sec
 *   2f5ed: sub  rsi,rax                ; rsi = ms % 1000
 *   2f5f0: imul rsi,rsi,0x3e8          ; rsi = ns
 *   2f620: call 7790 <nanosleep@@plt>
 *
 * This driver previously hardcoded usleep(100000), i.e. 100 ms, which is
 * neither DAS's default nor derived from anything device specific.
 */
void ifxdap_set_post_reset_delay(struct ifxdap_ctx *ctx, unsigned int ms);

/** Read the DAP IOINFO status register (bit IFXDAP_IOINFO_ERROR = sticky
 *  transaction-error latch). */
int ifxdap_read_ioinfo(struct ifxdap_ctx *ctx, uint16_t *out);

/** Clear the sticky OCMTS transaction-error latch without resetting. */
int ifxdap_clear_error(struct ifxdap_ctx *ctx);

/**
 * Full DAP initialization + OCDS unlock sequence:
 *   test_reset -> slow clock -> DAPISC -> set_io_client -> write_ojconf ->
 *   fast clock.
 *
 * @param ctx     DAP context
 * @param slow_hz initial clock for handshake (e.g. 720000 = 720 kHz)
 * @param fast_hz operating clock after unlock (e.g. 5000000 = 5 MHz)
 * Returns 0 on success, -1 on error.
 */
int ifxdap_connect(struct ifxdap_ctx *ctx, uint32_t slow_hz, uint32_t fast_hz);

/** ifxdap_connect() that always resets the chip (AURIX_DAP_ATTACH is ignored). */
int ifxdap_connect_reset(struct ifxdap_ctx *ctx, uint32_t slow_hz, uint32_t fast_hz);

/**
 * Hardware reset with the CPU released to run freely (OJCONF HARR bit
 * clear), instead of the halt-on-reset performed by ifxdap_connect().
 */
int ifxdap_reset_and_run(struct ifxdap_ctx *ctx, uint32_t slow_hz, uint32_t fast_hz);

/**
 * TC4x COMDATA key-exchange handshake (CSelDevIfTc::mTc4xTmKeyExchange).
 *
 * TC4x devices keep their OCMTS debug registers unresponsive (every read
 * comes back zero) until this additional handshake completes on top of the
 * DAPISC handshake ifxdap_connect() already performs. See the
 * implementation in ifxdap.c for the full traced algorithm and its sources.
 *
 * @param ctx      DAP context
 * @param iopath   IO path selector (accepted for interface completeness;
 *                 not currently applied -- see implementation comment)
 * @param key      key words, LSW first
 * @param n_words  number of 32-bit words in @p key (must be >= 1)
 * @return 0 on success, -1 on failure
 */
int ifxdap_tc4x_key_exchange(struct ifxdap_ctx *ctx, unsigned int iopath,
			const uint32_t *key, size_t n_words);

/** CRC6 computation over an LSB-first bit array (exported for testing). */
uint8_t ifxdap_crc6(const uint8_t *bits, size_t nbit);

/* 5-bit DAP command IDs used directly by callers building their own
 * telegrams (e.g. target/tricore/ocmts.c block transfer setup). */
#define IFXDAP_CMD_WRITEREG      8
#define IFXDAP_CMD_WRITEMEM      9
#define IFXDAP_CMD_READMEM       10
#define IFXDAP_CMD_DAPISC        17
#define IFXDAP_CMD_READREG       26
#define IFXDAP_CMD_SET_IO_CLIENT 28

/* DAP opcode table as recovered from das_dap_cmd_ifx_name @ 0x812a0 in
 * /opt/Tools/DAS/8.3.0/bin/tas_server (32 pointers into .rodata).  Index ==
 * 5-bit DAP command ID.  Opcodes DAS never uses on the AURIX path are left
 * unnamed; the names below are the ones this driver or the recovered DAS
 * code paths reference. */
#define IFXDAP_CMD_JTAG_SWAP_DR  2   /* jtag_swapDR                       */
#define IFXDAP_CMD_SET_MAXWAIT   7   /* set_maxwait                       */
#define IFXDAP_CMD_CLIENT_BLOCKW 9   /* client_blockwrite (= WRITEMEM)    */
#define IFXDAP_CMD_CLIENT_BLOCKR 10  /* client_blockread  (= READMEM)     */
#define IFXDAP_CMD_POLL          14  /* poll                              */
#define IFXDAP_CMD_DAP_SYNC      16  /* sync                              */
#define IFXDAP_CMD_JTAG_SET_IR   19  /* jtag_setIR                        */

/* DAP register (CBS "io") map as recovered from das_cbs_io_name
 * @ 0x81220 in the same binary.  Index == the 4-bit register field of a
 * client_read/client_write telegram argument. */
#define IFXDAP_REG_IOCONF        0x0 /* IO_CONFIG      (12-bit value)     */
#define IFXDAP_REG_IOADDR        0x1 /* IO_SET_ADDRESS(32-bit value)     */
#define IFXDAP_REG_WBLOCK        0x2 /* IO_WRITE_BLOCK (32-bit value)     */
#define IFXDAP_REG_RBLOCK        0x3 /* IO_READ_BLOCK  (32-bit value)     */
#define IFXDAP_REG_COMDATA       0x4 /* IO_WRITE_WORD  (32-bit value)     */
#define IFXDAP_REG_RWORD         0x5 /* IO_READ_WORD   (32-bit value)     */
#define IFXDAP_REG_WHWORD        0x6 /* IO_WRITE_HWORD (32-bit value)     */
#define IFXDAP_REG_RHWORD        0x7 /* IO_READ_HWORD  (32-bit value)     */
#define IFXDAP_REG_WBYTE         0x8 /* IO_WRITE_BYTE  (32-bit value)     */
#define IFXDAP_REG_RBYTE         0x9 /* IO_READ_BYTE   (32-bit value)     */
#define IFXDAP_REG_IOINFO        0xB /* IO_SUPERVISOR  (16-bit value)     */
#define IFXDAP_REG_OJCONF        0xE /* IO_SET_OJCONF  (16-bit value)     */
#define IFXDAP_REG_CLIENT_ID     0xF /* IO_CLIENT_ID   (16-bit value)     */

/* Value width, in bits, of the DAP register field for each CBS register.
 * Recovered from CSWTCH.163 @ 0x6bd00 in tas_server, indexed by register
 * number; CSWTCH.160 @ 0x6bd40 is the read-side equivalent.  Only the
 * entries the AURIX paths actually use are reproduced here. */
#define IFXDAP_NBITS_IOCONF      12
#define IFXDAP_NBITS_IOADDR      32
#define IFXDAP_NBITS_WBLOCK      32
#define IFXDAP_NBITS_COMDATA     32
#define IFXDAP_NBITS_IOINFO      16
#define IFXDAP_NBITS_OJCONF      16

/* Default "CBS max busy bits" budget.  DAS initialises CSelDevIfTc+0x74 to
 * 34 in CSelDevIfTc::init() @ 0x2d7d3, from which it derives the DAPISC
 * selector ((34 + 3) >> 3) << 8 == 0x0400. */
#define IFXDAP_BUSY_BITS_DEFAULT 34
/* Range DAS clamps the budget to, mCbsSetMaxBusyBits() @ 0x2f3d0. */
#define IFXDAP_BUSY_BITS_MIN     12
#define IFXDAP_BUSY_BITS_MAX     248

/* Default post-reset settle time in milliseconds.  DAS initialises
 * CSelDevIfTc+0x64 to 400 in CSelDevIfTc::init() @ 0x2d83f and then varies
 * it per device family (20, 40, 100, 120, 175, 200, 300 ms -- see
 * evidence/notes/50_post_reset_delay_table.txt).  It is applied as a
 * nanosleep() in mResetDeviceAurixDap_PostResetDelay() @ 0x2f5cd-0x2f62a. */
#define IFXDAP_POST_RESET_DELAY_DEFAULT 400

/* IOINFO bit 2 is a sticky error latch. While set, every OCMTS read
 * returns zero. Set by aborted SRI transactions, e.g. reading erased
 * PFLASH. */
#define IFXDAP_IOINFO_ERROR 0x0004

#endif	/* OPENOCD_JTAG_IFXDAP_H */
