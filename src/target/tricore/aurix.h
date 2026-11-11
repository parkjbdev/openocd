/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2026 Rafael J. Cruz <rafa.junio@hardenedlinux.org>      *
 *                                                                         *
 *   Infineon AURIX SoC family identification                               *
 *                                                                         *
 *   "TriCore" names the CPU core, which is what this driver implements,    *
 *   while "AURIX" names the microcontroller family built around it.  The   *
 *   family is split in three generations -- TC2xx, TC3xx and TC4xx -- and  *
 *   those are where the debug resources actually differ, so the family-    *
 *   specific knowledge lives here rather than in the CPU driver.          *
 *                                                                         *
 *   AURIX devices are identified by the IEEE 1149.1 device ID read from    *
 *   the JTAG/DAP TAP. The upper nibble carries the silicon version and is  *
 *   masked out before comparing against a device type.                     *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_TRICORE_AURIX_H
#define OPENOCD_TARGET_TRICORE_AURIX_H

#include <stdbool.h>
#include <stdint.h>

#include <jtag/jtag.h>

/**
 * Infineon AURIX product generations.
 *
 * The naming matches the generation names the target configurations pass to
 * 'ocmts create' ('tc2xx', 'tc3xx', 'tc4xx').
 */
enum aurix_family {
	AURIX_FAMILY_UNKNOWN = 0,
	AURIX_FAMILY_TC2XX,
	AURIX_FAMILY_TC3XX,
	AURIX_FAMILY_TC4XX,
};

/** JTAG ID with the silicon version nibble masked out. */
#define AURIX_DT_VERSION_MASK_OUT	0x0fffffff

/** AURIX TC2x device types (version nibble already masked out). */
#define AURIX_DT_TC27X			0x001da083

/**
 * AURIX TC3x device types (version nibble already masked out).
 *
 * TC38x and TC39x currently share one ID: tc38x.cfg and tc39x.cfg both
 * declare 0x20205083 as their OCMTS TAP ID, so the two cannot be told apart
 * from the ID alone. They are the same for this driver's purposes anyway.
 */
#define AURIX_DT_TC36X			0x00209083
#define AURIX_DT_TC37X			0x00207083
#define AURIX_DT_TC37X_EXT		0x00208083
#define AURIX_DT_TC38X			0x00205083

/** AURIX TC4x device types (version nibble already masked out). */
#define AURIX_DT_TC41X			0x00218083
#define AURIX_DT_TC42X			0x00219083
#define AURIX_DT_TC44X			0x0021a083
#define AURIX_DT_TC45X			0x0021b083
#define AURIX_DT_TC46X			0x0021c083
#define AURIX_DT_TC48X			0x0021d083
#define AURIX_DT_TC49AA			0x0021e083	/* TC49xA */
#define AURIX_DT_TC49X			0x0022b083	/* TC49xN */
#define AURIX_DT_TC4DX			0x00225083
#define AURIX_DT_TC4RX			0x00223083

/**
 * Device type actually known for a TAP.
 *
 * On a real JTAG chain, tap->idcode is filled in by jtag_examine_chain() and is
 * the authoritative device identity. The ifxdap transport never shifts an
 * IDCODE/BYPASS scan -- its transport .init hook is a no-op
 * (src/target/tricore/ifxdap.c) and 'init' never issues 'jtag init'
 * (src/openocd.c) -- so tap->idcode stays at its zero-initialised default and
 * testing it directly makes every TC4x branch permanently unreachable.
 *
 * The device type configured on the TAP with 'newtap ... -expected-id' is the
 * only identity this transport actually knows, and tc3x.cfg/tc4dx.cfg both set
 * it deliberately, so falling back to it is using declared configuration rather
 * than guessing. A real idcode, when one exists, always wins.
 *
 * @param tap the TAP to identify; may be NULL
 * @return the IDCODE to test with aurix_family_of(), or 0 if unknown
 */
static inline uint32_t aurix_tap_device_type(const struct jtag_tap *tap)
{
	if (!tap)
		return 0;
	if (tap->idcode)
		return tap->idcode;
	if (tap->expected_ids_cnt > 0)
		return tap->expected_ids[0];
	return 0;
}

/**
 * AURIX generation a device ID belongs to.
 *
 * @param device_type IEEE 1149.1 device ID read from the TAP
 * @return the generation, or AURIX_FAMILY_UNKNOWN if the ID is not a known
 *         AURIX device type
 */
static inline enum aurix_family aurix_family_of(uint32_t device_type)
{
	switch (device_type & AURIX_DT_VERSION_MASK_OUT) {
	case AURIX_DT_TC27X:
		return AURIX_FAMILY_TC2XX;
	case AURIX_DT_TC36X:
	case AURIX_DT_TC37X:
	case AURIX_DT_TC37X_EXT:
	case AURIX_DT_TC38X:
		return AURIX_FAMILY_TC3XX;
	case AURIX_DT_TC41X:
	case AURIX_DT_TC42X:
	case AURIX_DT_TC44X:
	case AURIX_DT_TC45X:
	case AURIX_DT_TC46X:
	case AURIX_DT_TC48X:
	case AURIX_DT_TC49AA:
	case AURIX_DT_TC49X:
	case AURIX_DT_TC4DX:
	case AURIX_DT_TC4RX:
		return AURIX_FAMILY_TC4XX;
	default:
		return AURIX_FAMILY_UNKNOWN;
	}
}

/**
 * Check whether a device ID belongs to the AURIX TC4x family.
 *
 * TC4x differs from TC3x in the layout of the OCMTS access path and in the
 * flash controller programming sequence, so several code paths need to
 * distinguish the two.
 *
 * @param device_type IEEE 1149.1 device ID read from the TAP.
 * @return true if @a device_type is a TC4x device.
 */
static inline bool aurix_device_is_tc4x(uint32_t device_type)
{
	return aurix_family_of(device_type) == AURIX_FAMILY_TC4XX;
}

/**
 * User Configuration Block windows of every AURIX generation: boot mode
 * headers, debug and flash passwords, protection and their CONFIRMED state.
 * A bad write there cannot be undone and can lock a device for good, so this
 * build never erases, programs or plainly writes them, whatever a
 * configuration declares. The windows are wider than the UCBs themselves so
 * that one table covers every derivative.
 *
 * Checks whether [@a addr, @a addr + @a len) touches a UCB window, in either
 * the cached (0x8) or non-cached (0xA) segment.
 * @return the window's name, or NULL if the range is clear of every UCB
 */
static inline const char *aurix_range_ucb(uint64_t addr, uint64_t len)
{
	static const struct {
		uint32_t start, end;	/* [start, end) */
		const char *name;
	} windows[] = {
		{ 0xAE400000, 0xAE500000, "TC4x UCB0" },
		{ 0xAEC00000, 0xAED00000, "TC4x UCB1" },
		{ 0xAF400000, 0xAF500000, "TC3x UCB" },
		{ 0xAF100000, 0xAF110000, "TC2x UCB" },
	};

	if (len == 0)
		return NULL;
	uint64_t last = addr + len - 1;
	/* A range that wraps or leaves the 32-bit space is never legitimate. */
	if (last < addr || last > 0xFFFFFFFFull)
		return "an out-of-range address";
	for (size_t i = 0; i < sizeof(windows) / sizeof(windows[0]); i++) {
		static const uint64_t segments[] = { 0x80000000, 0xA0000000 };

		for (size_t k = 0; k < 2; k++) {
			uint64_t s = (windows[i].start & 0x0FFFFFFF) | segments[k];
			uint64_t e = (windows[i].end & 0x0FFFFFFF) | segments[k];
			if (addr < e && last >= s)
				return windows[i].name;
		}
	}
	return NULL;
}

/** @returns the name of @a family, as used by 'ocmts create' ('tc2xx', ...). */
const char *aurix_family_name(enum aurix_family family);

/**
 * OCMTS control registers of an AURIX generation.
 *
 * The OCMTS register pair the debug interface answers on lives at a
 * different address on TC4x than it does on TC2xx/TC3xx. Unknown device
 * types get the TC2xx/TC3xx addresses, which is what every AURIX part
 * predating TC4x uses.
 *
 * @param family the AURIX generation
 * @param oec_addr receives the OEC register address
 * @param ostate_addr receives the OSTATE register address
 */
void aurix_ocmts_registers(enum aurix_family family, uint32_t *oec_addr,
			   uint32_t *ostate_addr);

#endif /* OPENOCD_TARGET_TRICORE_AURIX_H */
