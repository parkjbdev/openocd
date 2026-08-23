/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2026 Rafael J. Cruz <rafa.junio@hardenedlinux.org>      *
 *                                                                         *
 *   Infineon AURIX device identification                                  *
 *                                                                         *
 *   AURIX devices are identified by the IEEE 1149.1 device ID read from    *
 *   the JTAG/DAP TAP. The upper nibble carries the silicon version and is  *
 *   masked out before comparing against a device type.                     *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_TRICORE_AURIX_DEVICE_H
#define OPENOCD_TARGET_TRICORE_AURIX_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#include <jtag/jtag.h>

/** JTAG ID with the silicon version nibble masked out. */
#define AURIX_DT_VERSION_MASK_OUT	0x0fffffff

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
	switch (device_type & AURIX_DT_VERSION_MASK_OUT) {
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
		return true;
	default:
		return false;
	}
}

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
 * @return the IDCODE to test with aurix_device_is_tc4x(), or 0 if unknown
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

#endif /* OPENOCD_TARGET_TRICORE_AURIX_DEVICE_H */
