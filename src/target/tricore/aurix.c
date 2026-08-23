// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Infineon AURIX SoC family parameters
 *
 * The TriCore CPU driver in tricore.c is generation agnostic; what actually
 * differs between AURIX TC2xx, TC3xx and TC4xx is gathered here.
 *
 * Copyright (C) 2026 Rafael J. Cruz <rafa.junio@hardenedlinux.org>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>

#include <helper/log.h>

#include "aurix.h"

/** OCMTS register pair used by AURIX TC2xx and TC3xx. */
#define AURIX_OCMTS_OEC_TC2XX_TC3XX	0xF0000478
#define AURIX_OCMTS_OSTATE_TC2XX_TC3XX	0xF0000480

/** OCMTS register pair used by AURIX TC4xx. */
#define AURIX_OCMTS_OEC_TC4XX		0xFA180068
#define AURIX_OCMTS_OSTATE_TC4XX	0xFA18006C

const char *aurix_family_name(enum aurix_family family)
{
	switch (family) {
	case AURIX_FAMILY_TC2XX:
		return "tc2xx";
	case AURIX_FAMILY_TC3XX:
		return "tc3xx";
	case AURIX_FAMILY_TC4XX:
		return "tc4xx";
	case AURIX_FAMILY_UNKNOWN:
		break;
	}
	return "unknown";
}

void aurix_ocmts_registers(enum aurix_family family, uint32_t *oec_addr,
			   uint32_t *ostate_addr)
{
	if (family == AURIX_FAMILY_TC4XX) {
		*oec_addr = AURIX_OCMTS_OEC_TC4XX;
		*ostate_addr = AURIX_OCMTS_OSTATE_TC4XX;
		return;
	}

	/* TC2xx and TC3xx share one pair of addresses, and an unrecognised
	 * device type is given the same one rather than being rejected: the
	 * register addresses are only a starting point for the readiness
	 * handshake in ocmts_init(), which reports a failure of its own. */
	*oec_addr = AURIX_OCMTS_OEC_TC2XX_TC3XX;
	*ostate_addr = AURIX_OCMTS_OSTATE_TC2XX_TC3XX;
}
