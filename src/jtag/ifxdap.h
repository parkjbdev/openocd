/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   IFX DAP public interface                                              *
 *   Copyright (c) 2026 Infineon Technologies AG                           *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_IFXDAP_H
#define OPENOCD_JTAG_IFXDAP_H

#include <jtag/interface.h>

struct ifxdap_driver {
	int (*init)(void);
};

#endif	/* OPENOCD_JTAG_IFXDAP_H */
