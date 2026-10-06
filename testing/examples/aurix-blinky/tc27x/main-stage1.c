// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Stage 1 bring-up probe for the Infineon AURIX TC275 Lite Kit.
 *
 * No GPIO, no watchdog-disable: this just free-runs an incrementing
 * counter into a fixed, known DSPR address so OpenOCD can sample it
 * (halt, read, resume, wait, halt, read again) to tell, with zero risk
 * to the board, whether the counter keeps advancing (no watchdog is
 * resetting the core) or keeps snapping back near zero (something is
 * resetting it, most likely an armed watchdog).
 */

#include <stdint.h>

#define COUNTER_ADDR	0x70001000u
#define COUNTER		(*(volatile uint32_t *)COUNTER_ADDR)

int main(void)
{
	COUNTER = 0;

	for (;;)
		COUNTER++;

	return 0;
}
