// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Blinky for the Infineon AURIX TC275 Lite Kit, used to check that
 * OpenOCD can flash the board and that the flashed image really runs
 * from a cold boot.
 *
 * CPU0 alternates LED1 (P00.5) and LED2 (P00.6) at about 1 Hz. Both
 * LEDs are low active, confirmed against the official TC275 Lite Kit
 * manual: same port/pin assignment as TC375 (identical IOCR4/OMR
 * layout for P00, confirmed by register read-back on real TC275
 * hardware). The other two cores are not started by the SSW, but the
 * guard below keeps the image correct if they ever are.
 *
 * Unlike TC3x, no watchdog needs disabling first: a stage 1 probe with
 * no watchdog code at all (startup-stage1.S / main-stage1.c in this
 * same directory) ran a free-running counter loop for 300ms+ with no
 * reset on real TC275 hardware, confirming this family's Lite Kit does
 * not arm a watchdog that would otherwise kill an unserviced
 * application -- same finding as TC4D7. See startup.S's comment for
 * the full reasoning, and tc27x.ld's comment for why _START is linked
 * at 0xa0000020 instead of 0xa0000000 (TC2x BootROM's "Internal Start"
 * fallback mode, confirmed against the official TC21x/22x/23x manual).
 */

#include <stdint.h>

/* Port 00 - LED1 on P00.5, LED2 on P00.6 */
#define P00_BASE		0xf003a000u
#define P00_OMR			(*(volatile uint32_t *)(P00_BASE + 0x04u))
#define P00_IOCR4		(*(volatile uint32_t *)(P00_BASE + 0x14u))

/* IOCRx field: 0x10 = push-pull general purpose output */
#define IOCR_OUTPUT_PUSH_PULL	0x10u
#define IOCR_FIELD_MASK		0x1fu
#define IOCR4_P5_SHIFT		11u
#define IOCR4_P6_SHIFT		19u

/* OMR bits [15:0] set a pin high, bits [31:16] clear it low */
#define OMR_SET(pin)		(1u << (pin))
#define OMR_CLR(pin)		(1u << ((pin) + 16u))

#define CPU_ID_CSFR		0xfe1cu
#define CPU_ID_CORE_MASK	0x00000007u

/*
 * Not yet measured on TC275 silicon; reuse TC375's calibration (3M
 * iterations ~= 125ms at that core's clock) as a starting point for a
 * visibly-blinking LED. Adjust once the actual period is observed.
 */
#define DELAY_COUNT		12000000ul

static inline uint32_t core_id(void)
{
	uint32_t id;

	__asm__ volatile ("mfcr %0, %1" : "=d" (id) : "i" (CPU_ID_CSFR) : "memory");
	return id & CPU_ID_CORE_MASK;
}

static void delay(void)
{
	volatile uint32_t i = DELAY_COUNT;

	while (i--)
		;
}

int main(void)
{
	/* only CPU0 drives the LEDs, so the cores cannot fight over P00 */
	while (core_id() != 0u)
		;

	uint32_t iocr4 = P00_IOCR4;

	iocr4 &= ~(IOCR_FIELD_MASK << IOCR4_P5_SHIFT);
	iocr4 &= ~(IOCR_FIELD_MASK << IOCR4_P6_SHIFT);
	iocr4 |= IOCR_OUTPUT_PUSH_PULL << IOCR4_P5_SHIFT;
	iocr4 |= IOCR_OUTPUT_PUSH_PULL << IOCR4_P6_SHIFT;
	P00_IOCR4 = iocr4;

	for (;;) {
		/* LED1 on, LED2 off */
		P00_OMR = OMR_CLR(5) | OMR_SET(6);
		delay();
		/* LED1 off, LED2 on */
		P00_OMR = OMR_SET(5) | OMR_CLR(6);
		delay();
	}

	return 0;
}
