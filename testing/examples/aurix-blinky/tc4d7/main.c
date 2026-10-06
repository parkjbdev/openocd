// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Blinky for the Infineon AURIX TC4D7 Lite Kit, used to check that
 * OpenOCD can flash the board and that the flashed image really runs
 * from a cold boot with no debugger attached.
 *
 * CPU0 alternates LED1 (P03.9) and LED2 (P03.10) at about 1 Hz. Both
 * LEDs are low active (Table 3, infineon-aurix-tc4d7-lite-kit-userguide).
 * The other cores (CPU1-CPU5, CPUCS) are not started by the SSW, but the
 * guard below keeps the image correct if they ever are.
 *
 * Unlike TC3x, no watchdog needs disabling first: a stage 1 probe with
 * no watchdog code at all (startup-stage1.S / main-stage1.c in this same
 * directory) ran a free-running counter loop for 500ms+ with no reset on
 * real TC4D7 hardware, confirming this family's Lite Kit does not arm a
 * watchdog that would otherwise kill an unserviced application. See
 * startup.S's comment for the full reasoning.
 *
 * TC4x's port peripheral is a redesign from TC3x's: pin direction/mode
 * now lives in a per-pin PADCFGn_DRVCFG register instead of a shared
 * IOCRn register, but the OUT/OMR scheme for driving an already-configured
 * GPIO pin is unchanged (same bit layout, confirmed against
 * tc4-work-2026/re/IfxPort_regdef.h's Ifx_P_OMR_Bits).
 */

#include <stdint.h>

/* Port 03 - LED1 on P03.9, LED2 on P03.10 (both low active) */
#define P03_BASE		0xf003ac00u
#define P03_OMR			(*(volatile uint32_t *)(P03_BASE + 0x3cu))
#define P03_PADCFG9_DRVCFG	(*(volatile uint32_t *)0xf003af94u)
#define P03_PADCFG10_DRVCFG	(*(volatile uint32_t *)0xf003afa4u)

/*
 * PADCFGn_DRVCFG fields (Ifx_P_PADCFG_DRVCFG_Bits):
 *   DIR    [0]   pad signal direction, 1 = output
 *   OD     [1]   output buffer config, 0 = push-pull
 *   MODE   [7:4] 0 = general purpose (software driven via OUT/OMR)
 * PD/PL/MODEX are left at their reset defaults.
 */
#define DRVCFG_DIR_OUTPUT	0x00000001u
#define DRVCFG_OD_PUSH_PULL	0x00000000u
#define DRVCFG_MODE_MASK	0x000000f0u
#define DRVCFG_MODE_GPIO	0x00000000u
#define DRVCFG_DIR_OD_MASK	0x00000003u

/* OMR bits [15:0] set a pin high, bits [31:16] clear it low */
#define OMR_SET(pin)		(1u << (pin))
#define OMR_CLR(pin)		(1u << ((pin) + 16u))

#define CPU_ID_CSFR		0xfe1cu
#define CPU_ID_CORE_MASK	0x00000007u

/*
 * Not yet measured on TC4D7 silicon; reuse TC375's calibration (3M
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
	/* only CPU0 drives the LEDs, so the cores cannot fight over P03 */
	while (core_id() != 0u)
		;

	uint32_t cfg9 = P03_PADCFG9_DRVCFG;
	uint32_t cfg10 = P03_PADCFG10_DRVCFG;

	cfg9 &= ~(DRVCFG_DIR_OD_MASK | DRVCFG_MODE_MASK);
	cfg9 |= DRVCFG_DIR_OUTPUT | DRVCFG_OD_PUSH_PULL | DRVCFG_MODE_GPIO;
	P03_PADCFG9_DRVCFG = cfg9;

	cfg10 &= ~(DRVCFG_DIR_OD_MASK | DRVCFG_MODE_MASK);
	cfg10 |= DRVCFG_DIR_OUTPUT | DRVCFG_OD_PUSH_PULL | DRVCFG_MODE_GPIO;
	P03_PADCFG10_DRVCFG = cfg10;

	for (;;) {
		/* LED1 on, LED2 off */
		P03_OMR = OMR_CLR(9) | OMR_SET(10);
		delay();
		/* LED1 off, LED2 on */
		P03_OMR = OMR_SET(9) | OMR_CLR(10);
		delay();
	}

	return 0;
}
