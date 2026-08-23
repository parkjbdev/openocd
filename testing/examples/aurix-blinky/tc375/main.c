// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Blinky for the Infineon AURIX TC375 Lite Kit V2, used to check that
 * OpenOCD can flash the board and that the flashed image really runs
 * from a cold boot with no debugger attached.
 *
 * CPU0 alternates LED1 (P00.5) and LED2 (P00.6) at about 1 Hz. Both
 * LEDs are low active. The other two cores are not started by the SSW,
 * but the guard below keeps the image correct if they ever are.
 *
 * The watchdogs must be disabled before the blink loop is entered. The
 * TC3xx boots with the CPU watchdog and the safety watchdog armed with
 * a window of roughly 128 ms, so an application that does not service
 * them resets the device before the first LED transition is visible.
 * This is easy to miss when developing under a debugger, because an
 * attached debugger suspends the watchdogs and the very same image then
 * runs correctly after "reset halt" followed by "resume".
 *
 * The SMU is deliberately left alone. Writing to SMU_CMD without the
 * documented key sequence raises an SMU alarm whose default reaction is
 * a system reset.
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

/*
 * Watchdog control. Each core has its own WDTCPUyCON0/CON1 pair, and
 * the safety watchdog has one more pair of its own.
 */
#define WDTCPU_BASE		0xf003624cu
#define WDTCPU_STRIDE		0x0cu
#define WDTS_BASE		0xf00362a8u

/*
 * WDTxCON1.DR ("disable request") is bit 3. Bit 0 is CLRIRF in
 * WDTSCON1 and reserved in WDTCPUyCON1, so writing bit 0 disables
 * nothing and leaves the watchdog armed.
 */
#define WDTCON1_DR		0x00000008u

/* ENDINIT password protocol, see TC3xx User Manual, "Safety Protection" */
#define ENDINIT_PW_MASK		0xffffff00u
#define ENDINIT_PW_STEP1	0xf1u
#define ENDINIT_PW_STEP2	0xf2u
#define ENDINIT_SET		0x03u

#define CPU_ID_CSFR		0xfe1cu
#define CPU_ID_CORE_MASK	0x00000007u

/*
 * Measured on silicon: 3,000,000 iterations of the loop below take
 * about 125 ms, so 12,000,000 gives about 500 ms per half period.
 */
#define DELAY_COUNT		12000000ul

static inline void dsync(void)
{
	__asm__ volatile ("dsync" ::: "memory");
}

static inline uint32_t core_id(void)
{
	uint32_t id;

	__asm__ volatile ("mfcr %0, %1" : "=d" (id) : "i" (CPU_ID_CSFR) : "memory");
	return id & CPU_ID_CORE_MASK;
}

static inline volatile uint32_t *wdt_con0(uint32_t base)
{
	return (volatile uint32_t *)(uintptr_t)base;
}

static inline volatile uint32_t *wdt_con1(uint32_t base)
{
	return (volatile uint32_t *)(uintptr_t)(base + 4u);
}

static uint32_t this_core_wdt_base(void)
{
	return WDTCPU_BASE + core_id() * WDTCPU_STRIDE;
}

static void endinit_clear(uint32_t base)
{
	volatile uint32_t *con0 = wdt_con0(base);
	uint32_t pw = *con0 & ENDINIT_PW_MASK;

	*con0 = pw | ENDINIT_PW_STEP1;
	dsync();
	*con0 = pw | ENDINIT_PW_STEP2;
	(void)*con0;
}

static void endinit_set(uint32_t base)
{
	volatile uint32_t *con0 = wdt_con0(base);
	uint32_t pw = *con0 & ENDINIT_PW_MASK;

	*con0 = pw | ENDINIT_PW_STEP1;
	dsync();
	*con0 = pw | ENDINIT_SET | ENDINIT_PW_STEP2;
	(void)*con0;
}

static void watchdog_disable(uint32_t base)
{
	endinit_clear(base);
	*wdt_con1(base) |= WDTCON1_DR;
	endinit_set(base);
}

/*
 * Called from the reset handler before .bss is cleared. The stack and
 * the context save area are already valid at that point, so a normal C
 * call is safe here.
 */
void early_safety_init(void)
{
	watchdog_disable(this_core_wdt_base());
	watchdog_disable(WDTS_BASE);
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
