/*
 * @brief Windowed Watchdog Timer (WWDT) with RTC calibration
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include <limits.h>
#include "sensorhub.h"
#include "kernel_timer.h"
#include "kernel_debug.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* WDT window size */
#define WDT_MAX_TICKS_RELOAD              0xFFFFFF
#define WDT_WINDOW_WARNING_SIZE           512
#define WDT_CLK_DELAY                     6

#define TICK_Q                            12			/* Scale factor */

#define WDT_UPDATE_WAIT_TIMEOUT           430			/* Worst case, WDT feed-to-reload seems to be 2WDT clocks: 2*(96M/125k)*1.4 = 2150 CPU clocks
														   Assuming 5 CPU clocks for the wait loop, WDT_UPDATE_WAIT_TIMEOUT = 430 is a safe value */

#define SLEEP_TIMER_MARGIN                10
#define WDT_IDEAL_FREQ                    125000

struct sleepEvent_t {
	void (*callback)(void);
	uint64_t wakeupTimeInTicks;
	uint8_t active;
};

volatile static uint32_t calWdtFreq;			/* Measured frequency of watchdog peripheral (WDTOSC/4) */
volatile static uint16_t tickQn = 0;			/* Scaled correction factor */

volatile static uint64_t calib_tick_count = 0;	/* High resolution internal counter @1 MHz */

volatile static struct sleepEvent_t sleepEvents[LPCSH_TIMER_ID_COUNT];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/
/* Initialize WDT and RTC module
    Notice we use RTC internal osiloscope, so that we should keep 32K_OSC on
 */

static void wdt_rtc_init(void)
{
#ifdef CFG_FW_USE_EXTERNAL_RTC_CLOCK
	Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_32K_OSC);
#else
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_32K_OSC);
#endif

	/* update thresholds */
	g_Timer.sleep_threshold_us = CFG_FW_SLEEP_THRESHOLD_US;
	g_Timer.pwrDown_threshold_us = CFG_FW_PWRDOWN_THRESHOLD_US;
	g_Timer.bgProc_threshold_us = CFG_FL_BACKGROUND_US;

	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_WDT_OSC);
	/* To Calculate WDT timer frequency by RTC 32k RC */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);

	Chip_INMUX_SetFreqMeasRefClock(FREQMSR_IRC);
	Chip_INMUX_SetFreqMeasTargClock(FREQMSR_WDOSC);

	/* Start a measurement cycle and wait for it to complete. */
	Chip_SYSCON_StartFreqMeas();
	while (!Chip_SYSCON_IsFreqMeasComplete()) {}

	calWdtFreq = Chip_SYSCON_GetCompFreqMeas(Chip_Clock_GetIntOscRate());

	/* Frequency into WDT as a fixed divider of 4 */
	calWdtFreq >>= 2;
	tickQn = ((uint32_t) (1L << TICK_Q) * 1000000L / calWdtFreq) & 0xffffffff;

	Chip_WWDT_Init(LPC_WWDT);

	Chip_WWDT_SetTimeOut(LPC_WWDT, WDT_MAX_TICKS_RELOAD);	// WWDT_TICKS_MAX
	Chip_WWDT_SetWarning(LPC_WWDT, WDT_WINDOW_WARNING_SIZE);// Should no warning happens

	/* Clear watchdog timeout interrupt */
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, (WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT));

	/* Enable the RTC oscillator, oscillator rate can be determined by
	    calling Chip_Clock_GetRTCOscRate()	*/
	Chip_Clock_EnableRTCOsc();

	Chip_RTC_Init(LPC_RTC);

	/* Enable RTC as a peripheral wakeup event */
	Chip_SYSCON_EnableWakeup(SYSCON_STARTER_RTC);
	/* RTC reset */
	Chip_RTC_Reset(LPC_RTC);

	/* Start RTC at a count of 0 when RTC is disabled. If the RTC is enabled, you
	    need to disable it before setting the initial RTC count. */
	Chip_RTC_Disable(LPC_RTC);

	Chip_RTC_SetCount(LPC_RTC, 0);
	Chip_RTC_SetAlarm(LPC_RTC, 1);

	Chip_RTC_Enable(LPC_RTC);

	/* Clear latched RTC interrupt statuses */
	Chip_RTC_ClearStatus(LPC_RTC, (RTC_CTRL_OFD | RTC_CTRL_ALARM1HZ));

	NVIC_SetPriority(RTC_IRQn, CFG_FW_RTC_IRQ_PRIORITY);

	/* Enable RTC interrupt */
	NVIC_EnableIRQ(RTC_IRQn);

	/* Enable RTC alarm interrupt */
	Chip_RTC_EnableWakeup(LPC_RTC, RTC_CTRL_ALARMDPD_EN);

	Chip_WWDT_Start(LPC_WWDT);

	/* Initialize UTICK driver */
	Chip_UTICK_Init(LPC_UTICK);

	/* Clear UTICK interrupt status */
	Chip_UTICK_ClearInterrupt(LPC_UTICK);

	/* Enable Wake up from deep sleep mode due to UTick */
	Chip_SYSCON_EnableWakeup(SYSCON_STARTER_UTICK);

	NVIC_SetPriority(UTICK_IRQn, CFG_FW_UTICK_IRQ_PRIORITY);
	/* Disable UTICK interrupt */
	NVIC_EnableIRQ(UTICK_IRQn);
}

/* Converts a 125 KHz kernel tick time to a uS count */
static uint32_t wdt_ticksToUs(uint64_t ticks)
{
	return (uint32_t) ticks * 8;
}

/* Converts a 125 KHz kernel tick time to a mS count */
static uint32_t wdt_ticksToMs(uint64_t ticks)
{
	return (uint32_t) ((ticks * 8) / 1000);
}

/* Converts a passed mS count to a 125 KHz kernel tick time */
static uint32_t wdt_msToTicks(uint32_t milliseconds)
{
	return (milliseconds * 1000) / 8;
}

/* Converts a passed mS count to a 125 KHz kernel tick time */
static uint32_t wdt_usToTicks(uint32_t microseconds)
{
	return microseconds / 8;
}

/* Get current time in 125 KHz kernel tick time units */
static uint64_t wdt_rtc_GetCurrent(void)
{
	uint32_t intStatus;
	uint64_t current;

	INT_DISABLE(intStatus, 0);
	current = calib_tick_count;
	current += ((WDT_MAX_TICKS_RELOAD - Chip_WWDT_GetCurrentCount(LPC_WWDT) + WDT_CLK_DELAY) * tickQn) >> (TICK_Q);
	INT_RESTORE(intStatus, 0);

	return current / 8;
}

/* Get current time in msec */
static uint32_t wdt_rtc_GetCurrentMsec(void)
{
	uint32_t intStatus;
	uint64_t current;

	INT_DISABLE(intStatus, 0);
	current = calib_tick_count;
	current += ((WDT_MAX_TICKS_RELOAD - Chip_WWDT_GetCurrentCount(LPC_WWDT) + WDT_CLK_DELAY) * tickQn) >> (TICK_Q);
	INT_RESTORE(intStatus, 0);

	return current / 1000;
}

/* Get current time in usec */
static uint64_t wdt_rtc_GetCurrentUsec(void)
{
	uint32_t intStatus;
	uint64_t current;

	INT_DISABLE(intStatus, 0);
	current = calib_tick_count;
	current += ((WDT_MAX_TICKS_RELOAD - Chip_WWDT_GetCurrentCount(LPC_WWDT) + WDT_CLK_DELAY) * tickQn) >> (TICK_Q);
	INT_RESTORE(intStatus, 0);

	return current;
}

static int32_t wdt_rtc_diff(uint32_t operand1, uint32_t operand2)
{
	return operand1 - operand2;
}

static uint32_t wdt_rtc_add(uint32_t operand1, uint32_t operand2)
{
	return operand1 + operand2;
}

/* Returns WDT resolution in ticks per seconds, from the outside it's a 125 KHz timer */
static uint32_t wdt_rtc_GetResolution(void)
{
	return WDT_IDEAL_FREQ;
}

/**
 * @brief	watchdog timer Interrupt Handler
 * @return	Nothing
 * @note	Handles watchdog timer feed and overflow count
 */
void RTC_IRQHandler(void)
{
	uint32_t intStatus;
	uint32_t curr_tick;
	uint32_t rtcStatus;
	volatile uint32_t i = 0;

	INT_DISABLE(intStatus, 0);

	//	DEBUG_LED_Set(LED_WDT_ISR_WRAP, 1);

	rtcStatus = Chip_RTC_GetStatus(LPC_RTC);

	if (rtcStatus & RTC_CTRL_ALARM1HZ) {
		LPC_RTC->MATCH = LPC_RTC->COUNT + 1;
		curr_tick = Chip_WWDT_GetCurrentCount(LPC_WWDT);
		Chip_WWDT_Feed(LPC_WWDT);

		/* Calculate WDT freq as decrease of WDT counter in 1s */
		calWdtFreq = WDT_MAX_TICKS_RELOAD - curr_tick + WDT_CLK_DELAY;
		/* Increase calib_tick_count with calWdtFreq * previous_correction */
		calib_tick_count += (((uint64_t) calWdtFreq * tickQn) >> (TICK_Q));
		/* Calculate new correction factors */
		tickQn = ((uint32_t) (1L << TICK_Q) * 1000000L / calWdtFreq) & 0xffffffff;

		/* Wait for feed to be done to avoid wrong tickcount when wdt_GetCurrent() is called shortly after this IRQ */
		while (Chip_WWDT_GetCurrentCount(LPC_WWDT) <= curr_tick) {
			if (i++ > WDT_UPDATE_WAIT_TIMEOUT) {
				break;	/* To avoid possible hang while halting the CPU core by the debugger */
			}
		}
	}

	Chip_RTC_ClearStatus(LPC_RTC, (rtcStatus & RTC_CTRL_ALARM1HZ));
	Chip_WWDT_ClearStatusFlag(LPC_WWDT, (WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT));

	//	DEBUG_LED_Set(LED_WDT_ISR_WRAP, 0);

	INT_RESTORE(intStatus, 0);
}

static uint8_t sleepGetTimerIdNextWakeup(void) {
	uint8_t timerId = 0;
	uint8_t timerIdNextWakeup = LPCSH_TIMER_ID_COUNT;
	uint64_t earliestTime = ULLONG_MAX;

	do {
		if (sleepEvents[timerId].active != 0) {
			if (sleepEvents[timerId].wakeupTimeInTicks < earliestTime) {
				earliestTime = sleepEvents[timerId].wakeupTimeInTicks;
				timerIdNextWakeup = timerId;
			}
		}
	} while (++timerId < LPCSH_TIMER_ID_COUNT);

	return timerIdNextWakeup;
}

/**
 * @brief	UTICK Interrupt Handler
 * @return	None
 */
void UTICK_IRQHandler(void)
{
	uint32_t intStatus;
	uint8_t timerId;
	uint64_t currTime;
	uint32_t ticks;

	//	DEBUG_LED_Set(LED_WDT_ISR_WRAP, 0);

	/* If Interrupt flag is set, toggle LED 0 */
	if (Chip_UTICK_GetStatus(LPC_UTICK) & UTICK_STATUS_INTR) {
		Chip_UTICK_ClearInterrupt(LPC_UTICK);
		/* check which sleep events have expired */
		timerId = sleepGetTimerIdNextWakeup();
		while (timerId < LPCSH_TIMER_ID_COUNT) {
			currTime = wdt_rtc_GetCurrent();
			if ((sleepEvents[timerId].wakeupTimeInTicks - SLEEP_TIMER_MARGIN) <= currTime) {
				/* sleep event expired */
				sleepEvents[timerId].active = false;
				if (sleepEvents[timerId].callback != NULL) {
					(*sleepEvents[timerId].callback)();
				}
			}
			else {
				/* next wakeup in future, reconfigure utick */
				INT_DISABLE(intStatus, 0);
				ticks = sleepEvents[timerId].wakeupTimeInTicks - currTime;
				/* Convert from calibrated 125 KHz ticks to actual utick count */
				ticks = (uint32_t) ((((uint64_t) ticks) << (3 + TICK_Q)) / tickQn);
				ticks = ticks << 2;
				/* Set the UTick for a delay in oneshot mode */
				Chip_UTICK_SetTick(LPC_UTICK, ticks, false);
				INT_RESTORE(intStatus, 0);
				break;
			}
			timerId = sleepGetTimerIdNextWakeup();
		}
	}
}

/* Starts the sleep timer for a period specified by ticks. Once started,
   enter sleep mode with a call to WFI(). The timer will be cleared and
   disabled automatically. */
static void sleepSetupTicks(uint32_t ticks, void (*callback)(void), enum LPCSH_TIMER_ID timerId)
{
	uint32_t intStatus;

	/* Disable all interrupts */
	INT_DISABLE(intStatus, 0);

	/* configure sleep event */
	sleepEvents[timerId].wakeupTimeInTicks = wdt_rtc_GetCurrent() + (ticks >> 2);
	sleepEvents[timerId].active = true;
	sleepEvents[timerId].callback = callback;

	/* if next wakeup event is this timer event, then configure utick */
	if (sleepGetTimerIdNextWakeup() == timerId) {
		/* Convert from calibrated 125 KHz ticks to actual utick count */
		ticks = (uint32_t) ((((uint64_t) ticks) << (3 + TICK_Q)) / tickQn);
		/* Set the UTick for a delay in oneshot mode */
		Chip_UTICK_SetTick(LPC_UTICK, ticks, false);
	}

	INT_RESTORE(intStatus, 0);

	//	DEBUG_LED_Set(LED_WDT_ISR_WRAP, 1);

	if (callback == NULL) {
		while (sleepEvents[timerId].active) {
			__WFI();
		}
	}
}

/* Starts the sleep timer for a period specified by uS (microSeconds).
   Once started, enter sleep mode with a call to WFI(). The timer will be
   cleared and disabled automatically. */
static void sleepSetupMicroSecs(uint32_t uS, void (*callback)(void), enum LPCSH_TIMER_ID timerId)
{
	uint32_t sleepTicks = wdt_usToTicks(uS) << 2;
	sleepSetupTicks(sleepTicks, callback, timerId);
}

/* Starts the sleep timer for a period specified by mS (milliSeconds).
   Once started, enter sleep mode with a call to WFI(). The timer will be
   cleared and disabled automatically. */
static void sleepSetupMilliSecs(uint32_t mS, void (*callback)(void), enum LPCSH_TIMER_ID timerId)
{
	uint32_t sleepTicks = wdt_msToTicks(mS) << 2;
	sleepSetupTicks(sleepTicks, callback, timerId);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* create the always on timer instance */
KernelTimer_Ctrl_t g_Timer = {
	0,
	0,
	0,
	wdt_rtc_init,
	wdt_rtc_GetCurrent,
	wdt_rtc_GetCurrentUsec,
	wdt_rtc_GetCurrentMsec,
	wdt_rtc_diff,
	wdt_rtc_add,
	wdt_rtc_GetResolution,
	sleepSetupTicks,
	sleepSetupMilliSecs,
	sleepSetupMicroSecs,
	wdt_usToTicks,
	wdt_msToTicks,
	wdt_ticksToMs,
	wdt_ticksToUs
};
