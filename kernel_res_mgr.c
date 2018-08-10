/*
 * @brief Kernel Resource/Power Manager Interface
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2015
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
#include "sensorhub.h"
// #include "pmu_540xx.h"
#include "kernel_res_mgr.h"
#include "kernel_timer.h"
#include "sensors.h"
#include "kernel_debug.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
ResMgr_Ctrl_T g_ResMgr;

#define PLL_MULTIPLIER (CFG_HW_PLL_MODE_FREQUENCY / CFG_HW_NORMAL_MODE_FREQUENCY)

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize resource manager */
void ResMgr_Init(void)
{
	g_ResMgr.currMode = g_ResMgr.prevMode = RESMGR_STARTUP;
	g_ResMgr.fSkipSleep = 0;

	/* disable RTC */
	Chip_Clock_DisableRTCOsc();
	/* disable BOD for time being.*/
	Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_BOD_RST | SYSCON_PDRUNCFG_PD_BOD_INTR);

	ResMgr_EnterNormalMode();
	/* set voltage levels properly for I2C slave event to wake the part */
	/* vd1     VD2          VD3    VD8        FINE LEVEL FOR 0.7 CHOICES   */
	Chip_POWER_SetLPVDLevel();

	/* Switch over to the IRC so that the PLL can be configured */
	Chip_Clock_SetSystemPLLSource(SYSCON_PLLCLKSRC_CLKIN);

	/* Power down PLL to change the PLL divider ratio */
	Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_SYS_PLL);

	/* First parameter is the multiplier, the second parameter is the input frequency in MHz */
	Chip_Clock_SetupSystemPLL(PLL_MULTIPLIER, Chip_Clock_GetMainClockRate());

	/*  FYI - VD1 -logic , VD2 - Analog, VD3 - RAM, VD8 - Core side of IO */
	/*  VD3 can be 0.7v (which is 0.8v if using P100 fine trim) in powerdown mode, without affecting IO timing, but this needs to be verified on final silicon */
	/* The setting 0.7,1.2,1.2,1.2, p100, can 'unofficially' be used for running 12MHz by setting in PDRUNCFG the PMU into low power mode as follows */
	/* LPC_SYSCON->PDRUNCFGSET = PDRUNCFG_LP_VD1 | PDRUNCFG_LP_VD2 | PDRUNCFG_VD3 | PDRUNCFG_VD8;  // this puts PMU in LP mode*/
	/* Note that you must ensure that the regulator is in non-LP mode when entering/leaving Powerdown/Deepsleep. user PDRUNCFGCLR*/
}

/* Request resource manager to put system in Power-down mode. */
void ResMgr_EnterPowerDownMode(uint32_t estSleepTimeUs)
{
	POWER_MODE_T  powerMode = POWER_SLEEP;
	uint32_t pdruncfg;

	if (estSleepTimeUs > g_Timer.pwrDown_threshold_us) {
#ifdef NDEBUG
		powerMode = POWER_POWER_DOWN;
#else
		powerMode = POWER_SLEEP;
#endif
	}
	else {
		if (estSleepTimeUs > g_Timer.sleep_threshold_us) {
			powerMode = POWER_SLEEP;
		}
		else {
			/* skip sleep */
			g_ResMgr.fSkipSleep = 1;
		}
	}

	if (g_ResMgr.fSkipSleep == 0) {

		//		DEBUG_LED_Set(LED_SLEEP_MODE_STATE, 1);

		if (g_ResMgr.currMode == RESMGR_PLL_MODE) {
			ResMgr_EnterNormalMode();
		}
		g_ResMgr.prevMode = g_ResMgr.currMode;
		g_ResMgr.currMode = RESMGR_PWRDOWN_MODE;

#if defined(CFG_FW_USE_LP_REG)
		Chip_SYSCON_PowerUp(
			SYSCON_PDRUNCFG_LP_VD1 | SYSCON_PDRUNCFG_LP_VD2 | SYSCON_PDRUNCFG_LP_VD3 | SYSCON_PDRUNCFG_LP_VD8);
		Chip_POWER_SetLPVDLevel();
#endif
		/* Go to power-down mode */
		if (powerMode == POWER_SLEEP) {
			__WFI();
		}
		else {
#ifdef CFG_FW_USE_EXTERNAL_RTC_CLOCK
			pdruncfg =	SYSCON_PDRUNCFG_PD_WDT_OSC | SYSCON_PDRUNCFG_PD_SRAM0A | SYSCON_PDRUNCFG_PD_SRAM0B | SYSCON_PDRUNCFG_PD_SRAM1;
#else
			pdruncfg = SYSCON_PDRUNCFG_PD_WDT_OSC | SYSCON_PDRUNCFG_PD_SRAM0A | SYSCON_PDRUNCFG_PD_SRAM0B | SYSCON_PDRUNCFG_PD_SRAM1 | SYSCON_PDRUNCFG_PD_32K_OSC;
#endif
			Chip_POWER_EnterPowerMode(powerMode, pdruncfg);
		}
#if defined(CFG_FW_USE_LP_REG)
		Chip_POWER_SetLPVDLevel();
		Chip_SYSCON_PowerDown(
			SYSCON_PDRUNCFG_LP_VD1 | SYSCON_PDRUNCFG_LP_VD2 | SYSCON_PDRUNCFG_LP_VD3 | SYSCON_PDRUNCFG_LP_VD8);
#endif

	}

	//	DEBUG_LED_Set(LED_SLEEP_MODE_STATE, 0);

	/* clear skip flag */
	g_ResMgr.fSkipSleep = 0;
	/* STALL untill flash is restored */
	g_ResMgr.currMode = RESMGR_NORMAL_MODE;
}

/* Request resource manager to put system in normal operation mode. */
void ResMgr_EnterNormalMode(void)
{
	if (g_ResMgr.currMode != RESMGR_NORMAL_MODE) {
//		if (g_ResMgr.currMode == RESMGR_PLL_MODE) {

			//			DEBUG_LED_Set(LED_PLL_PROCESSING, 0);			// Turn GPIO low

			Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_CLKIN);
			SystemCoreClockUpdate();
			/* trun off the PLL */
			Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_PD_SYS_PLL);
//		}

		/* Reduce the voltage level required to run at 12MHz system clock */
		Chip_POWER_SetVDLevel(POWER_VD1, POWER_V0850, POWER_FINE_V_NONE);
		Chip_POWER_SetVDLevel(POWER_VD8, POWER_V0900, POWER_FINE_V_NONE);
		Chip_SYSCON_SetFLASHAccess(FLASHTIM_20MHZ_CPU);

#if defined(CFG_FW_USE_LP_REG)
		Chip_POWER_SetLPVDLevel();

		Chip_SYSCON_PowerDown(
			SYSCON_PDRUNCFG_LP_VD1 | SYSCON_PDRUNCFG_LP_VD2 | SYSCON_PDRUNCFG_LP_VD3 | SYSCON_PDRUNCFG_LP_VD8);
#endif

		g_ResMgr.prevMode = g_ResMgr.currMode;
		g_ResMgr.currMode = RESMGR_NORMAL_MODE;
	}
}

/* Request resource manager to put system in PLL mode for higher computational power. */
void ResMgr_EnterPLLMode(void)
{
	if (g_ResMgr.currMode == RESMGR_NORMAL_MODE) {
		/* start the PLL */

		//		DEBUG_LED_Set(LED_PLL_PROCESSING, 1);	// Turn GPIO high

		/* By default, VDx levels are set to low to medium to preserve power.
		   In order to run the code at higher speed using PLL, the VDx level
		   have to be raised. Be careful with the FLASH wait state and VD level
		   if you want to set the PLL in order to run at the highest frequency possible. */
		Chip_POWER_SetVDLevel(POWER_VD1, POWER_V1100, POWER_FINE_V_NONE);
		Chip_POWER_SetVDLevel(POWER_VD8, POWER_V1100, POWER_FINE_V_NONE);

#if defined(CFG_FW_USE_LP_REG)
		Chip_SYSCON_PowerUp(
			SYSCON_PDRUNCFG_LP_VD1 | SYSCON_PDRUNCFG_LP_VD2 | SYSCON_PDRUNCFG_LP_VD3 | SYSCON_PDRUNCFG_LP_VD8);
#endif
		/* Set the flash to have 5 wait states (allows running up to 84Mhz) */
		Chip_Clock_SetSystemPLLSource(SYSCON_PLLCLKSRC_CLKIN);
		Chip_SYSCON_SetFLASHAccess(SYSCON_FLASH_5CYCLE);

		/* VD2_ANA needs to be powered up too. According to the design team, it's powered up at chip reset. */
		if (Chip_SYSCON_GetPowerStates() & SYSCON_PDRUNCFG_PD_VD2_ANA) {
			Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_VD2_ANA);
		}
		/* Turn on the PLL by clearing the power down bit */
		Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_SYS_PLL);

		/* Wait for PLL to lock */
		while (!Chip_Clock_IsSystemPLLLocked()) {}

		/* Set main clock source to the system PLL. This will drive 84MHz
		   for the main clock and 84MHz for the system clock */
		Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_PLLOUT);
		SystemCoreClockUpdate();
	}
	g_ResMgr.currMode = RESMGR_PLL_MODE;
}

int32_t ResMgr_EstimateSleepTime(void)
{
	uint32_t currTimeUs = g_Timer.GetCurrentUsec();

	uint32_t temp;
	uint32_t diff = CFG_FW_MAX_DURATION_US;

	PhysicalSensor_t *pSens;
	int32_t i;

	if (g_ResMgr.fSkipSleep) {
		g_ResMgr.fSkipSleep = 0;
		return 0;
	}

	for (i = 0; i < PHYS_MAX_ID; i++) {
		/* get next sensor in the list */
		pSens = g_phySensors[i];
		if ((pSens == NULL) || (!pSens->enabled) || (pSens->period == 0)) {
			continue;
		}

		/* if we just received an IRQ then don't go to power down */
		if ( pSens->irq_pending ) {
			return 0;
		}
		temp = pSens->ts_nextSampleUs - currTimeUs;
		/* if difference is too large, it means we passed the time,
		   or if we just received an IRQ then don't go to power down */
		if ((temp >= (CFG_FW_MAX_DURATION_US / 2)) || pSens->irq_pending ) {
			temp = 0;
		}

		if (temp < diff) {
			diff = temp;
		}
	}
	return diff;
}

void Kernel_Init(void)
{
	/* Initialize Timer service */
	g_Timer.init();
	/* Initialize resource manager */
	ResMgr_Init();
}
