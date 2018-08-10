/*
 * @brief Common SystemInit function for LPC412x chips
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

#define __INCLUDE_SYS_INIT__

#include <stdio.h>
#include "sensorhub.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
const uint32_t ExtClockIn = 24000000;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system pin muxing */
static void Board_SetupMuxing(void)
{
	/* Enable IOCON clock */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);

	Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing, sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));

	/* IOCON clock left on, this is needed if CLKIN is used. */
}

/* Set up and initialize clocking prior to call to main */
static void Board_SetupClocking(void)
{
	/* The IRC is always the first clock source even if CLK_IN is used later.
	   Once CLK_IN is selected as the clock source. We can turned off the IRC later.
	   Turn on the IRC by clearing the power down bit */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_PD_IRC_OSC | SYSCON_PDRUNCFG_PD_IRC);

	/* Wait State setting TBD */
	/* Setup FLASH access to 2 clocks (up to 20MHz) */
	Chip_SYSCON_SetFLASHAccess(FLASHTIM_20MHZ_CPU);

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 24MHz
	   for the main clock and 24MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_CLKIN);

	/* Select the CLKOUT clocking source */
	Chip_Clock_SetCLKOUTSource(SYSCON_CLKOUTSRC_MAINCLK, 1);

	/* ASYSNC SYSCON needs to be on or all serial peripheral won't work.
	   Be careful if PLL is used or not, ASYNC_SYSCON source needs to be
	   selected carefully. */
	Chip_SYSCON_Enable_ASYNC_Syscon(true);
	Chip_Clock_SetAsyncSysconClockDiv(1);
	Chip_Clock_SetAsyncSysconClockSource(SYSCON_ASYNC_CLKIN);
}

/* Set up and initialize hardware prior to call to main */
static void Board_SystemInit(void)
{
	/* Setup system clocking and muxing */
	Board_SetupMuxing();/* Muxing first as it sets up ext oscillator pins */
	Board_SetupClocking();
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
#if defined(__CC_ARM)
void _ttywrch(int ch)
{
	ch = ch;
}

struct __FILE {
	int handle;
};

FILE __stdout;
FILE __stdin;
FILE __stderr;

void *_sys_open(const char *name, int openmode)
{
	return 0;
}

int fputc(int c, FILE *f)
{
	_ttywrch(c);
	return 0;
}

int fgetc(FILE *f)
{
	return 0;
}

int ferror(FILE *f)
{
	return EOF;
}

void _sys_exit(int return_code)
{
	while (1) {
		__WFI(); /* endless loop */
	}
}

#endif

/* Set up and initialize hardware prior to call to main */
void SystemInit(void)
{
#if defined(__CODE_RED)
	extern void(*const g_pfnVectors[]) (void);
	SCB->VTOR = (uint32_t) &g_pfnVectors;
#else
	extern void *__Vectors;
	SCB->VTOR = (uint32_t) &__Vectors;
#endif

#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
	fpuInit();
#endif

	/* Board specific SystemInit */
	Board_SystemInit();
}
