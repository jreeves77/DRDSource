/*
 * @brief Kernel debug module for application
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
#include <stdio.h>
#include "kernel_debug.h"
#include "kernel_timer.h"

#ifndef NDEBUG

#define TRACE_SIZE 1000

volatile static opTraceCode_t trace_op[TRACE_SIZE] = {op_none};
volatile static uint32_t trace_param[TRACE_SIZE] = {0};
volatile static uint32_t trace_timeTicks[TRACE_SIZE] = {0};
volatile static uint32_t trace_timeUs[TRACE_SIZE] = {0};
static int trace_idx = 0;

uint32_t    intStatus;

#endif /* NDEBUG */

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#if defined(DEBUG_LED)
/* Initialize the LEDs on the NXP LPC54000 LPCXpresso Board */
void DEBUG_LED_Init(void)
{
	int i;

	/* Pin muxing setup as part of board_sysinit */
	for (i = 0; i < NUM_LED_DEBUG; i++) {
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, ledBits[i][0], ledBits[i][1]);
		Chip_GPIO_SetPinState(LPC_GPIO, ledBits[i][0], ledBits[i][1], true);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set the LED to the state of "On" */
void DEBUG_LED_Set(uint8_t LEDNumber, bool On)
{
	if (LEDNumber < sizeof(ledBits)) {
		Chip_GPIO_SetPinState(LPC_GPIO, ledBits[LEDNumber][0], ledBits[LEDNumber][1], (bool) !On);
	}
}

/* Return the state of LEDNumber */
bool DEBUG_LED_Test(uint8_t LEDNumber)
{
	if (LEDNumber < sizeof(ledBits)) {
		return (bool) !Chip_GPIO_GetPinState(LPC_GPIO, ledBits[LEDNumber][0], ledBits[LEDNumber][1]);
	}
	return false;
}

/* Toggles the current state of a board LED */
void DEBUG_LED_Toggle(uint8_t LEDNumber)
{
	if (LEDNumber < sizeof(ledBits)) {
		Chip_GPIO_SetPinToggle(LPC_GPIO, ledBits[LEDNumber][0], ledBits[LEDNumber][1]);
	}
}

#endif

#if defined(DEBUG_UART)

/* Sends a character on the UART */
void Board_UARTPutChar(char ch)
{
	Chip_UART_SendBlocking(DEBUG_UART, &ch, 1);
}

/* Gets a character from the UART, returns EOF if no character is ready */
int Board_UARTGetChar(void)
{
	uint8_t data;

	if (Chip_UART_Read(DEBUG_UART, &data, 1) == 1) {
		return (int) data;
	}
	return EOF;
}

/* Outputs a string on the debug UART */
void Board_UARTPutSTR(char *str)
{
	while (*str != '\0') {
		Board_UARTPutChar(*str++);
	}
}

/* Board Debug UART Initialisation function */
void Board_UART_Init(void)
{
	Chip_SYSCON_Enable_ASYNC_Syscon(true);
#ifndef CHIP_LPC5411X
	/* Divided by 1 */
	Chip_Clock_SetAsyncSysconClockDiv(1);
#endif

	/* UART0 pins are setup in board_sysinit.c */
}

#else

#define Board_UARTPutSTR(str)
#define Board_UARTGetChar()
#define Board_UARTPutChar(ch)
#define Board_UART_Init()

#endif

/* Initialize debug output via UART for board */
void Board_Debug_Init(void)
{
	Board_UART_Init();
	Chip_UART_Init(DEBUG_UART);
	Chip_UART_ConfigData(DEBUG_UART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(DEBUG_UART, 115200);
	Chip_UART_Enable(DEBUG_UART);
	Chip_UART_TXEnable(DEBUG_UART);
}

/* Set up and initialize all required blocks and functions related to the
   board hardware */
void Board_Init(void)
{
	/* INMUX and IOCON are used by many apps, enable both INMUX and IOCON clock bits here. */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);

	/* Sets up DEBUG UART */
	DEBUGINIT();

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO);

	/* Initialize the LEDs. Be careful with below routine, once it's called some of the I/O will be set to output. */
	DEBUG_LED_Init();
}

#ifndef NDEBUG

/* This debug trace is different then logging service, debug trace is for development only for viewing traces in the debugger,
    the debug trace is not transmitted to the host  */
void DEBUG_TRACE(opTraceCode_t op, uint32_t param)
{
	INT_DISABLE(intStatus, 0);

	trace_op[trace_idx] = op;
	trace_param[trace_idx] = param;
	trace_timeTicks[trace_idx] = (uint32_t) g_Timer.GetCurrent();
	trace_timeUs[trace_idx] = (uint32_t) g_Timer.GetCurrentUsec();
	++trace_idx;
	trace_idx %= TRACE_SIZE;
	if (trace_idx == 0) {
		trace_idx = 0;	// for breakpoint at trace full.
	}

	INT_RESTORE(intStatus, 0);
}

#endif /* NDEBUG */
