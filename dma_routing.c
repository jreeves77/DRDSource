/*
 * @brief DMA router for Niobe Sensor app
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
#include "kernel_res_mgr.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

extern void dev_i2cm_dma_callback(bool error);

extern void dev_i2cs_dma_callback(bool error);

extern void Hostif_SpiDmaTxCB(bool error);

extern void Hostif_SpiDmaRxCB(bool error);

/* Reset DMA Channel */
void dmaClearChannel(DMA_CHID_T ch)
{
	Chip_DMA_DisableChannel(LPC_DMA, ch);
	while ((Chip_DMA_GetBusyChannels(LPC_DMA) & (1 << ch)) != 0) {}

	Chip_DMA_AbortChannel(LPC_DMA, ch);
	Chip_DMA_ClearErrorIntChannel(LPC_DMA, ch);
}

uint16_t dmaDetermineTransferSize(DMA_CHID_T dmaCh, uint32_t numXfers, uint32_t xferSize)
{
	uint32_t cnt;

	/* Determine count datum transferred */
	cnt = ((LPC_DMA->DMACH[dmaCh].XFERCFG >> 16) & 0x3FF);
	if (cnt == 0x3ff) {
		/* Entire DMA buffer was used */
		cnt = numXfers;
	}
	else {
		/* Partial DMA buffer was used */
		cnt = numXfers - cnt - 1;
	}

	/* Modify count by size */
	cnt = cnt * xferSize;

	return cnt;
}

/**
 * @brief	DMA Interrupt Handler
 * @return	None
 */
void DMA_IRQHandler(void)
{

#if defined(CFG_FW_SENSOR_I2C_DMA_BASED) || defined(CFG_FW_HOSTIF_SPI) || defined(CFG_FW_HOSTIF_I2C)
	uint32_t errors, pending;

	/* Get DMA error and interrupt channels */
	errors = Chip_DMA_GetErrorIntChannels(LPC_DMA);
	pending = Chip_DMA_GetActiveIntAChannels(LPC_DMA);

#endif

#if defined(CFG_FW_SENSOR_I2C_DMA_BASED)
	/* Check DMA interrupts of I2C0 master channel */
	if ((errors | pending) & (1 << CFG_FW_SENSOR_I2C_DMAID)) {
		/* Clear DMA interrupt for the channel */
		Chip_DMA_ClearActiveIntAChannel(LPC_DMA, CFG_FW_SENSOR_I2C_DMAID);

		dev_i2cm_dma_callback((errors & (1 << CFG_FW_SENSOR_I2C_DMAID)) != 0);
	}
#endif

#if defined(CFG_FW_HOSTIF_SPI)
	/* Check DMA interrupts of SPI slave Tx channel */
	if ((errors | pending) & (1 << CFG_FW_HOSTIF_SPI_DMA_TXCH)) {
		/* Clear DMA interrupt for the channel */
		Chip_DMA_ClearActiveIntAChannel(LPC_DMA, CFG_FW_HOSTIF_SPI_DMA_TXCH);

		Hostif_SpiDmaTxCB((errors & (1 << CFG_FW_HOSTIF_SPI_DMA_TXCH)) != 0);
	}
	/* Check DMA interrupts of SPI slave Rx channel */
	if ((errors | pending) & (1 << CFG_FW_HOSTIF_SPI_DMA_RXCH)) {
		/* Clear DMA interrupt for the channel */
		Chip_DMA_ClearActiveIntAChannel(LPC_DMA, CFG_FW_HOSTIF_SPI_DMA_RXCH);

		Hostif_SpiDmaRxCB((errors & (1 << CFG_FW_HOSTIF_SPI_DMA_RXCH)) != 0);
	}
#elif defined(CFG_FW_HOSTIF_I2C)
	/* Check DMA interrupts of I2C2 slave channel */
	if ((errors | pending) & (1 << CFG_FW_HOSTIF_I2C_DMAID)) {
		/* Clear DMA interrupt for the channel */
		Chip_DMA_ClearActiveIntAChannel(LPC_DMA, CFG_FW_HOSTIF_I2C_DMAID);

		dev_i2cs_dma_callback((errors & (1 << CFG_FW_HOSTIF_I2C_DMAID)) != 0);
	}
#endif

	ResMgr_IRQDone();
}
