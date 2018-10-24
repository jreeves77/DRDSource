/*
 * @brief Blinky example using SysTick and interrupt
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

#include <string.h>
#include <stdio.h>
#include "sensorhub.h"

#include "kernel_timer.h"
#include "kernel_res_mgr.h"
#include "kernel_debug.h"
#include "arm_math.h"
#include "spim_5410x.h"
#include "SEGGER_RTT.h"
#include "dsp.h"
#include "m0main.h"
#include "drdstate.h"

#define BEEP_COUNT 7

#define MESSAGE_SOFTWARE_VERSION 101

#define TRANSFER_SIZE 256
#define STACK_SIZE 1024

#define BLOCK_SIZE 256
#define WINDOW_FREQUENCY (SAMPLE_FREQUENCY/BLOCK_SIZE)
#define FILTER_LENGTH (TRANSFER_SIZE/2)
#define MAGIC_NUMBER (3744.8222f)
#define VOLTAGE_THRESHOLD 9000000.0
#define CURRENT_THRESHOLD 400.0
#define NUMBER_INT_AD_CHANNELS 5
#define FIRST_IIR_CONSTANT 1.5
#define SECOND_IIR_CONSTANT .5625
#define THIRD_IIR_CONSTANT .0625
#define VOLTAGE_REF (2.5f)
#define VOLTAGE_PER_BIT (VOLTAGE_REF/65536.0f)

#define ADC0_GAIN 3.0f
#define ADC1_GAIN 2.0f
#define ADC7_GAIN 3.0f

#define SENSTYPE_NONE       0
#define SENSTYPE_RAILSENS   1
#define SENSTYPE_SHUNT      2
#define SENSTYPE_DIRECT     4

#define SENSDET_SHORT    0x0100
#define SENSDET_200OHM   0x0200
#define SENSDET_500OHM   0x0400

#define BUTTON_FD    1
#define BUTTON_FU    2
#define BUTTON_TYPE  4
#define BUTTON_MODE  8
#define BUTTON_BL   16

#define LOW_BATTERY_THRESHOLD 4.0f
#define LOW_BATTERY_WARNING 4.25f

#include "ret.h"
// #include "stl.h"

unsigned int Parameters[64];
unsigned int inactivePeriod;

CONFIG_HEADER_T *DRDConfig, ConfigParameters;

struct iir_filter
{
  float Y[3];
} IIR_Filters[NUMBER_INT_AD_CHANNELS];

struct _drd_state_ DRDState;

unsigned char lcdbuf[20];
unsigned char TransmitRingBuffer[RING_BUFFER_SIZE];
unsigned char ReceiveRingBuffer[RING_BUFFER_SIZE];
unsigned char TransmitRingBuffer1[RING_BUFFER_SIZE];
unsigned char ReceiveRingBuffer1[RING_BUFFER_SIZE];
unsigned short LCDTransmitBuffer[LCD_RING_BUFFER_SIZE];

struct ring_buffer USART0TransmitRingBuffer = {RING_BUFFER_SIZE, 0, 0, TransmitRingBuffer};
struct ring_buffer USART0ReceiveRingBuffer = {RING_BUFFER_SIZE, 0, 0, ReceiveRingBuffer};
struct lcd_ring_buffer LCDTransmitRingBuffer = {LCD_RING_BUFFER_SIZE, 0, 0, LCDTransmitBuffer};

UART_BAUD_T BaudRate;

typedef enum {
  TRANSMIT1A_DESC,
  CLR_PININT1_DESC,
  TRANSMIT1B_DESC,
  CLR_PININT2_DESC,
  TRIGGER0_DESC,
  TRIGGER3_DESC,
  RECEIVE1_DESC,
  TRIGGER1_DESC,
  TRANSMIT2_DESC,
  RECEIVE2_DESC,
  TRIGGER2_DESC,
  COLLECTA_DESC,
  COLLECTB_DESC,
  NUMBER_OF_DESCRIPTORS
} Dma_Descriptors1;

#if defined(__CC_ARM)
__align(16) static DMA_CHDESC_T dmaSPIMDesc[NUMBER_OF_DESCRIPTORS];
#elif defined(__ICCARM__)
#pragma data_alignment=16
static DMA_CHDESC_T dmaSPIMDesc[NUMBER_OF_DESCRIPTORS];
#elif defined( __GNUC__ )
static DMA_CHDESC_T dmaSPIMDesc[NUMBER_OF_DESCRIPTORS] __attribute__ ((aligned(16)));
#endif

int M0PlusStack[STACK_SIZE];

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

typedef uint32_t (*ProcessPtr_t)(void);

uint64_t total_time;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

uint32_t selectBuffer, triggerBufferA;
uint32_t selectBufferA, pinIntClear, triggerBuffer0;

uint16_t rxBuff[2][TRANSFER_SIZE], currentBuffer, DMAInterrupt, ADCInterrupt;
uint32_t rxTemp;

int done = 0;
uint64_t loop_count;

void UART_IRQHandler( LPC_USART_T *uart, struct ring_buffer *rtPointer, struct ring_buffer *rrPointer );
extern void dmaClearChannel(DMA_CHID_T ch);

void ClearScreen( void );
void LCD_Init( struct _drd_state_ *state );

/*
    Name      : SendBytes

    Purpose   : The purpose of this routine is to add characters to the outgoing serial string.

    Argurments:
       rPointer - Pointer to the ring buffer data structure.
       buffer   - Buffer containing the outgoing data.
       count    - Count of outgoing characters.

    Return Value : Number of character sent.
*/

int SendBytes( LPC_USART_T *uart, struct ring_buffer *rPointer, unsigned char *buffer, int count )
{
  int RetValue = 0;
  int i, nextByte;
  int bufferIndex = 0;

  if ( Chip_UART_GetStatus( uart ) & UART_STAT_TXRDY )
  {
    if ( rPointer -> readIndex != rPointer -> writeIndex )
    {
      Chip_UART_SendByte( uart, rPointer -> buffer[rPointer -> readIndex] );

      rPointer -> readIndex += 1;

      if ( rPointer -> readIndex >= rPointer -> size )
      {
        rPointer -> readIndex = 0;
      }
    }
    else
    {
      Chip_UART_SendByte( uart, buffer[0] );

      bufferIndex += 1;

      count -= 1;

      RetValue += 1;
    }
  }

  for( i = 0;i < count;i++ )
  {
    nextByte = rPointer -> writeIndex + 1;

    if ( nextByte >= rPointer -> size )
    {
      nextByte = 0;
    }

    if ( nextByte != rPointer -> readIndex )
    {
      rPointer -> buffer[rPointer -> writeIndex] = buffer[i + bufferIndex];

      rPointer -> writeIndex = nextByte;

      RetValue += 1;
    }
    else
    {
      break;
    }
  }

  if ( RetValue != 0 )
  {
    Chip_UART_IntEnable( uart, UART_INTEN_TXRDY );
  }

  return( RetValue );
}


/*
    Name      : ReceiveBytes

    Purpose   : The purpose of this routine is to move bytes from the receive ring buffer to the users buffer.

    Argurments:
       rPointer - Pointer to the data structure for the receive ring buffer.
       buffer   - Pointer to the users buffer.
       count    - Length of the users buffer.

    Return Value : There is no return value.
*/

int ReceiveBytes( LPC_USART_T *uart, struct ring_buffer *rPointer, unsigned char *buffer, int count )
{
  int RetValue = 0;
  int bufferIndex = 0;

  if ( rPointer -> readIndex != rPointer -> writeIndex )
  {
    while( (count > 0) && (rPointer -> readIndex != rPointer -> writeIndex) )
    {
      buffer[bufferIndex++] = rPointer -> buffer[rPointer -> readIndex];

      count -= 1;

      RetValue += 1;

      rPointer -> readIndex += 1;

      if ( rPointer -> readIndex >= rPointer -> size )
      {
        rPointer -> readIndex = 0;
      }
    }
  }

  return( RetValue );
}

/*
    Name      : GetChar

    Purpose   : The purpose of this routine is to one byte from the receive ring buffer and return it to the user.

    Argurments:
       rPointer - Pointer to the data structure for the receive ring buffer.
       byte     - Address of the place to put the character returned.

    Return Value : One if a byte was returned and a zero otherwise.
*/

int GetChar( struct ring_buffer *rPointer, unsigned char *byte )
{
  int RetValue = 0;

  if ( rPointer -> readIndex != rPointer -> writeIndex )
  {
    byte[0] = rPointer -> buffer[rPointer -> readIndex];

    RetValue += 1;

    rPointer -> readIndex += 1;

    if ( rPointer -> readIndex >= rPointer -> size )
    {
      rPointer -> readIndex = 0;
    }
  }

  return( RetValue );
}

/*
    Name      : UART0_IRQHandler1

    Purpose   : This routine is the interrupt handler for UART0.

    Argurments: It has no arguments.

    Return Value : It has no return value.
*/

void UART0_IRQHandler1(void)
{
  uint64_t eTime, sTime;

  sTime = g_Timer.GetCurrentUsec();

  UART_IRQHandler( LPC_USART0, &USART0TransmitRingBuffer, &USART0ReceiveRingBuffer );

  eTime = g_Timer.GetCurrentUsec() - sTime;

  total_time += eTime;
}

void UART_IRQHandler( LPC_USART_T *uart, struct ring_buffer *rtPointer, struct ring_buffer *rrPointer )
{
  unsigned int status = Chip_UART_GetIntStatus( uart );
  unsigned char nextByte;
  int nextIndex;

  if ( status & UART_INT_TXRDY )
  {
    if ( rtPointer -> readIndex != rtPointer -> writeIndex )
    {
      Chip_UART_SendByte( uart, rtPointer -> buffer[rtPointer -> readIndex] );

      rtPointer -> readIndex += 1;

      if ( rtPointer -> readIndex >= rtPointer -> size )
      {
        rtPointer -> readIndex = 0;
      }
    }
    else
    {
      Chip_UART_IntDisable( uart, UART_INTEN_TXRDY );
    }
  }

  if ( status & UART_INT_RXRDY )
  {
    nextByte = Chip_UART_ReadByte( uart );

    nextIndex = rrPointer -> writeIndex + 1;

    if ( nextIndex >= rrPointer -> size )
    {
      nextIndex = 0;
    }

    if ( nextIndex !=  rrPointer -> readIndex )
    {
      rrPointer -> buffer[rrPointer -> writeIndex] = nextByte;

      rrPointer -> writeIndex = nextIndex;
    }
  }
}

/*
    Name      : DMA_IRQHandler1

    Purpose   : This routine is the interrupt handler for the DMA controller.

    Argurments: It has no arguments.

    Return Value : It has no return value.
*/

void DMA_IRQHandler1(void)
{
  uint64_t eTime, sTime;
  static uint64_t lTime = 0;

  sTime = g_Timer.GetCurrentUsec();
  if ( lTime == 0 )
  {
    lTime = sTime;
  }
  else
  {
    lTime = 0;
  }

  currentBuffer ^= 1;

  DMAInterrupt = 1;

  Chip_GPIO_SetPinState( LPC_GPIO, 1,  5, true );

  LPC_ADC->SEQ_CTRL[ADC_SEQA_IDX] |= ADC_SEQ_CTRL_START;

  Chip_DMA_ClearActiveIntAChannel( LPC_DMA, DMAREQ_SPI1_TX );

  eTime = g_Timer.GetCurrentUsec() - sTime;

  total_time += eTime;
}

/*
    Name      : CT32B0_IRQHandler1

    Purpose   : This routine is the interrupt handler for the first general purpose timer.

    Argurments: It has no arguments.

    Return Value : It has no return value.
*/

void CT32B1_IRQHandler1(void)
{
  unsigned char nextByte;
  int nextIndex;
  struct lcd_ring_buffer *rtPointer = &LCDTransmitRingBuffer;

#if 0
  if ( LCD_IsBusy() )
  {
    DRDState.LCDPresent = 0;

    Chip_TIMER_Disable( LPC_TIMER1 );

    return;
  }
#endif


  if ( rtPointer -> readIndex != rtPointer -> writeIndex )
  {
    rtPointer -> readIndex += 1;

    if ( rtPointer -> readIndex >= rtPointer -> size )
    {
      rtPointer -> readIndex = 0;
    }
  }

  if ( rtPointer -> readIndex != rtPointer -> writeIndex )
  {
    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0) & ~SPI_TXDATCTL_EOT) | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x40);

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDAT = 0x0A;

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
    LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA((LCDTransmitBuffer[rtPointer -> readIndex] & 0xff));

    while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_MSTIDLE) );

    LPC_SPI1->TXDATCTL = ((LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK) | SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0);

    if ( LCDTransmitBuffer[rtPointer -> readIndex] & LCD_RS )
    {
      Chip_GPIO_SetPinState( LPC_GPIO, 1,  8, true );
    }
    else
    {
      Chip_GPIO_SetPinState( LPC_GPIO, 1,  8, false );
    }

    if ( LCDTransmitBuffer[rtPointer -> readIndex] & LCD_RW )
    {
      Chip_GPIO_SetPinState( LPC_GPIO, 1,  10, true );
    }
    else
    {
      Chip_GPIO_SetPinState( LPC_GPIO, 1,  10, false );
    }
  }
  else
  {
    Chip_GPIO_SetPinState( LPC_GPIO, 1,  8, false );

    Chip_GPIO_SetPinState( LPC_GPIO, 1,  10, true );

    Chip_TIMER_MatchDisableInt( LPC_TIMER1, 3 );
  }

  LPC_TIMER1->IR = TIMER_IR_CLR( 3 );
}

void SetupSPIDMA1( void )
{
  Chip_DMA_SetupChannelConfig( LPC_DMA, DMAREQ_SPI0_TX,
         (DMA_CFG_HWTRIGEN | DMA_CFG_TRIGPOL_HIGH  | DMA_CFG_TRIGTYPE_EDGE | DMA_CFG_TRIGBURST_SNGL | DMA_CFG_CHPRIORITY(0)));

  Chip_DMA_SetupChannelConfig( LPC_DMA, DMAREQ_SPI0_RX,
         (DMA_CFG_PERIPHREQEN | DMA_CFG_TRIGBURST_SNGL | DMA_CFG_CHPRIORITY(1)));

  Chip_DMA_SetupChannelConfig( LPC_DMA, DMAREQ_SPI1_TX,
         (DMA_CFG_HWTRIGEN | DMA_CFG_TRIGPOL_LOW  | DMA_CFG_TRIGTYPE_EDGE | DMA_CFG_TRIGBURST_BURST |
          DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));

  Chip_DMA_SetupChannelConfig( LPC_DMA, DMAREQ_I2C0_MASTER, 
         (DMA_CFG_HWTRIGEN | DMA_CFG_TRIGPOL_LOW | DMA_CFG_TRIGTYPE_EDGE | DMA_CFG_TRIGBURST_SNGL | DMA_CFG_CHPRIORITY(1)));

  Chip_INMUX_SetDMATrigger( DMAREQ_SPI0_TX, DMATRIG_PININT0 );

  dmaClearChannel( DMAREQ_SPI0_TX );
  dmaClearChannel( DMAREQ_SPI0_RX );
  dmaClearChannel( DMAREQ_I2C0_MASTER );

  selectBufferA = (LPC_SPI0->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(1)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_DATA(0xffff);
  triggerBuffer0 = 1 << DMAREQ_I2C0_MASTER;
  triggerBufferA = 1 << DMAREQ_SPI1_TX;
  pinIntClear = 1;

  dmaSPIMDesc[TRANSMIT1A_DESC].source   = DMA_ADDR(&selectBufferA);
  dmaSPIMDesc[TRANSMIT1A_DESC].dest     = DMA_ADDR( &LPC_SPI0->TXDATCTL );
  dmaSPIMDesc[TRANSMIT1A_DESC].next     = DMA_ADDR(&dmaSPIMDesc[CLR_PININT1_DESC]);
  dmaSPIMDesc[TRANSMIT1A_DESC].xfercfg  = (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_WIDTH_32 |
                                             DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(1));
  
  dmaSPIMDesc[CLR_PININT1_DESC].source  = DMA_ADDR(&pinIntClear);
  dmaSPIMDesc[CLR_PININT1_DESC].dest    = DMA_ADDR(&LPC_PININT->IST);
  dmaSPIMDesc[CLR_PININT1_DESC].next    = DMA_ADDR(&dmaSPIMDesc[TRANSMIT1B_DESC]);
  dmaSPIMDesc[CLR_PININT1_DESC].xfercfg = DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_CLRTRIG | DMA_XFERCFG_WIDTH_32 |
                                            DMA_XFERCFG_SRCINC_1 | DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(1);

  dmaSPIMDesc[TRANSMIT1B_DESC].source   = DMA_ADDR(&selectBufferA);
  dmaSPIMDesc[TRANSMIT1B_DESC].dest     = DMA_ADDR( &LPC_SPI0->TXDATCTL);
  dmaSPIMDesc[TRANSMIT1B_DESC].next     = DMA_ADDR(&dmaSPIMDesc[CLR_PININT2_DESC]);
  dmaSPIMDesc[TRANSMIT1B_DESC].xfercfg  = (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_WIDTH_32 |
                                             DMA_XFERCFG_SRCINC_1 | DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(1));
  
  dmaSPIMDesc[CLR_PININT2_DESC].source  = DMA_ADDR(&pinIntClear);
  dmaSPIMDesc[CLR_PININT2_DESC].dest    = DMA_ADDR(&LPC_PININT->IST);
  dmaSPIMDesc[CLR_PININT2_DESC].next    = DMA_ADDR(&dmaSPIMDesc[TRANSMIT1A_DESC]);
  dmaSPIMDesc[CLR_PININT2_DESC].xfercfg = DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_CLRTRIG | DMA_XFERCFG_WIDTH_32 |
                                            DMA_XFERCFG_SRCINC_1 | DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(1);

  dmaSPIMDesc[TRIGGER0_DESC].source  = DMA_ADDR(&triggerBuffer0);
  dmaSPIMDesc[TRIGGER0_DESC].dest    = DMA_ADDR(&LPC_DMA->DMACOMMON[0].SETTRIG);
  dmaSPIMDesc[TRIGGER0_DESC].next    = DMA_ADDR(&dmaSPIMDesc[TRIGGER3_DESC]);
  dmaSPIMDesc[TRIGGER0_DESC].xfercfg = (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_SWTRIG | DMA_XFERCFG_WIDTH_32 |
                                          DMA_XFERCFG_SRCINC_1 | DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(1));

  dmaSPIMDesc[TRIGGER3_DESC].source  = DMA_ADDR(&triggerBuffer0);
  dmaSPIMDesc[TRIGGER3_DESC].dest    = DMA_ADDR(&LPC_DMA->DMACOMMON[0].SETTRIG);
  dmaSPIMDesc[TRIGGER3_DESC].next    = DMA_ADDR(&dmaSPIMDesc[TRIGGER0_DESC]);
  dmaSPIMDesc[TRIGGER3_DESC].xfercfg = (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_SWTRIG | DMA_XFERCFG_WIDTH_32 |
                                          DMA_XFERCFG_SRCINC_1 | DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(1));

  dmaSPIMDesc[RECEIVE1_DESC].source  = DMA_ADDR(&LPC_SPI0->RXDAT);
  dmaSPIMDesc[RECEIVE1_DESC].dest    = DMA_ADDR(&rxTemp);
  dmaSPIMDesc[RECEIVE1_DESC].next    = DMA_ADDR(&dmaSPIMDesc[TRIGGER1_DESC]);
  dmaSPIMDesc[RECEIVE1_DESC].xfercfg = (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_WIDTH_32 |
                                          DMA_XFERCFG_SRCINC_1 | DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(1));

  dmaSPIMDesc[TRIGGER1_DESC].source  = DMA_ADDR(&triggerBufferA);
  dmaSPIMDesc[TRIGGER1_DESC].dest    = DMA_ADDR(&LPC_DMA->DMACOMMON[0].SETTRIG);
  dmaSPIMDesc[TRIGGER1_DESC].next    = DMA_ADDR(&dmaSPIMDesc[RECEIVE1_DESC]);
  dmaSPIMDesc[TRIGGER1_DESC].xfercfg = DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_CLRTRIG | DMA_XFERCFG_WIDTH_32 |
                                         DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_1 | DMA_XFERCFG_XFERCOUNT(1);

  dmaSPIMDesc[COLLECTA_DESC].source  = DMA_ADDR(&rxTemp);
  dmaSPIMDesc[COLLECTA_DESC].dest    = DMA_ADDR(&rxBuff[0][0]) + TRANSFER_SIZE*2 - 2;
  dmaSPIMDesc[COLLECTA_DESC].next    = DMA_ADDR(&dmaSPIMDesc[COLLECTB_DESC]);
  dmaSPIMDesc[COLLECTA_DESC].xfercfg = (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_CLRTRIG | DMA_XFERCFG_SETINTA | DMA_XFERCFG_WIDTH_16 |
                                          DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_1 | DMA_XFERCFG_XFERCOUNT(TRANSFER_SIZE));

  dmaSPIMDesc[COLLECTB_DESC].source  = DMA_ADDR(&rxTemp);
  dmaSPIMDesc[COLLECTB_DESC].dest    = DMA_ADDR(&rxBuff[1][0]) + TRANSFER_SIZE*2 - 2;
  dmaSPIMDesc[COLLECTB_DESC].next    = DMA_ADDR(&dmaSPIMDesc[COLLECTA_DESC]);
  dmaSPIMDesc[COLLECTB_DESC].xfercfg = (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_CLRTRIG | DMA_XFERCFG_SETINTA | DMA_XFERCFG_WIDTH_16 |
                                          DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_1 | DMA_XFERCFG_XFERCOUNT(TRANSFER_SIZE));

  while(!Chip_DMA_SetupTranChannel(LPC_DMA, DMAREQ_SPI0_TX, &dmaSPIMDesc[TRANSMIT1A_DESC])) {};

  while(!Chip_DMA_SetupTranChannel(LPC_DMA, DMAREQ_SPI0_RX, &dmaSPIMDesc[TRIGGER0_DESC])) {};

  while(!Chip_DMA_SetupTranChannel(LPC_DMA, DMAREQ_SPI1_TX, &dmaSPIMDesc[COLLECTA_DESC])) {};

  while(!Chip_DMA_SetupTranChannel(LPC_DMA, DMAREQ_I2C0_MASTER, &dmaSPIMDesc[RECEIVE1_DESC])) {};

  Chip_DMA_SetupChannelTransfer( LPC_DMA, DMAREQ_SPI0_TX, (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_WIDTH_32 |
                                          DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_0 | DMA_XFERCFG_XFERCOUNT(1)) );

  Chip_DMA_SetupChannelTransfer( LPC_DMA, DMAREQ_SPI0_RX, (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_SWTRIG | DMA_XFERCFG_WIDTH_32 |
                                                           DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_1 | DMA_XFERCFG_XFERCOUNT(1)) );
                                          
  Chip_DMA_SetupChannelTransfer( LPC_DMA, DMAREQ_SPI1_TX, (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_SETINTA | DMA_XFERCFG_WIDTH_16 |
                                                           DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_1 | DMA_XFERCFG_XFERCOUNT(TRANSFER_SIZE)) );

  Chip_DMA_SetupChannelTransfer( LPC_DMA, DMAREQ_I2C0_MASTER, (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_RELOAD | DMA_XFERCFG_WIDTH_32 |
                                                           DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_1 | DMA_XFERCFG_XFERCOUNT(1)) );

  Chip_DMA_EnableChannel(LPC_DMA, DMAREQ_SPI0_TX);
  Chip_DMA_EnableChannel(LPC_DMA, DMAREQ_SPI0_RX);
  Chip_DMA_EnableChannel(LPC_DMA, DMAREQ_SPI1_TX);
  Chip_DMA_EnableChannel(LPC_DMA, DMAREQ_I2C0_MASTER);

  Chip_DMA_EnableIntChannel( LPC_DMA, DMAREQ_SPI1_TX );

  NVIC_EnableIRQ( DMA_IRQn );
}

void ADC_SEQA_IRQHandler1(void)
{
  uint32_t adcData;
  uint64_t eTime, sTime;

  sTime = g_Timer.GetCurrentUsec();

  Chip_GPIO_SetPinState( LPC_GPIO, 1,  5, false );

  ADCInterrupt = 1;

  adcData = Chip_ADC_GetDataReg( LPC_ADC, 0 );

  IIR_Filters[0].Y[0] = FIRST_IIR_CONSTANT*IIR_Filters[0].Y[1] - SECOND_IIR_CONSTANT*IIR_Filters[0].Y[2] + THIRD_IIR_CONSTANT*(adcData & 0xffff)*VOLTAGE_PER_BIT*ADC0_GAIN;

  IIR_Filters[0].Y[2] = IIR_Filters[0].Y[1];

  IIR_Filters[0].Y[1] = IIR_Filters[0].Y[0];

  adcData= Chip_ADC_GetDataReg( LPC_ADC, 1 );

  IIR_Filters[1].Y[0] = FIRST_IIR_CONSTANT*IIR_Filters[1].Y[1] - SECOND_IIR_CONSTANT*IIR_Filters[1].Y[2] + THIRD_IIR_CONSTANT*(adcData & 0xffff)*VOLTAGE_PER_BIT*ADC1_GAIN;

  IIR_Filters[1].Y[2] = IIR_Filters[1].Y[1];

  IIR_Filters[1].Y[1] = IIR_Filters[1].Y[0];

  adcData = Chip_ADC_GetDataReg( LPC_ADC, 3 );

  IIR_Filters[2].Y[0] = FIRST_IIR_CONSTANT*IIR_Filters[2].Y[1] - SECOND_IIR_CONSTANT*IIR_Filters[2].Y[2] + THIRD_IIR_CONSTANT*(adcData & 0xffff)*VOLTAGE_PER_BIT;

  IIR_Filters[2].Y[2] = IIR_Filters[2].Y[1];

  IIR_Filters[2].Y[1] = IIR_Filters[2].Y[0];

  adcData = Chip_ADC_GetDataReg( LPC_ADC, 4 );

  IIR_Filters[3].Y[0] = FIRST_IIR_CONSTANT*IIR_Filters[3].Y[1] - SECOND_IIR_CONSTANT*IIR_Filters[3].Y[2] + THIRD_IIR_CONSTANT*(adcData & 0xffff)*VOLTAGE_PER_BIT;

  IIR_Filters[3].Y[2] = IIR_Filters[3].Y[1];

  IIR_Filters[3].Y[1] = IIR_Filters[3].Y[0];

  adcData = Chip_ADC_GetDataReg( LPC_ADC, 7 );

  IIR_Filters[4].Y[0] = FIRST_IIR_CONSTANT*IIR_Filters[4].Y[1] - SECOND_IIR_CONSTANT*IIR_Filters[4].Y[2] + THIRD_IIR_CONSTANT*(adcData & 0xffff)*VOLTAGE_PER_BIT*ADC7_GAIN;

  IIR_Filters[4].Y[2] = IIR_Filters[4].Y[1];

  IIR_Filters[4].Y[1] = IIR_Filters[4].Y[0];

  Chip_ADC_ClearFlags( LPC_ADC, ADC_FLAGS_SEQA_INT_MASK );

  eTime = g_Timer.GetCurrentUsec() - sTime;

  total_time += eTime;
}

float GetADCVoltage( uint16_t channelIndex )
{
  if ( channelIndex  <= NUM_INT_AD_CHANNELS )
  {
    return( IIR_Filters[channelIndex].Y[0] );
  }

  return( 0.0f );
}

typedef void (*IAP) (uint32_t[], uint32_t[]);

__attribute__((section(".data"))) int SaveParameters( void )
{
  int i, IAPResult;
  IAP iapRoutine = (IAP) 0x03000205;
  uint32_t command[5], result[4];

  __disable_irq();

  command[0] = IAP_PREWRRITE_CMD;
  command[1] = 14;
  command[2] = 14;
  iapRoutine(command, result);

  IAPResult = result[0];

  do {
    command[0] = IAP_ERSSECTOR_CMD;
    command[1] = 14;
    command[2] = 14;
    command[3] = SystemCoreClock / 1000;
    iapRoutine(command, result);

    IAPResult = result[0];
  } while( IAPResult == IAP_BUSY );

  command[0] = IAP_PREWRRITE_CMD;
  command[1] = 14;
  command[2] = 14;
  iapRoutine(command, result);

  IAPResult = result[0];

  do {
    command[0] = IAP_WRISECTOR_CMD;
    command[1] = 0x70000;
    command[2] = (uint32_t) Parameters;
    command[3] = 256;
    command[4] = SystemCoreClock / 1000;
    iapRoutine(command, result);

    IAPResult =  result[0];
  } while( IAPResult == IAP_BUSY );

  __enable_irq();

  return IAPResult;
}

/*
    Name      : main

    Purpose   : This is the main routine for the software. It has two jobs : 1) Initialize everything and 2) process incoming samples.

    Argurments: It has no arguments.

    Return Value : It has no return value.
*/

int main(void)
{  int charPresent, j, last_sensor, sensor, didsensor = 1, lookbuttons, val32, displowbatt = 0, beepCounter = 0, total;
  uint32_t *storedParameters = (uint32_t *) 0x70000;
  uint16_t i , frequencyIndex = 1;
  uint64_t sTime, eTime;
  unsigned char output[140], input[80];
  SPI_CFGSETUP_T spiSetup;
  SPIM_DELAY_CONFIG_T spiDelay;

  memset( rxBuff, 0, sizeof( rxBuff ) );

  for( i = 0;i < 5;i++ )
  {
    IIR_Filters[i].Y[0] = 0.0;
    IIR_Filters[i].Y[1] = 0.0;
    IIR_Filters[i].Y[2] = 0.0;
  }

  DRDConfig = (CONFIG_HEADER_T *) &ConfigParameters;

  for( i = 0;i < 64;i++ )
  {
    Parameters[i] = storedParameters[i];
  }

  if ( Parameters[0] = 0x12345678 )
  {
    ConfigParameters.ValidCode = Parameters[0];
    ConfigParameters.Personality = Parameters[1];
    ConfigParameters.AmpsCalibration = Parameters[2]/1000.0;
    ConfigParameters.AmpsOffset = Parameters[3];
  }
  else
  {
    ConfigParameters.ValidCode = 0x12345678;
    ConfigParameters.Personality = DRDPersTable.PersID;
    ConfigParameters.AmpsCalibration = 1000.0;
    ConfigParameters.AmpsOffset = 0.0;
  }

  DRDState.LineMax = MAX_CHARACTERS;

  SystemCoreClockUpdate();

  Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_INPUTMUX);

  Board_Init();

  Kernel_Init();

  Chip_CRC_Init();

  Chip_PININT_Init(LPC_PININT);

  Chip_UART_Init( LPC_USART0 );

  Chip_ADC_Init( LPC_ADC, ADC_CR_RESOL(3) );

  Chip_ADC_SetClockRate( LPC_ADC, 2000000 );

  Chip_ADC_Calibration( LPC_ADC );

  Chip_ADC_SetupSequencer( LPC_ADC, ADC_SEQA_IDX, ADC_SEQ_CTRL_MODE_EOS | ADC_SEQ_CTRL_CHANSEL(0) | ADC_SEQ_CTRL_CHANSEL(1) | ADC_SEQ_CTRL_CHANSEL(3) | ADC_SEQ_CTRL_CHANSEL(4) | ADC_SEQ_CTRL_CHANSEL(7) | ADC_SEQ_CTRL_HWTRIG_POLPOS | ADC_SEQ_CTRL_TRIGGER(7) );

  Chip_ADC_EnableSequencer( LPC_ADC, ADC_SEQA_IDX );

  Chip_ADC_EnableInt( LPC_ADC, ADC_INTEN_SEQA_ENABLE );

  NVIC_EnableIRQ( ADC_SEQA_IRQn );

  Chip_GPIO_SetPinState( LPC_GPIO, 0,  2, false );
  Chip_GPIO_SetPinState( LPC_GPIO, 0,  3, false );
  Chip_GPIO_SetPinState( LPC_GPIO, 0,  4, false );
  Chip_GPIO_SetPinState( LPC_GPIO, 0,  7, false );
  Chip_GPIO_SetPinState( LPC_GPIO, 0,  8, false );
  Chip_GPIO_SetPinState( LPC_GPIO, 1,  5, false );
  Chip_GPIO_SetPinState( LPC_GPIO, 1,  8, false );
  Chip_GPIO_SetPinState( LPC_GPIO, 1, 10, true );

  Chip_GPIO_SetPinDIROutput( LPC_GPIO, 0,  2 );
  Chip_GPIO_SetPinDIROutput( LPC_GPIO, 0,  3 );
  Chip_GPIO_SetPinDIROutput( LPC_GPIO, 0,  4 );
  Chip_GPIO_SetPinDIROutput( LPC_GPIO, 0,  7 );
  Chip_GPIO_SetPinDIROutput( LPC_GPIO, 1,  5 );
  Chip_GPIO_SetPinDIROutput( LPC_GPIO, 1,  8 );
  Chip_GPIO_SetPinDIROutput( LPC_GPIO, 1, 10 );

  Chip_UART_ConfigData( LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1 );

  BaudRate.clk = Chip_Clock_GetMain_B_ClockRate();
  BaudRate.baud = 9600;
  BaudRate.ovr = 16;
  BaudRate.mul = 0;
  BaudRate.div = 0;

  Chip_Clock_SetUARTBaseClockRate( BaudRate.clk );

  Chip_UART_CalcBaud( LPC_USART0, &BaudRate );

  Chip_UART_Div( LPC_USART0, BaudRate.div, BaudRate.ovr );

  Chip_UART_Enable( LPC_USART0 );

  Chip_UART_TXEnable( LPC_USART0 );

  Chip_UART_IntEnable( LPC_USART0, UART_INTEN_RXRDY );

  NVIC_SetPriority( UART0_IRQn, 4 );

  NVIC_EnableIRQ( UART0_IRQn );

  Chip_DMA_Init(LPC_DMA);

  Chip_DMA_Enable(LPC_DMA);
  Chip_DMA_SetSRAMBase(LPC_DMA, DMA_ADDR(Chip_DMA_Table));

  ResMgr_EnterPLLMode();

  Chip_SPI_Init(LPC_SPI0);
  Chip_SPI_Init(LPC_SPI1);

  Chip_PININT_SetPinModeEdge( LPC_PININT, PININTCH0 );
  Chip_PININT_EnableIntLow( LPC_PININT, PININTCH0 );
  Chip_INMUX_PinIntSel( PININTSELECT0, 0, 20 );

  Chip_TIMER_Init( LPC_TIMER0 );
  Chip_TIMER_Reset( LPC_TIMER0 );
  Chip_TIMER_ResetOnMatchEnable( LPC_TIMER0, 1 );
  Chip_TIMER_SetMatch( LPC_TIMER0, 1, 1 );
  Chip_TIMER_ExtMatchControlSet( LPC_TIMER0, 0, TIMER_EXTMATCH_TOGGLE, 1 );
  LPC_TIMER0-> PWMC = 0;
        
  Chip_TIMER_Init( LPC_TIMER1 );
  Chip_TIMER_Reset( LPC_TIMER1 );
  Chip_TIMER_ResetOnMatchEnable( LPC_TIMER1, 3 );
  Chip_TIMER_SetMatch( LPC_TIMER1, 2, 239750 );
  Chip_TIMER_SetMatch( LPC_TIMER1, 3, 239999 );
  LPC_TIMER1-> PWMC = 1 << 2;

  NVIC_EnableIRQ( CT32B1_IRQn );
        
  Chip_TIMER_Init( LPC_TIMER2 );
  Chip_TIMER_Reset( LPC_TIMER2 );
  Chip_TIMER_ResetOnMatchEnable( LPC_TIMER2, 1 );
  Chip_TIMER_SetMatch( LPC_TIMER2, 1, 23437 );
  Chip_TIMER_ExtMatchControlSet( LPC_TIMER2, 0, TIMER_EXTMATCH_TOGGLE, 1 );
  LPC_TIMER2-> PWMC = 0;
  

  Chip_TIMER_Init( LPC_TIMER3 );
  Chip_TIMER_Reset( LPC_TIMER3 );
  Chip_TIMER_ResetOnMatchEnable( LPC_TIMER3, 3 );
  Chip_TIMER_SetMatch( LPC_TIMER3, 0, 575999);
  Chip_TIMER_SetMatch( LPC_TIMER3, 3, 639999 );
  LPC_TIMER3-> PWMC = 1;
  
  spiSetup.master = 1;
  spiSetup.lsbFirst = 0;
  spiSetup.mode = SPI_CLOCK_MODE0;
  Chip_SPI_ConfigureSPI(LPC_SPI0, &spiSetup);

  Chip_SPI_SetCSPolLow(LPC_SPI0, 0);

  Chip_SPI_SetXferSize(LPC_SPI0, 16);

  Chip_SPIM_SetClockRate(LPC_SPI0, 6000000);

  spiDelay.PreDelay = 1;

  Chip_SPIM_DelayConfig(LPC_SPI0, &spiDelay );

  Chip_SPI_Enable(LPC_SPI0);

  Chip_SPI_ConfigureSPI(LPC_SPI1, &spiSetup);

  Chip_SPI_SetCSPolLow(LPC_SPI1, 0);

  Chip_SPI_SetXferSize(LPC_SPI1, 8);

  Chip_SPIM_SetClockRate(LPC_SPI1, 6000000);

  Chip_SPI_Enable(LPC_SPI1);

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0) & ~SPI_TXDATCTL_EOT) | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x40);

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDAT = 0x00;

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x00);

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0) & ~SPI_TXDATCTL_EOT) | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x40);

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDAT = 0x05;

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x20);

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0) & ~SPI_TXDATCTL_EOT) | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0x40);

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDAT = 0x0A;

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );
  LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0xaa);

  while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_MSTIDLE) );

  LPC_SPI1->TXDATCTL = ((LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK) | SPI_TXDATCTL_DEASSERTNUM_SSEL(0)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(0);

  LCD_Init( &DRDState );

  SetupSPIDMA1();
        
  Chip_TIMER_Enable( LPC_TIMER0 );
  Chip_TIMER_Enable( LPC_TIMER3 );

  i = 0;

  Chip_MBOX_Init( LPC_MBOX );

  Chip_CPU_CM0Boot( m0plus, &M0PlusStack[1024] );

  DRDState.MenuState = MENU_TOP_LEVEL;

  InitializeFilters();

  for( i = 0;i < 50;i++ )
  {
    Chip_GPIO_SetPinState( LPC_GPIO, 1,  5, true );

    LPC_ADC->SEQ_CTRL[ADC_SEQA_IDX] |= ADC_SEQ_CTRL_START;

    while( !ADCInterrupt )
    {
      __WFI();
    }

    ADCInterrupt = 0;
  }

  ClearScreen();

  DRDState.PowerSaveEnable = 1;

  DoBatteryTest(&DRDState);

  DRDState.DRDPersonality = DRDPersTable.PersID;
  DRDState.Mode = DRDPersTable.InitMode;
  DRDState.SigType = DRDPersTable.InitSigType;

  PersonalityInit(&DRDState);
  CalculateNewSettings(&DRDState, 1);

  // Build initial LCD screen

  LCD_Clrscr( &DRDState );

  //  Check sensor against expected for the default type.

  sensor = SensorDetect();

  if ( sensor == 0 ) // Sensor detected.
  {
    LCD_Gotoxy( &DRDState, 1, 1 );

    if ( (DRDState.SensorType & SENSTYPE_RAILSENS) || (DRDState.SensorType & SENSTYPE_DIRECT) )
    {
      sprintf( lcdbuf, "NO SENSOR       " );
    }
    else if ( DRDState.SensorType & SENSTYPE_SHUNT )
    {
      sprintf( lcdbuf, "Check Shunt      " );
    }

    LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

    sprintf( output, "\r\n%s", lcdbuf );
    SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

    didsensor = 1;
  }
  else // No sensor detected.
  {
    if ( DRDState.SensorType == SENSTYPE_SHUNT )
    {
      i = ReadShuntType();

      // For BART peronality change the sigType based on the detected shunt.

      if ( DRDState.DRDPersonality == DRD_PERSONALITY_BART )
      {
        if ( i == SENSDET_SHORT )
        {
          DRDState.SigType = SIGTYPE_BARTSC;

          DRDState.SensorType = DRDPersTable.SensorType1 & 0x00FF;
          DRDState.SensDetect = DRDPersTable.SensorType1 & 0xFF00;
        }
        else if ( i == SENSDET_500OHM )
        {
          DRDState.SigType = SIGTYPE_BARTMV;
          DRDState.SensorType = DRDPersTable.SensorType2 & 0x00FF;
          DRDState.SensDetect = DRDPersTable.SensorType2 & 0xFF00;
        }
      }
    }
    else // Railsense or Direct.
    {
      i = CheckSensorMatch( &DRDState );

      sprintf( output, "\r\nSensor type%svalid",(i == 0) ? " in": " " );
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

      sensor = i;
    }
  }

  BuildBaseDisplay(  &DRDState, DRDState.Mode, 1, lcdbuf );
  LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

  sprintf( output, "\r\n%s", lcdbuf );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  BuildBaseDisplay( &DRDState, DRDState.Mode, 2, lcdbuf);
  LCD_Gotoxy( &DRDState, 1,2 );
  LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

  sprintf( output, "\r\n%s\r\n", lcdbuf );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  lookbuttons = 0;

  eTime = 0;

  while( !done )
  {
    __WFI();

    if ( !DMAInterrupt )
    {
      continue;
    }

    DMAInterrupt = 0;

    sTime = g_Timer.GetCurrentUsec();

    if ( !Chip_GPIO_GetPinState( LPC_GPIO, 0, 11 ) )
    {
      if ( !(DRDState.ButtonHistory[0] & BUTTON_FU) && (DRDState.ButtonHistory[1] & BUTTON_FU)  )
      {
        DRDState.ButtonEvent |= BUTTON_FU;
      }

      if ( DRDState.ButtonHistory[1] & BUTTON_FU )
      {
        DRDState.ButtonHistory[0] |= BUTTON_FU;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_FU;
      }

      DRDState.ButtonHistory[1] |= BUTTON_FU;
    }
    else
    {
      if ( DRDState.ButtonHistory[1] & BUTTON_FU )
      {
        DRDState.ButtonHistory[0] |= BUTTON_FU;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_FU;
      }

      DRDState.ButtonHistory[1] &= ~BUTTON_FU;
    }

    if ( !Chip_GPIO_GetPinState( LPC_GPIO, 0, 12 ) )
    {
      if ( !(DRDState.ButtonHistory[0] & BUTTON_FD) && (DRDState.ButtonHistory[1] & BUTTON_FD) )
      {
        DRDState.ButtonEvent |= BUTTON_FD;
      }

      if ( DRDState.ButtonHistory[1] & BUTTON_FD )
      {
        DRDState.ButtonHistory[0] |= BUTTON_FD;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_FD;
      }

      DRDState.ButtonHistory[1] |= BUTTON_FD;
    }
    else
    {
      if ( DRDState.ButtonHistory[1] & BUTTON_FD )
      {
        DRDState.ButtonHistory[0] |= BUTTON_FD;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_FD;
      }

      DRDState.ButtonHistory[1] &= ~BUTTON_FD;
    }

    if ( !Chip_GPIO_GetPinState( LPC_GPIO, 0, 5 ) )
    {
      if ( !(DRDState.ButtonHistory[0] & BUTTON_TYPE) && (DRDState.ButtonHistory[1] & BUTTON_TYPE) )
      {
        DRDState.ButtonEvent |= BUTTON_TYPE;
      }

      if ( DRDState.ButtonHistory[1] & BUTTON_TYPE )
      {
        DRDState.ButtonHistory[0] |= BUTTON_TYPE;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_TYPE;
      }

      DRDState.ButtonHistory[1] |= BUTTON_TYPE;
    }
    else
    {
      if ( DRDState.ButtonHistory[1] & BUTTON_TYPE )
      {
        DRDState.ButtonHistory[0] |= BUTTON_TYPE;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_TYPE;
      }

      DRDState.ButtonHistory[1] &= ~BUTTON_TYPE;
    }

    if ( !Chip_GPIO_GetPinState( LPC_GPIO, 0, 25 ) )
    {
      if ( !(DRDState.ButtonHistory[0] & BUTTON_MODE) && (DRDState.ButtonHistory[1] & BUTTON_MODE) )
      {
        DRDState.ButtonEvent |= BUTTON_MODE;
      }

      if ( DRDState.ButtonHistory[1] & BUTTON_MODE )
      {
        DRDState.ButtonHistory[0] |= BUTTON_MODE;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_MODE;
      }

      DRDState.ButtonHistory[1] |= BUTTON_MODE;
    }
    else
    {
      if ( DRDState.ButtonHistory[1] & BUTTON_MODE )
      {
        DRDState.ButtonHistory[0] |= BUTTON_MODE;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_MODE;
      }

      DRDState.ButtonHistory[1] &= ~BUTTON_MODE;
    }

    if ( !Chip_GPIO_GetPinState( LPC_GPIO, 0, 6 ) )
    {
      if ( !(DRDState.ButtonHistory[0] & BUTTON_BL) && (DRDState.ButtonHistory[1] & BUTTON_BL) )
      {
        DRDState.ButtonEvent |= BUTTON_BL;
      }

      if ( DRDState.ButtonHistory[1] & BUTTON_BL )
      {
        DRDState.ButtonHistory[0] |= BUTTON_BL;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_BL;
      }

      DRDState.ButtonHistory[1] |= BUTTON_BL;
    }
    else
    {
      if ( DRDState.ButtonHistory[1] & BUTTON_BL )
      {
        DRDState.ButtonHistory[0] |= BUTTON_BL;
      }
      else
      {
        DRDState.ButtonHistory[0] &= ~BUTTON_BL;
      }

      DRDState.ButtonHistory[1] &= ~BUTTON_BL;
    }

    if ( loop_count == 10 )
    {
      eTime = sTime;
    }

    if ( loop_count >= 110 )
    {
      eTime = 0;
    }

    loop_count += 1;

    if ( beepCounter > 0 )
    {
      beepCounter -= 1;

      if ( beepCounter <= 0 )
      {
        Beeper( 0 );
      }
    }


    if ( loop_count && !(loop_count  % 10) )
    {
      DRDState.TenthSecondsCounter += 1;

      if ( DRDState.TenthSecondsCounter && !(DRDState.TenthSecondsCounter % 10) )
      {
        last_sensor = sensor;
        sensor = SensorDetect();

        if ( sensor )
        {
          sensor = CheckSensorMatch( &DRDState );
        }

        DRDState.SecondsCounter += 1;

        if ( (last_sensor != sensor) && (sensor == 1) )
        {
          DRDState.SecondsCounter = 0;

          sprintf(output, "\r\nSensor Detect changed");

          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          i = ReadShuntType();

          if ( DRDState.DRDPersonality == DRD_PERSONALITY_BART )
	  {
            if ( i == SENSDET_SHORT )
            {
              DRDState.SigType = SIGTYPE_BARTSC;
              DRDState.SensorType = DRDPersTable.SensorType1 & 0x00FF;
              DRDState.SensDetect = DRDPersTable.SensorType1 & 0xFF00;
            }
            else if ( i == SENSDET_500OHM )
            {
              DRDState.SigType = SIGTYPE_BARTMV;
              DRDState.SensorType = DRDPersTable.SensorType2 & 0x00FF;
              DRDState.SensDetect = DRDPersTable.SensorType2 & 0xFF00;
            }
	  }

          PersonalityInit( &DRDState );                  // new signalling type, must reinit everything
          CalculateNewSettings( &DRDState, 1 );

          sprintf(output, " : SensType=%ld  SensDet=%ld\r\n", DRDState.SensorType, DRDState.SensDetect >> 8);

          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
        }

        if ( sensor == 0 )
        {
          if ( ( DRDState.SensorType & SENSTYPE_RAILSENS) || ( DRDState.SensorType & SENSTYPE_DIRECT) )
	  {
            sprintf( lcdbuf, "NO SENSOR       ");
	  }
          else if ( DRDState.SensorType & SENSTYPE_SHUNT )
	  {
            sprintf(lcdbuf, "Check Shunt      ");
	  }

          LCD_Gotoxy( &DRDState, 1, 1 );
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
          sprintf(output, "\r\n%s", lcdbuf);

          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          didsensor = 1;
        }
        else if ( DRDState.Mode == MODE_MEAS )
        {
          if ( didsensor )
          {
            didsensor = 0;
            BuildBaseDisplay( &DRDState, DRDState.Mode, 1, lcdbuf );
            LCD_Gotoxy( &DRDState, 1, 1 );
            LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

            BuildBaseDisplay( &DRDState, DRDState.Mode, 2, lcdbuf );
            LCD_Gotoxy( &DRDState, 1, 2 );
            LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
	  }

          val32 = DRDState.PeakAvg;                // get the present peak average from the sensor

          LCD_Gotoxy( &DRDState, 1, 1 );
          GetCorrectAmpsForDisplay( &DRDState, lcdbuf, val32 ); // obtain the correct AMPS value and units for display (applies calibration)
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );                        // send AMPS to display

          LCD_Gotoxy( &DRDState, 1, 2 );
          GetFreqForDisplay( &DRDState, lcdbuf );
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
          LCD_Gotoxy( &DRDState, 1, 1 );
        }
        else
        {
          if ( didsensor )
          {
            didsensor = 0;
            BuildBaseDisplay( &DRDState, DRDState.Mode, 1, lcdbuf );
            LCD_Gotoxy( &DRDState, 1, 1 );
            LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
          }

          GetCorrectCodeForDisplay( &DRDState, lcdbuf );        // obtain correct code for display
          LCD_Gotoxy( &DRDState, 1, 1 );
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );                        // send the CODE to the LCD display

          if ( DRDState.DRDPersonality == DRD_PERSONALITY_DRD_RET )
          {
            GetCorrectDutyForDisplay( &DRDState, lcdbuf );        // obtain correct duty cycle for display
          }
          else
          {
            GetFreqForDisplay( &DRDState, lcdbuf );
          }

          LCD_Gotoxy( &DRDState, 1, 2 );
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );                        // send the DUTY CYCLE to the LCD display
        }
      }

      if ( displowbatt && !(loop_count % 200) )
      {
        displowbatt = 0;
      }

      if ( !(DRDState.TenthSecondsCounter % 5) )
      {
        if ( DRDState.Calibrating )
        {
          input[0] = 0;

          Calib_DoCalibration( &DRDState, input, 0 );
        }

        DRDState.BattSampling = GetADCVoltage( ADC_BAT_MONV );

        if ( DRDState.BattSampling < LOW_BATTERY_THRESHOLD )    // at less than 4.000V power down
        {
          LCD_Gotoxy( &DRDState, 1, 1 );
          sprintf( lcdbuf, "BATTERY TOO LOW " );
          sprintf(output, "\r\n%s", lcdbuf );

          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
          LCD_Gotoxy( &DRDState, 1, 2 );
          sprintf( lcdbuf, "POWERING DOWN..." );
          sprintf( output, "\r\n%s", lcdbuf );
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
          PowerDown();
        }
        else if ( (DRDState.BattSampling < LOW_BATTERY_WARNING) && !displowbatt )
        {
          displowbatt = 1;
          LCD_Gotoxy( &DRDState, 1, 2 );
          sprintf( lcdbuf, "%s", DRDPersTable.LoBatMsg);
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
#ifdef _TEST_
          total = (int) loop_count;

          sprintf(output, "\r\n%s %d", lcdbuf, total);

          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
#endif
        }
      }
    }

    if ( lookbuttons >= 5 )
    {
      lookbuttons = 0;

      // if a button event has occurred, see which button and handle it
      if ( DRDState.ButtonEvent )
      {
        // Check for FREQUENCY_UP button pressed
        if ( DRDState.ButtonEvent & BUTTON_FU )
        {
          // to be really super safe, stop the backlight timer and
          // the main interrupt timer and disable interrupts while
          // processing a button event
	        
          DRDState.ButtonEvent &= (~BUTTON_FU);                // clear this specific button even flag

          if ( !DRDState.PresFreqLock )
          {
            DRDState.PresFreqIdx++;                              // inc the frequency table index

            if ( DRDState.PresFreqIdx >= DRDState.NumFreqs )     // check if need to wrap back to index 0
            {
              DRDState.PresFreqIdx = 0;
            }

            CalculateNewSettings( &DRDState, 1 );                              // re-calculate all frequency dependent variables
          }

          GetFreqForDisplay( &DRDState, lcdbuf );                         // update display with new frequency
          LCD_Gotoxy( &DRDState, 1, 2 );
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

          DRDState.SecondsCounter = 0;                         // reset the seconds count so power save will not kick in

          Beeper( 1 );

          beepCounter = BEEP_COUNT;
        }

        // Check for FREQUENCY_DOWN button pressed
	      
        if ( DRDState.ButtonEvent & BUTTON_FD )
        {
          DRDState.ButtonEvent &= (~BUTTON_FD);                // clear this specific button even flag
            
          // decrement frequency and correct for wrap-around
		      
          if ( !DRDState.PresFreqLock )
          {
            if ( DRDState.PresFreqIdx > 0 )
            {
              DRDState.PresFreqIdx--;
            }
            else
            {
              DRDState.PresFreqIdx = DRDState.NumFreqs - 1;
            }

            CalculateNewSettings( &DRDState, 1 );                              // re-calculate all frequency dependent variables
          }

          GetFreqForDisplay( &DRDState, lcdbuf );                         // update display with new frequency
          LCD_Gotoxy( &DRDState, 1, 2 );
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

          DRDState.SecondsCounter = 0;                         // reset the seconds count so power save will not kick in

          Beeper( 1 );

          beepCounter = BEEP_COUNT;
        }

        // check for TYPE button pressed (type of signalling if more than 1)
		  
        if ( DRDState.ButtonEvent & BUTTON_TYPE )
        {
          DRDState.ButtonEvent &= (~BUTTON_TYPE);              // clear this specific button event flag

          i = DRDState.SigType;

          // Ignore Type button on BART DRD type (cable sense automatically determines SigType)
		      
          if ( DRDPersTable.PersID != DRD_PERSONALITY_BART)
          {
            // up to three signalling types are allowed for a particular personality
            // get the correct next one

            if ( DRDPersTable.SigType1 == DRDState.SigType)
            {
              if ( DRDPersTable.SigType2 != SIGTYPE_NONE)
              {
                DRDState.SigType = DRDPersTable.SigType2;
              }
            }
            else if ( DRDPersTable.SigType2 == DRDState.SigType)
            {
              if ( DRDPersTable.SigType3 != SIGTYPE_NONE)
              {
                DRDState.SigType = DRDPersTable.SigType3;
              }
              else
              {
                DRDState.SigType = DRDPersTable.SigType1;
              }
            }
            else if ( DRDPersTable.SigType3 == DRDState.SigType )
            {
              DRDState.SigType = DRDPersTable.SigType1;
            }
          }

          if ( i != DRDState.SigType )
          {
            last_sensor = 1;
            sensor = 0;

            sprintf(output, "\r\nNew signalling type");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            PersonalityInit( &DRDState );                          // new signalling type, must re-init everything
            CalculateNewSettings( &DRDState, 1 );                     // re-calculate all frequency dependent variables

            // must re-build entire display

            BuildBaseDisplay(  &DRDState, DRDState.Mode, 1, lcdbuf );
            LCD_Clrscr( &DRDState );
            LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

            BuildBaseDisplay( &DRDState, DRDState.Mode, 2, lcdbuf );
            LCD_Gotoxy( &DRDState, 1, 2 );
            LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf)  );
          }

          DRDState.SecondsCounter = 0;                         // reset the seconds count so power save will not kick in

          Beeper( 1 );

          beepCounter = BEEP_COUNT;
        } // end if(gButtonEvent & BUTTON_TYPE)

        // check for MODE button pressed (mode is current or code)

        if ( DRDState.ButtonEvent & BUTTON_MODE )
        {
          DRDState.ButtonEvent &= (~BUTTON_MODE);              // clear this specific button even flag

          // the only valid modes are MEAS and CODE, select the opposite of current mode

          if ( DRDState.Mode == MODE_MEAS )
          {
            DRDState.Mode = MODE_CODE;
          }
          else // gMode == MODE_CODE
          {
            DRDState.Mode = MODE_MEAS;
          }

// %%%    CalculateNewSettings( &DRDState, 0 );                     // re-calculate all frequency dependent variables

          // must re-build entire display

          BuildBaseDisplay( &DRDState, DRDState.Mode, 1, lcdbuf );
          LCD_Clrscr(&DRDState);
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

          BuildBaseDisplay( &DRDState, DRDState.Mode, 2, lcdbuf );
          LCD_Gotoxy( &DRDState, 1, 2 );
          LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

          DRDState.SecondsCounter = 0;                         // reset the seconds count so power save will not kick in

          Beeper( 1 );

          beepCounter = BEEP_COUNT;
        } // end if(gButtonEvent & BUTTON_MODE)

        // check for BACKLIGHT button pressed

        if ( DRDState.ButtonEvent & BUTTON_BL )
        {
          DRDState.ButtonEvent &= (~BUTTON_BL);                // clear this specific button even flag

          // increment backlight duty cycle in steps of 10%
          // to 100 then wrap back to 0

          if ( DRDState.BLightDuty < 100 )
          {
            DRDState.BLightDuty += 10;
	  }
          else
	  {
            DRDState.BLightDuty = 0;
	  }
        
          Backlight_SetDuty( DRDState.BLightDuty );
        
          DRDState.SecondsCounter = 0;                         // reset the seconds count so power save will not kick in
          Beeper( 1 );

          beepCounter = BEEP_COUNT;
        }
      }

      // check if a serial port char is available
      // this is factory only code, will never occur in
      // the field because no serial port is connected

      input[0] = 0;

      charPresent = GetChar( &USART0ReceiveRingBuffer, input );

      MenuRoutine( &DRDState, input, charPresent );

      // if the backlight is on and no buttons were pressed
      // for 180 secs then kill the backlight to save power

      if ( (DRDState.BLightDuty != 0) && (DRDState.SecondsCounter >= POWERSAVE_STAGE1_TIME) && (DRDState.PowerSaveEnable == 1) )
      {
        DRDState.BLightDuty = 0;
        Backlight_SetDuty(DRDState.BLightDuty);
      }

      // if no buttons were pressed for 240 seconds (or 3600
      // seconds if power save disabled,) kill app.

      if ( DRDState.PowerSaveEnable != 2 )
      {
        if ( (DRDState.SecondsCounter >= NOPOWERSAVE_TIME) || 
             ((DRDState.SecondsCounter >= POWERSAVE_STAGE2_TIME) && (DRDState.PowerSaveEnable == 1)) )
        {
          LCD_Clrscr( &DRDState );

          LCD_Gotoxy(  &DRDState, 1, 1 );
          LCD_Puts( &LCDTransmitRingBuffer, "POWERING DOWN,", 14 );
          LCD_Gotoxy( &DRDState, 1, 2 );
          LCD_Puts( &LCDTransmitRingBuffer, " BYE BYE", 8 );
	  PowerDown();
        }
      }
    } //  end if(lookbuttons >= 5)

    // Demodulation processing
	
    switch( DRDState.DRDPersonality )
    {
      case DRD_PERSONALITY_BART:
        BART_Decoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE );
      break;

      case DRD_PERSONALITY_DRD11:
        DRD11_Decoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE );
      break;

      case DRD_PERSONALITY_DRD_ROW:
        if ( DRDState.SigType == SIGTYPE_DRD_ROW_AFTC )
        {
          DRD_AFTCDecoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE) ;
        }
        else // (gSigType == SIGTYPE_DRD_ROW_CAB)
        {
          DRD_ROW_CABDecoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE) ;
        }
      break;

      case DRD_PERSONALITY_DRD_MFOR:
        if ( DRDState.SigType == SIGTYPE_DRD_MFOR_AFTC )
        {
          DRD_AFTCDecoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE) ;
        }
        else // (gSigType == SIGTYPE_DRD_MFOR_CAB)
        {
          DRD_MFOR_CABDecoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE) ;
        }
      break;

      case DRD_PERSONALITY_DRD_STL:
        if ( DRDState.SigType == SIGTYPE_DRD_STL_AFTC )
        {
          DRD_AFTCDecoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE) ;
        }
        else // (gSigType == SIGTYPE_DRD_STL_CAB)
        {
          DRD_STL_CABDecoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE) ;
        }
      break;

      case DRD_PERSONALITY_DRD_RET:
        if ( DRDState.SigType == SIGTYPE_DRD_RET_AFTC )
        {
          DRD_AFTCDecoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE) ;
        }
        else // (gSigType == SIGTYPE_DRD_STL_CAB)
        {
          DRD_RET_CABDecoder( &DRDState, rxBuff[currentBuffer ^ 1], TRANSFER_SIZE) ;
        }
      break;
    } // switch(gDRDPersonality) 

    lookbuttons++;
  }

  ResMgr_EnterNormalMode();

  Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_INPUTMUX);
}
