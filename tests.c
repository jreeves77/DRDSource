#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sensorhub.h"
#include "drdstate.h"

static char output[80];

void ClearScreen( void );
void LCD_Init( struct _drd_state_ *state );
void PrintMainMenu( struct _drd_state_ *state );

extern struct ring_buffer USART0TransmitRingBuffer;
extern struct lcd_ring_buffer LCDTransmitRingBuffer;

void PrintTestsMenu( void )
{
  ClearScreen();

  sprintf( output, "\r\nLow Level Tests Menu\r\n" );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

  sprintf( output, "\r\n 1) LCD Test" );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

  sprintf( output, "\r\n 2) LCD Backlight" );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

  sprintf( output, "\r\n 3) Beeper" );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

  sprintf( output, "\r\n 4) Gain Pot" );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

  sprintf( output, "\r\n 5) Power Save Disable" );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

  sprintf( output, "\r\n 6) Software Power Down" );
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

  sprintf( output, "\r\n\nEnter Selection : ");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );
}

void LowLevelTests( struct _drd_state_ *state, unsigned char *point, int charPresent )
{
  int i;

  switch( state -> TestsMenuLevel )
  {
  case TESTS_TOP_MENU_LEVEL :
    PrintTestsMenu();
    state -> TestsMenuLevel = TESTS_MAIN_INPUT_MENU_LEVEL;

  case TESTS_MAIN_INPUT_MENU_LEVEL :

    if ( charPresent && ((point[0] & 0xdf) == 'Q') )
    {
      ClearScreen();

      state -> MenuState = MENU_MAIN_INPUT_LEVEL;

      state -> SecondsCounter = 0;

      PrintMainMenu( state );
    }
    else
    {
      if ( charPresent )
      {
        if ( point[1] != '\r' )
        {
          PrintTestsMenu();

          state -> TestsMenuLevel = TESTS_MAIN_INPUT_MENU_LEVEL;

          return;
        }

        switch( point[0] )
        {
        case '1' :
          LCD_Init( state );

          LCD_Gotoxy(state, 1, 1);
          LCD_Puts( &LCDTransmitRingBuffer, "0123456789012345", 16);

          LCD_Gotoxy( state, 1, 2);
          LCD_Puts( &LCDTransmitRingBuffer, "ABCDEFGHIJKLMNOP", 16);

          PrintTestsMenu();
          break;

        case '2' :
          sprintf(output, "\r\nEnter Duty (0,10,20...,90,100) : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

	  state -> LineMax = 9;

	  state -> LineSize = 0;

	  state -> LineString[0] = 0;

	  state -> TestsMenuLevel = TESTS_INPUT1_MENU_LEVEL;
          break;

        case '3' :
          sprintf(output, "\r\nBeeper - Enter 0=OFF  1=ON : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

	  state -> LineMax = 3;

	  state -> LineSize = 0;

	  state -> LineString[0] = 0;

	  state -> TestsMenuLevel = TESTS_INPUT2_MENU_LEVEL;
          break;

        case '4' :
          sprintf(output, "\r\nGain Pot - Enter Setting ( 0 -> 255) : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

	  state -> LineMax = 5;

	  state -> LineSize = 0;

	  state -> LineString[0] = 0;

	  state -> TestsMenuLevel = TESTS_INPUT3_MENU_LEVEL;
          break;

        case '5' :
          state -> PowerSaveEnable = 2;

          PrintTestsMenu();
          break;

        case '6' :
          PowerDown();

        default :
          break;
        }
      }
    }
    break;

  case TESTS_INPUT1_MENU_LEVEL :
    if ( state -> LineSize < state -> LineMax )
    {
      state -> LineString[state -> LineSize - 1] = 0;

      i = atoi( state -> LineString );

      if ( i < 0 )
      {
        i = 0;
      }

      if ( i > 100 )
      {
        i = 100;
      }

      Backlight_SetDuty( i );

      PrintTestsMenu();

      state -> TestsMenuLevel = TESTS_MAIN_INPUT_MENU_LEVEL;
    }
    else
    {
      sprintf( output, "\r\nInvalid Entry.\r\n" );
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

      sprintf(output, "\r\nBeeper - Enter 0=OFF  1=ON : ");
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

      state -> LineMax = 2;

      state -> LineSize = 0;

      state -> LineString[0] = 0;
    }
    break;

  case TESTS_INPUT2_MENU_LEVEL :
    if ( state -> LineSize < state -> LineMax )
    {
      state -> LineString[state -> LineSize - 1] = 0;

      if ( state -> LineString[0] == '0' )
      {
        Beeper( 0 );
      }
      else
      {
        Beeper( 1 );
      }

      PrintTestsMenu();

      state -> TestsMenuLevel = TESTS_MAIN_INPUT_MENU_LEVEL;
    }
    else
    {
      sprintf( output, "\r\nInvalid Entry.\r\n" );
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

      sprintf(output, "\r\nBeeper - Enter 0=OFF  1=ON : ");
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

      state -> LineMax = 3;

      state -> LineSize = 0;

      state -> LineString[0] = 0;
    }
    break;

  case TESTS_INPUT3_MENU_LEVEL :
    if ( state -> LineSize < state -> LineMax )
    {
      state -> LineString[state -> LineSize - 1] = 0;

      i = atoi( state -> LineString );

      if ( i < 0 )
      {
        i = 0;
      }

      if ( i > 255 )
      {
        i = 255;
      }

      LPC_SPI1->TXDATCTL = (LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK & ~SPI_TXDATCTL_DEASSERTNUM_SSEL(1)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(i);
      while(!(Chip_SPI_GetStatus( LPC_SPI1 ) & SPI_STAT_TXRDY) );

      LPC_SPI1->TXDATCTL = ((LPC_SPI1->TXCTRL & SPI_TXDATCTL_CTRLMASK) | SPI_TXDATCTL_DEASSERTNUM_SSEL(1)) | SPI_TXDATCTL_EOT | SPI_TXDATCTL_RXIGNORE | SPI_TXDATCTL_DATA(i);

      PrintTestsMenu();

      state -> TestsMenuLevel = TESTS_MAIN_INPUT_MENU_LEVEL;
    }
    else
    {
      sprintf( output, "\r\nInvalid Entry.\r\n" );
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

      sprintf(output, "\r\nGain Pot - Enter Setting ( 0 -> 255) : ");
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen( output ) );

      state -> LineMax = 5;

      state -> LineSize = 0;

      state -> LineString[0] = 0;
    }
    break;

  default :
    PrintTestsMenu();

    state -> TestsMenuLevel = TESTS_MAIN_INPUT_MENU_LEVEL;
    break;
  }
}
