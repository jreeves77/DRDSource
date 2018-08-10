#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensorhub.h"
#include "drdstate.h"

static char output[80];
static char lcdbuf[20];

extern struct ring_buffer USART0TransmitRingBuffer;

void LowLevelTests( struct _drd_state_ *state, unsigned char *point, int charPresent );

bool IsNumeric( char *point, int size )
{
  int i;
  bool RetValue = true, dpoint = false;

  for( i = 0;point[i] && (i < size);i++ )
  {
    if ( (point[i] < '0') && (point[0] > '9') && (point[i] == '.') )
    {
      RetValue = false;
      break;
    }

    if ( point[i] == '.' )
    {
      if ( dpoint == false )
      {
        dpoint = true;
      }
      else
      {
        RetValue = false;
        break;
      }
    }
  }

  return( RetValue );
}

void ClearScreen( void )
{
  sprintf(output, "\x1b[2J");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
}

void PrintMainMenu( struct _drd_state_ *state )
{
  sprintf(output, "\r\n\nMain Menu\r\n");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 1) Erase Config");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 2) Perform Calibration");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 3) Change calibration [now %5.4f]", state -> PresAmpsGainCal);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 4) Change offset      [now %d]", state -> PresAmpsOffsetCal);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 5) Change threshold   [now %d]", state -> DemodThreshold);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 6) Low Level Tests");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n\nEnter Selection : ");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
}

void MenuRoutine( struct _drd_state_ *state, unsigned char *point, int charPresent )
{
  uint32_t val32;

  if ( charPresent )
  {
    if ( (point[0] & 0xf0) || (point[0] == '\r') )
    {
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, point, 1 );

      state -> LineString[state -> LinePosition] = point[0];

      if ( state -> LinePosition < MAX_CHARACTERS )
      {
        state -> LinePosition += 1;
      }
    }
    
    if ( point[0] != '\r' )
    {
      return;
    }
    else
    {
      state -> LineSize = state -> LinePosition;

      state -> LinePosition = 0;
    }

    switch( state -> MenuState )
    {
      case MENU_TOP_LEVEL :
        state -> LineString[0] &= 0xdf;

        if ( (state -> LineString[0] != '\r') && (state -> LineString[1] != '\r') ) 
        {
          return;
        }

        switch( state -> LineString[0] )
        {
          case 'U' :
            state -> ButtonEvent |= BUTTON_FU;
	  break;

	  case 'D' :
            state -> ButtonEvent |= BUTTON_FD;
	  break;

	  case 'T' :
            state -> ButtonEvent |= BUTTON_TYPE;
	  break;

	  case 'M' :
            state -> ButtonEvent = BUTTON_MODE;
	  break;

	  case 'B' :
            state -> ButtonEvent = BUTTON_BL;
	  break;

	  case '\r' :
            ClearScreen();

            PrintMainMenu( state );

            state -> MenuState = MENU_MAIN_INPUT_LEVEL;
	  break;

	  default :
	  break;
        }
      break;

      case MENU_MAIN_INPUT_LEVEL :
        if ( (state -> LineString[0] & 0xdf) == 'Q' )
        {
          ClearScreen();

          state -> MenuState = MENU_TOP_LEVEL;

          state -> SecondsCounter = 0;                         // reset the seconds count so power save will not kick in

          CalculateNewSettings( state, 1 );
        }
        else
        {
          switch( state -> LineString[0] )
	  {
            case '1' :
              ClearScreen();

              sprintf(output, "\r\n\nThis function has been deprecated.\r\n");

              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

              PrintMainMenu( state );
	    break;

	    case '2' :
	      state -> MenuState = MENU_CALIB_LEVEL;

              state -> CalibMenuLevel = TOP_CALIB_MENU_LEVEL;

              Calib_DoCalibration( state, state -> LineString, charPresent );
	    break;

	    case '3' :
              sprintf(output, "\r\nEnter new amps calibration : ");

              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	      state -> MenuState = MENU_GAIN_CAL_LEVEL;

	      state -> LineMax = 9;

	      state -> LineSize = 0;

	      state -> LineString[0] = 0;
	    break;

	    case '4' :
              sprintf(output, "\r\nEnter new amps offset : ");

              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	      state -> MenuState = MENU_AMPS_OFFSET_LEVEL;

	      state -> LineMax = 9;

	      state -> LineSize = 0;

	      state -> LineString[0] = 0;
	    break;

	    case '5' :
              sprintf(output, "\r\nEnter new threshold : ");

              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	      state -> MenuState = MENU_THRESHOLD_LEVEL;

	      state -> LineMax = 9;

	      state -> LineSize = 0;

	      state -> LineString[0] = 0;
	    break;

	    case '6' :
              state -> MenuState = MENU_TESTS_LEVEL;
              state -> TestsMenuLevel = TESTS_TOP_MENU_LEVEL;

              LowLevelTests( state, state -> LineString, charPresent );
	    break;

            default:
            break;
	  }
        }
      break;

      case MENU_TESTS_LEVEL :
        LowLevelTests( state, state -> LineString, charPresent );
        break;

      case MENU_GAIN_CAL_LEVEL :
        if ( state -> LineSize < state -> LineMax )
        {
          if ( IsNumeric( state -> LineString, state -> LineSize - 1 ) )
          {
            state -> LineString[state -> LineSize - 1] = 0;

	    state -> PresAmpsGainCal = atof( state -> LineString );

            ClearScreen();

            PrintMainMenu( state );

            state -> MenuState = MENU_MAIN_INPUT_LEVEL;
          }
          else
          {
	    sprintf( output, "\r\nInvalid Entry." );
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            sprintf(output, "\r\nEnter new amps calibration (x/10000) : ");

            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	    state -> LineMax = 9;

	    state -> LineSize = 0;

	    state -> LineString[0] = 0;
          }
        }
        else
        {
	  sprintf( output, "\r\nInvalid Entry." );
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nEnter new amps calibration (x/10000) : ");

          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	  state -> LineMax = 9;

	  state -> LineSize = 0;

	  state -> LineString[0] = 0;
        }
      break;

      case MENU_AMPS_OFFSET_LEVEL :
        if ( state -> LineSize < state -> LineMax )
        {
          if ( IsNumeric( state -> LineString, state -> LineSize - 1 ) )
          {
            state -> LineString[state -> LineSize - 1] = 0;

	    state -> PresAmpsOffsetCal = atoi( state -> LineString );

            ClearScreen();

            PrintMainMenu( state );

            state -> MenuState = MENU_MAIN_INPUT_LEVEL;
          }
          else
          {
	    sprintf( output, "\r\nInvalid Entry." );
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            sprintf(output, "\r\nEnter new amps offset : ");

            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	    state -> LineMax = 9;

	    state -> LineSize = 0;

	    state -> LineString[0] = 0;
          }
        }
        else
        {
	  sprintf( output, "\r\nInvalid Entry." );
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nEnter new amps offset : ");

          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	  state -> LineMax = 9;

	  state -> LineSize = 0;

	  state -> LineString[0] = 0;
        }
      break;

      case MENU_THRESHOLD_LEVEL :
        if ( state -> LineSize < state -> LineMax )
        {
          if ( IsNumeric( state -> LineString, state -> LineSize - 1 ) )
          {
            state -> LineString[state -> LineSize - 1] = 0;

	    state -> DemodThreshold = atoi( state -> LineString );

            ClearScreen();

            PrintMainMenu( state );

            state -> MenuState = MENU_MAIN_INPUT_LEVEL;
          }
          else
          {
	    sprintf( output, "\r\nInvalid Entry." );
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            sprintf(output, "\r\nEnter new threshold : ");

            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	    state -> LineMax = 9;

	    state -> LineSize = 0;

	    state -> LineString[0] = 0;
          }
        }
        else
        {
	  sprintf( output, "\r\nInvalid Entry." );
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nEnter new threshold : ");

          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	  state -> LineMax = 9;

	  state -> LineSize = 0;

	  state -> LineString[0] = 0;
        }
      break;

      case MENU_CALIB_LEVEL :
        Calib_DoCalibration( state, state -> LineString, charPresent );
      break;

      default :
      break;
    }
  }
}
