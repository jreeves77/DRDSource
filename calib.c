#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sensorhub.h"
#include "drdstate.h"

static char       output[80];
static char       dispbuf[MAX_NUM_FREQS];
static char       ampsunits[10];
static int        numfreqs;
static float32_t  val32f;
static float32_t  actualamps;
static int        changes;

extern CONFIG_HEADER_T *DRDConfig;
extern PERS_TAB_T DRDPersTable;
extern struct ring_buffer USART0TransmitRingBuffer;
extern struct lcd_ring_buffer LCDTransmitRingBuffer;
extern unsigned int Parameters[64];

void ClearScreen( void );

void PrintMainMenu( struct _drd_state_ *state );
bool IsNumeric( char *point, int size );

void PrintCalibMenu( struct _drd_state_ *state )
{
  GetFreqForDisplay(state, dispbuf);
      // clrscr();
  sprintf(output, "\r\n\nCalibration Menu\r\n");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 1) Change Frequency [now= %s]", dispbuf);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 2) Do %s Offset Calibration", dispbuf);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 3) Test %s Offset Calibration", dispbuf);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 4) Do %s Amps Calibration", dispbuf);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 5) Test %s Amps Calibration", dispbuf);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n 6) Save ALL Calibrations to FLASH");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  sprintf(output, "\r\n\nEnter Selection : ");
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
}

/*******************************************************************************
*
* Function: Calib_DoCalibration
*
* Summary:  This routine runs the calibration menus.
*
*******************************************************************************/

void Calib_DoCalibration( struct _drd_state_ *state, unsigned char *point, int charPresent )
{
  CONFIG_HEADER_T NewConfig;
  char       a[MAX_NUM_FREQS];
  char       b[MAX_NUM_FREQS];
  char       c1;
  char       c2;
  char       c3;
  uint16_t   modereg;
  int        sel;
  int        i;
  int        j;
  int        c;
  int        status;
  int        freqoffset;
  uint32_t   rem32;
  float32_t  targetamps;
  uint32_t  val32;

  //UINT16  val16;
  //int     chan;
  //int     old_ipl;
  //INT32   ival32;
  //INT32   freqoff;


  switch( state -> CalibMenuLevel )
  {
    case TOP_CALIB_MENU_LEVEL :
      // Get the config stored in FLASH into a local structure

      NewConfig.ValidCode = DRDConfig -> ValidCode;
      NewConfig.Personality = DRDConfig -> Personality;

      // NOTE: The SW supports up to three signalling types "gSigType"
      // each of which can have a table of different frequencies.
      // That is why there are 3 sets of Cal values (1, 2 and 3).

      NewConfig.AmpsCalibration = DRDConfig -> AmpsCalibration;
      NewConfig.AmpsOffset = DRDConfig -> AmpsOffset;

      // save present frequency

      if ( state -> SigType == DRDPersTable.SigType1 )
      {
        numfreqs = DRDPersTable.NumFreqs1;
      }
      else if ( state -> SigType == DRDPersTable.SigType2)
      {
        numfreqs = DRDPersTable.NumFreqs2;
      }
      else
      {
        numfreqs = DRDPersTable.NumFreqs3;
      }

      state -> PresAmpsOffsetCal = state -> aAmpsOffsetCalTab;
      state -> PresAmpsGainCal = state -> aAmpsGainCalTab;

      GetMeasUnits(state, ampsunits);

      changes = 0;

      ClearScreen();

      PrintCalibMenu( state );

      state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
    break;

    case MAIN_CALIB_MENU_LEVEL :
      ClearScreen();

      PrintCalibMenu( state );

      state -> LineMax = 9;

      state -> LineSize = 0;

      state -> LineString[0] = 0;

      state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
    break;

    case INPUT_CALIB_MENU_LEVEL :
      
      if ( point[1] != '\r' )
      {
        ClearScreen();

        PrintCalibMenu( state );

        state -> LineMax = 9;

        state -> LineSize = 0;

        state -> LineString[0] = 0;

        state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;

        return;
      }

      switch( (point[0] & 0x40) ? (point[0] & 0xdf) : point[0] )
      {
        case 'Q' :
          if ( changes != 0 )
          {
            sprintf(output, "\r\n\nYou changed the calibration without saving.");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            sprintf(output, "\r\nAre you sure you want to quit? (y or n) : ");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            state -> LineMax = 9;

            state -> LineSize = 0;

            state -> LineString[0] = 0;

            state -> CalibMenuLevel = INPUT1_CALIB_MENU_LEVEL;
          }
          else
          {
            ClearScreen();

            PrintMainMenu( state );

            state -> MenuState = MENU_MAIN_INPUT_LEVEL;

	    state -> CalibMenuLevel = TOP_CALIB_MENU_LEVEL;
          }
        break;

        case '1' :
        {
          for(i = 0; i < numfreqs; i++)
          {
            //gPresDispFreq= gaDispFreqTab[i];
            state -> PresFreqIdx = i;
            GetFreqForDisplay(state, dispbuf);
            sprintf(output, "\r\n%2ld : %s", i+1, dispbuf);
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          }

          sprintf(output, "\r\nEnter new frequency index (1 to %ld) : ", numfreqs);
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> LineMax = 2;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

          state -> CalibMenuLevel = INPUT2_CALIB_MENU_LEVEL;
        }
	break;

	case '2' :
	{
          state -> SecondsCounter = 0;
          CalculateNewSettings( state, 0 );

          // must override the present cal values while doing the new cal
          state -> PresAmpsOffsetCal = 0;
          state -> PresAmpsGainCal = 1.0;

          sprintf(output, "\r\n\nHit any key when done");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\n");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> Calibrating = 1;

          state -> LineMax = 1;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

	  state -> CalibMenuLevel = INPUT3_CALIB_MENU_LEVEL;
	}
	break;

	case '3' :
          state -> SecondsCounter = 0;
          CalculateNewSettings( state, 0 );

          sprintf(output, "\r\n\nHit any key when done");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\n");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> Calibrating = 1;

          state -> LineMax = 1;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

     	  state -> CalibMenuLevel = INPUT4_CALIB_MENU_LEVEL;
	break;

	case '4' :
          state -> SecondsCounter = 0;
          CalculateNewSettings( state, 0 );

          // must override the present gain cal value while doing the new gain cal
          state -> PresAmpsGainCal = .75;                        // temporarily set cal to 10000

          sprintf(output, "\r\n\nHit any key when done");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\n");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> Calibrating = 1;

          state -> LineMax = 9;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

	  state -> CalibMenuLevel = INPUT5_CALIB_MENU_LEVEL;
	break;

	case '5' :
          state -> SecondsCounter= 0;
          CalculateNewSettings( state, 0 );

          sprintf(output, "\r\n\nHit any key when done");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\n");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> Calibrating = 1;

          state -> LineMax = 9;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

	  state -> CalibMenuLevel = INPUT10_CALIB_MENU_LEVEL;
	break;

	case '6' :
          if ( changes == 0 )
          {
            sprintf(output, "\r\n\nYou did not make any changes.");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            sprintf(output, "\r\nAre you sure you want to save? (y or n) : ");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            state -> LineMax = 9;

            state -> LineSize = 0;

            state -> LineString[0] = 0;

	    state -> CalibMenuLevel = INPUT11_CALIB_MENU_LEVEL;
          }
	  else
	  {
            sprintf(output, "\r\n\nThis will erase FLASH Config and save new values.");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            sprintf(output, "\r\nAre you sure? (y or n) : ");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            state -> LineMax = 9;

            state -> LineSize = 0;

            state -> LineString[0] = 0;

            state -> CalibMenuLevel = INPUT12_CALIB_MENU_LEVEL;
	  }
	  break;

	default :
	break;
      }
    break;

    case INPUT1_CALIB_MENU_LEVEL :
      if ( state -> LineSize < state -> LineMax )
      {
        if ( (state -> LineString[0] & 0xdf) == 'Y' )
        {
          // restore all parameters

          CalculateNewSettings( state, 0 );

          GetFreqForDisplay(state, dispbuf);
          LCD_Gotoxy(state, 1, 2);
          LCD_Puts( &LCDTransmitRingBuffer, dispbuf, strlen(dispbuf) );

          ClearScreen();

          PrintMainMenu( state );

          state -> MenuState = MENU_MAIN_INPUT_LEVEL;

          state -> CalibMenuLevel = TOP_CALIB_MENU_LEVEL;
        }
        else
        {
          ClearScreen();

          PrintCalibMenu( state );

          state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
        }
      }
    break;

    case INPUT2_CALIB_MENU_LEVEL :
      if ( state -> LineSize <= state -> LineMax )
      {
        if ( IsNumeric( state -> LineString, state -> LineSize - 1 ) )
        {
          j = state -> PresFreqIdx;

          state -> LineString[state -> LineSize - 1] = 0;

	  val32 = atoi( state -> LineString );

          sprintf(output, "Val: %ld\r\n", val32);
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          if ( (val32 > 0) && (val32 <= numfreqs) )
          {
            state -> PresFreqIdx = val32 - 1;

            ClearScreen();

            PrintCalibMenu( state );

            state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
          }
          else
          {
            state -> PresFreqIdx = j;

            sprintf(output, "\r\nInvalid Entry." );
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            sprintf(output, "\r\nEnter new frequency index (1 to %ld) : ", numfreqs);
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            state -> LineMax = 2;

            state -> LineSize = 0;

            state -> LineString[0] = 0;
          }
        }
        else
        {
          sprintf(output, "\r\nInvalid Entry." );
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nEnter new frequency index (1 to %ld) : ", numfreqs);
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> LineMax = 2;

          state -> LineSize = 0;

          state -> LineString[0] = 0;
        }
      }
      else
      {
        sprintf(output, "\r\nInvalid Entry." );
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        sprintf(output, "\r\nEnter new frequency index (1 to %ld) : ", numfreqs);
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        state -> LineMax = 2;

        state -> LineSize = 0;

        state -> LineString[0] = 0;
      }
    break;

    case INPUT3_CALIB_MENU_LEVEL :
      val32 = state -> PeakAvg;
      GetCorrectAmpsForDisplay(state, dispbuf, val32);
      //iprintf("\r%s mApp  (raw=%ld)  ",dispbuf, state -> PeakAvg);
      //iprintf("\r%s %s  (raw=%ld)  ",dispbuf, gPersTable[gDRDPersonality].AmpsUnits1, state -> PeakAvg);
      sprintf(output, "\r%s %s  (raw=%d)  ",dispbuf, ampsunits, val32);
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

      if ( charPresent )
      {
        state -> Calibrating = 0;

        state -> PresAmpsOffsetCal = state -> PeakAvg;
        state -> aAmpsOffsetCalTab = state -> PresAmpsOffsetCal;

        NewConfig.AmpsOffset = state -> PresAmpsOffsetCal;
        DRDConfig -> AmpsOffset = state -> PresAmpsOffsetCal;

        changes = 1;

        ClearScreen();

        PrintCalibMenu( state );

        state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
      }
    break;

    case INPUT4_CALIB_MENU_LEVEL :
      val32 = state -> PeakAvg;

      GetCorrectAmpsForDisplay(state, dispbuf, val32);

      //iprintf("\r%s mApp  (raw=%ld)  ",dispbuf, state -> PeakAvg);
      //iprintf("\r%s %s  (raw=%ld)  ",dispbuf, gPersTable[gDRDPersonality].AmpsUnits1, state -> PeakAvg);
      sprintf(output, "\r%s %s  (raw=%d)  ",dispbuf, ampsunits, val32);
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

      if ( charPresent )
      {
        state -> Calibrating = 0;

        ClearScreen();

        PrintCalibMenu( state );

        state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
      }
    break;

    case INPUT5_CALIB_MENU_LEVEL :
      val32 = state -> PeakAvg;

      actualamps = GetCorrectAmpsForDisplay(state, dispbuf, val32);

      //iprintf("\r%s mApp  ",dispbuf);
      //iprintf("\r%s %s  ",dispbuf, gPersTable[gDRDPersonality].AmpsUnits1);
      sprintf(output, "\r%s %s  (raw=%d)  ",dispbuf, ampsunits, val32);
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

      if ( charPresent )
      {
        state -> Calibrating = 0;

        if ( state -> DRDPersonality == DRD_PERSONALITY_BART )
        {
          val32f = actualamps;

          sprintf(output, "\r\nActual reading was %4.1f %s", val32f, ampsunits);
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nEnter desired reading : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> LineMax = 9;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

	  state -> CalibMenuLevel = INPUT6_CALIB_MENU_LEVEL;
	}
        else
        {
          //iprintf("\nActual reading was %s %s", dispbuf, gPersTable[gDRDPersonality].AmpsUnits1);
          sprintf(output, "\r\nActual reading was %s %s", dispbuf, ampsunits);
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nEnter desired reading : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> LineMax = 9;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

	  state -> CalibMenuLevel = INPUT7_CALIB_MENU_LEVEL;
	}
      }
    break;

    case INPUT6_CALIB_MENU_LEVEL :
      if ( state -> LineSize < state -> LineMax )
      {
        if ( IsNumeric( state -> LineString, state -> LineSize - 1 ) )
        {
          state -> LineString[state -> LineSize - 1] = 0;

          if ( state -> LineString[0] == 'x' )
          {
            val32f = atof( &state -> LineString[1] );
          }
          else if ( state -> SigType == SIGTYPE_BARTSC )
          {
	    targetamps = atof( state -> LineString );

            val32f = state -> PresAmpsGainCal*(targetamps/actualamps);
          }
          else if ( state -> SigType == SIGTYPE_BARTMV )
          {
	    targetamps = atof( state -> LineString );

            sprintf(output, "\r\ntarg amps = %5.4f\r\n", targetamps);
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

            val32f = state -> PresAmpsGainCal*(targetamps/actualamps);
          }

          sprintf(output, "\r\nNew Amps Cal will be %5.4f", val32f);
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nIs this OK? : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

	  state -> CalibMenuLevel = INPUT8_CALIB_MENU_LEVEL;
        }
        else
        {
          sprintf( output, "\r\nInvalid Entry." );
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nEnter desired reading : ");
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

        sprintf(output, "\r\nEnter desired reading : ");
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        state -> LineMax = 9;

        state -> LineSize = 0;

        state -> LineString[0] = 0;
      }
    break;

    case INPUT7_CALIB_MENU_LEVEL :
      if ( state -> LineSize < state -> LineMax )
      {
        if ( IsNumeric( state -> LineString, state -> LineSize ) )
        {
          state -> LineString[state -> LineSize - 1] = 0;

          if ( state -> LineString[0] == 'x' )
          {
            val32 = atof( &state -> LineString[1] );
          }
          else
          {
            targetamps = atof( state -> LineString );

            val32f = state -> PresAmpsGainCal*(targetamps/actualamps);
          }

          sprintf(output, "\r\nNew Amps Cal will be %5.4f", val32f);
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nIs this OK? : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> LineMax = 9;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

	  state -> CalibMenuLevel = INPUT9_CALIB_MENU_LEVEL;
        }
        else
        {
          sprintf(output, "\r\nInvalid Entry.");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nEnter desired reading : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> LineMax = 9;

          state -> LineSize = 0;

          state -> LineString[0] = 0;
        }
      }
      else
      {
        sprintf(output, "\r\nInvalid Entry.");
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        sprintf(output, "\r\nEnter desired reading : ");
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        state -> LineMax = 9;

        state -> LineSize = 0;

        state -> LineString[0] = 0;
      }
    break;

    case INPUT8_CALIB_MENU_LEVEL :
      if ( state -> LineSize < state -> LineMax )
      {
        state -> LineString[state -> LineSize - 1] = 0;

        if ( (state -> LineString[0] & 0xdf) == 'Y')
        {
          state -> PresAmpsGainCal = val32f;
          state -> aAmpsGainCalTab = state -> PresAmpsGainCal;

          NewConfig.AmpsCalibration = state -> PresAmpsGainCal;
          DRDConfig -> AmpsCalibration = state -> PresAmpsGainCal;
          changes = 1;
        }

        ClearScreen();

        PrintCalibMenu( state );

        state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
      }
      else
      {
        sprintf(output, "\r\nInvalid Entry.");
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        sprintf(output, "\r\nIs this OK? : ");
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        state -> LineMax = 9;

        state -> LineSize = 0;

        state -> LineString[0] = 0;
      }
    break;

    case INPUT9_CALIB_MENU_LEVEL :
      if ( state -> LineSize < state -> LineMax )
      {
        if ( (state -> LineString[0] & 0xdf) == 'Y' )
        {
          state -> PresAmpsGainCal = val32f;
          state -> aAmpsGainCalTab = state -> PresAmpsGainCal;

          NewConfig.AmpsCalibration = state -> PresAmpsGainCal;
          DRDConfig -> AmpsCalibration = state -> PresAmpsGainCal;

          changes = 1;
        }

        ClearScreen();

        PrintCalibMenu( state );

        state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
      }
      else
      {
        sprintf(output, "\r\nInvalid Entry.");
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        state -> LineMax = 9;

        state -> LineSize = 0;

        state -> LineString[0] = 0;
      }
    break;

    case INPUT10_CALIB_MENU_LEVEL :
      val32 = state -> PeakAvg;

      actualamps = GetCorrectAmpsForDisplay(state, dispbuf, val32);

      sprintf(output, "\r%s %s  (raw=%d)  ",dispbuf, ampsunits, val32);
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

      if ( charPresent )
      {
        state -> Calibrating = 0;

        ClearScreen();

        PrintCalibMenu( state );

        state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
      }
    break;

    case INPUT11_CALIB_MENU_LEVEL :
      if ( state -> LineSize <= state -> LineMax )
      {
        state -> LineString[state -> LineSize - 1] = 0;

        if ((state -> LineString[0] & 0xdf) != 'Y')
        {
          sprintf(output, "\r\n\nThis will erase FLASH Config and save new values.");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          sprintf(output, "\r\nAre you sure? (y or n) : ");
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          state -> LineMax = 9;

          state -> LineSize = 0;

          state -> LineString[0] = 0;

          state -> CalibMenuLevel = INPUT12_CALIB_MENU_LEVEL;
        }
	else
	{
          PrintCalibMenu( state );

          state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
	}
      }
      else
      {
        sprintf( output, "\r\nInvalid Entry." );
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        sprintf(output, "\r\nAre you sure you want to save? (y or n) : ");
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        state -> LineMax = 9;

        state -> LineSize = 0;

        state -> LineString[0] = 0;
      }
    break;

    case INPUT12_CALIB_MENU_LEVEL :
      if ( state -> LineSize < state -> LineMax )
      {
        if ((state -> LineString[0] & 0xdf) == 'Y')
        {
          sprintf(output, "\r\nSaving Config sector" );
          SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

          Parameters[2] = state -> PresAmpsGainCal*1000.0;
          Parameters[3] = state -> PresAmpsOffsetCal;

          status = SaveParameters();

          ClearScreen();

          if ( status )
          {
            sprintf(output, "\r\nCannot save Config sector");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          }
	  else
	  {
            changes = 0;
	  }

          state -> Calibrating = 0;

          PrintCalibMenu( state );

          state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
	}
	else
	{
          state -> Calibrating = 0;

          ClearScreen();

          PrintCalibMenu( state );

          state -> CalibMenuLevel = INPUT_CALIB_MENU_LEVEL;
	}
      }
      else
      {
        sprintf( output, "\r\nInvalid Entry." );
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        sprintf(output, "\r\nAre you sure? (y or n) : ");
        SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

        state -> LineMax = 9;

        state -> LineSize = 0;

        state -> LineString[0] = 0;
      }
    break;

    default :
    break;
  }
} // end Calib_DoCalibration()

