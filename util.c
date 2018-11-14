#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensorhub.h"
#include "drdstate.h"
#include "kernel_timer.h"

static char output[80];
static char dispbuf[20];

extern PERS_TAB_T DRDPersTable;
extern CONFIG_HEADER_T *DRDConfig;
extern struct ring_buffer USART0TransmitRingBuffer;
extern struct lcd_ring_buffer LCDTransmitRingBuffer;

/**************************************************************************
//           0 MPH   6 MPH   18 MPH  27 MPH  36 MPH  50 MPH  70 MPH  80 MPH
// --------  ------  ------  ------  ------  ------  ------  ------  ------
// PHASE 1   100000  100001  101001  100101  100011  101011  100111  101111
// PHASE 2   010000  110000  110100  110010  110001  110101  110011  110111
// PHASE 3   001000  011000  011010  011001  111000  111010  111001  111011
// PHASE 4   000100  001100  001101  101100  011100  011101  111100  111101
// PHASE 5   000010  000110  100110  010110  001110  101110  011110  111110
// PHASE 6   000001  000011  010011  001011  000111  010111  001111  011111
//
// -1 (255) is used for codes that are invalid
// -2 (254) is used for all 0's
// -3 (253) is used for all 1's
***************************************************************************/
const unsigned char BART_SpeedCodeLUT[64] =
{
  254,    // 000000 0x00
    0,    // 000001 0x01
    0,    // 000010 0x02
    6,    // 000011 0x03
    0,    // 000100 0x04
  255,    // 000101 0x05
    6,    // 000110 0x06
   36,    // 000111 0x07
    0,    // 001000 0x08
  255,    // 001001 0x09
  255,    // 001010 0x0A
   27,    // 001011 0x0B
    6,    // 001100 0x0C
   18,    // 001101 0x0D
   36,    // 001110 0x0E
   70,    // 001111 0x0F
    0,    // 010000 0x10
  255,    // 010001 0x11
  255,    // 010010 0x12
   18,    // 010011 0x13
  255,    // 010100 0x14
  255,    // 010101 0x15
   27,    // 010110 0x16
   50,    // 010111 0x17
    6,    // 011000 0x18
   27,    // 011001 0x19
   18,    // 011010 0x1A
  255,    // 011011 0x1B
   36,    // 011100 0x1C
   50,    // 011101 0x1D
   70,    // 011110 0x1E
   80,    // 011111 0x1F
    0,    // 100000 0x20
    6,    // 100001 0x21
  255,    // 100010 0x22
   36,    // 100011 0x23
  255,    // 100100 0x24
   27,    // 100101 0x25
   18,    // 100110 0x26
   70,    // 100111 0x27
  255,    // 101000 0x28
   18,    // 101001 0x29
  255,    // 101010 0x2A
   50,    // 101011 0x2B
   27,    // 101100 0x2C
  255,    // 101101 0x2D
   50,    // 101110 0x2E
   80,    // 101111 0x2F
    6,    // 110000 0x30
   36,    // 110001 0x31
   27,    // 110010 0x32
   70,    // 110011 0x33
   18,    // 110100 0x34
   50,    // 110101 0x35
  255,    // 110110 0x36
   80,    // 110111 0x37
   36,    // 111000 0x38
   70,    // 111001 0x39
   50,    // 111010 0x3A
   80,    // 111011 0x3B
   70,    // 111100 0x3C
   80,    // 111101 0x3D
   80,    // 111110 0x3E
  253     // 111111 0x3F
};

void DelayMSecs( int mSecs )
{
  uint64_t sTime, eTime;
  uint32_t time = mSecs*1000;

  sTime = g_Timer.GetCurrentUsec();

  while( (g_Timer.GetCurrentUsec() - sTime) < time );
}


////////////////////////////////////////////////////////////////////////////////
// Function: PersonalityInit
//
// Summary:  Init all variables after boot or after signalling type changes.
//
////////////////////////////////////////////////////////////////////////////////
 
void PersonalityInit( struct _drd_state_ *state )
{
  int i;

  state -> PeakSampRate = SAMPLE_FREQUENCY;

  // BART CIRCA
  switch (state -> DRDPersonality)
  {
    case DRD_PERSONALITY_BART:
    {
      state -> Mode = DRDPersTable.InitMode;
      state -> PresFreqIdx = DRDPersTable.InitFreqIdx;

      state -> DemodState = BART_STATE_NOCARRIER;
      state -> LastDemodState = BART_STATE_NOCARRIER;

      if( state -> SigType == SIGTYPE_BARTSC )
      {
        state -> DemodThreshold = BART_MIN_DETECT_SC;
        state -> DemodThresholdHyst = (BART_MIN_DETECT_SC * 95) / 100;     // about 5% lower low going threshold (roughly 50 mA)

        state -> SensorType = DRDPersTable.SensorType1 & 0x00FF;
        state -> SensDetect = DRDPersTable.SensorType1 & 0xFF00;

        state -> NumFreqs = DRDPersTable.NumFreqs1;

        // populate calibration gain, offset and freq tables with correct values

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency table with correct values

        for(i = 0; i < state -> NumFreqs; i++)
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable1[i];
        }
      }
      else if ( state -> SigType == SIGTYPE_BARTMV )
      {
        state -> DemodThreshold = BART_MIN_DETECT_MV;
        state -> DemodThresholdHyst = (BART_MIN_DETECT_MV * 95) / 100;     // about 5% lower low going threshold (roughly 50 mA)
    
        state -> SensorType = DRDPersTable.SensorType2 & 0xFF;
        state -> SensDetect = DRDPersTable.SensorType2 & 0xFF00;

        state -> NumFreqs = DRDPersTable.NumFreqs2;

        // populate calibration and offset tables with correct values

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency table with correct values

        for( i = 0; i < state -> NumFreqs; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable2[i];
        }
      }
    }
    break;

    // Denver RTD (DRD-11)
    // DRD-11 has only a single signalling type (CAB)
    // and a single carrier frequency (100 Hz)

    case DRD_PERSONALITY_DRD11:
    {
      state -> SensorType = DRDPersTable.SensorType1 & 0xFF;
      state -> SensDetect = DRDPersTable.SensorType1 & 0xFF00;

      state -> Mode = DRDPersTable.InitMode;
      state -> PresFreqIdx = DRDPersTable.InitFreqIdx;

      state -> DemodState = CAB_STATE_NOCARRIER;
      state -> LastDemodState = CAB_STATE_NOCARRIER;

      state -> DemodThreshold = DRD11_MIN_DETECT;
      state -> DemodThresholdHyst = (DRD11_MIN_DETECT * 95) / 100;     // about 5% lower low going threshold (roughly 50 mA)

      state -> NumFreqs = DRDPersTable.NumFreqs1;

      // populate calibration and offset tables with correct values
      // Note this will need modified for units with more than one frequency table
	
      state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
      state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

      // populate frequency tables with correct values
      // Note this will need modified for units with more than one frequency table
	
      for( i = 0; i < state -> NumFreqs; i++ )
      {
        state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable1[i];
      }
    }
    break;

    // DRD-ROW has two signalling types (AFTC and CAB)
    // Each of these types has it's own set of carrier frequencies needed!

    case DRD_PERSONALITY_DRD_ROW:
    {
      state -> Mode = DRDPersTable.InitMode;
      state -> PresFreqIdx = DRDPersTable.InitFreqIdx;

      if ( state -> SigType == SIGTYPE_DRD_ROW_AFTC)
      {
        state -> SensorType = DRDPersTable.SensorType2 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = AFTC_STATE_NOCARRIER;
        state -> LastDemodState = AFTC_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_ROW_AFTC_MIN_DETECT;
        state -> DemodThresholdHyst = (DRD_ROW_AFTC_MIN_DETECT * 95) / 100;

        state -> NumFreqs = DRDPersTable.NumFreqs2;

        // populate calibration and offset tables with correct values

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency table with correct values

        for( i = 0; i < MAX_NUM_FREQS; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable2[i];
        }
      }
      else if ( state -> SigType == SIGTYPE_DRD_ROW_CAB)
      {
        state -> SensorType = DRDPersTable.SensorType1 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = DRD_ROW_CAB_STATE_NOCARRIER;
        state -> LastDemodState = DRD_ROW_CAB_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_ROW_CAB_MIN_DETECT;
        state -> DemodThresholdHyst =  (DRD_ROW_CAB_MIN_DETECT * 95) / 100;     // about 5% lower low going threshold 

        state -> NumFreqs = DRDPersTable.NumFreqs1;

        // populate calibration and offset tables with correct values
        // Note this will need modified for units with more than one frequency table

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency tables with correct values
        // Note this will need modified for units with more than one frequency table

        for( i = 0; i < state -> NumFreqs; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable1[i];
        }
      }
    }
    break;

    // DRD-ST has two signalling types (AFTC and CAB)
    // Each of these types has it's own set of carrier frequencies needed!

    case DRD_PERSONALITY_DRD_MFOR:
    {
      state -> Mode = DRDPersTable.InitMode;
      state -> PresFreqIdx = DRDPersTable.InitFreqIdx;

      if ( state -> SigType == SIGTYPE_DRD_MFOR_AFTC )
      {
        state -> SensorType = DRDPersTable.SensorType2 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = AFTC_STATE_NOCARRIER;
        state -> LastDemodState = AFTC_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_MFOR_AFTC_MIN_DETECT;
        state -> DemodThresholdHyst = (DRD_MFOR_AFTC_MIN_DETECT * 95) / 100;

        state -> NumFreqs = DRDPersTable.NumFreqs2;

        // populate calibration and offset tables with correct values

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency table with correct values

        for( i = 0; i < MAX_NUM_FREQS; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable2[i];
        }
      }
      else if ( state -> SigType == SIGTYPE_DRD_MFOR_CAB )
      {
        state -> SensorType = DRDPersTable.SensorType1 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = DRD_MFOR_CAB_STATE_NOCARRIER;
        state -> LastDemodState = DRD_MFOR_CAB_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_MFOR_CAB_MIN_DETECT;
        state -> DemodThresholdHyst =  (DRD_MFOR_CAB_MIN_DETECT * 95) / 100;     // about 5% lower low going threshold 

        state -> NumFreqs = DRDPersTable.NumFreqs1;

        // populate calibration and offset tables with correct values
        // Note this will need modified for units with more than one frequency table

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency tables with correct values
        // Note this will need modified for units with more than one frequency table

        for( i = 0; i < state -> NumFreqs; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable1[i];
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_STL:
    {
      state -> Mode = DRDPersTable.InitMode;
      state -> PresFreqIdx = DRDPersTable.InitFreqIdx;

      if ( state -> SigType == SIGTYPE_DRD_STL_AFTC )
      {
        state -> SensorType = DRDPersTable.SensorType2 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = AFTC_STATE_NOCARRIER;
        state -> LastDemodState = AFTC_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_STL_AFTC_MIN_DETECT;
        state -> DemodThresholdHyst = (DRD_STL_AFTC_MIN_DETECT * 95) / 100;

        state -> NumFreqs = DRDPersTable.NumFreqs2;

        // populate calibration and offset tables with correct values

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency table with correct values

        for( i = 0; i < MAX_NUM_FREQS; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable2[i];
        }
      }
      else if ( state -> SigType == SIGTYPE_DRD_STL_CAB )
      {
        state -> SensorType = DRDPersTable.SensorType1 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = DRD_STL_CAB_STATE_NOCARRIER;
        state -> LastDemodState = DRD_STL_CAB_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_STL_CAB_MIN_DETECT;
        state -> DemodThresholdHyst =  (DRD_STL_CAB_MIN_DETECT * 95) / 100;     // about 5% lower low going threshold 

        state -> NumFreqs = DRDPersTable.NumFreqs1;

        // populate calibration and offset tables with correct values
        // Note this will need modified for units with more than one frequency table

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency tables with correct values
        // Note this will need modified for units with more than one frequency table

        for( i = 0; i < state -> NumFreqs; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable1[i];
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_RET:
    {
      state -> Mode = DRDPersTable.InitMode;
      state -> PresFreqIdx = DRDPersTable.InitFreqIdx;

      if ( state -> SigType == SIGTYPE_DRD_RET_AFTC )
      {
        state -> SensorType = DRDPersTable.SensorType2 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = AFTC_STATE_NOCARRIER;
        state -> LastDemodState = AFTC_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_RET_AFTC_MIN_DETECT;
        state -> DemodThresholdHyst = (DRD_RET_AFTC_MIN_DETECT * 95) / 100;

        state -> NumFreqs = DRDPersTable.NumFreqs2;

        // populate calibration and offset tables with correct values

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency table with correct values

        for( i = 0; i < MAX_NUM_FREQS; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable2[i];
        }
      }
      else if ( state -> SigType == SIGTYPE_DRD_RET_CAB )
      {
        state -> SensorType = DRDPersTable.SensorType1 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = DRD_RET_CAB_STATE_NOCARRIER;
        state -> LastDemodState = DRD_RET_CAB_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_RET_CAB_MIN_DETECT;
        state -> DemodThresholdHyst =  (DRD_RET_CAB_MIN_DETECT * 95) / 100;     // about 5% lower low going threshold 

        state -> NumFreqs = DRDPersTable.NumFreqs1;

        // populate calibration and offset tables with correct values
        // Note this will need modified for units with more than one frequency table

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency tables with correct values
        // Note this will need modified for units with more than one frequency table

        for( i = 0; i < state -> NumFreqs; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable1[i];
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_CHAR:
    {
      state -> Mode = DRDPersTable.InitMode;
      state -> PresFreqIdx = DRDPersTable.InitFreqIdx;

      if ( state -> SigType == SIGTYPE_DRD_CHAR_AFTC )
      {
        state -> SensorType = DRDPersTable.SensorType2 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = AFTC_STATE_NOCARRIER;
        state -> LastDemodState = AFTC_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_CHAR_AFTC_MIN_DETECT;
        state -> DemodThresholdHyst = (DRD_CHAR_AFTC_MIN_DETECT * 95) / 100;

        state -> NumFreqs = DRDPersTable.NumFreqs2;

        // populate calibration and offset tables with correct values

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency table with correct values

        for( i = 0; i < MAX_NUM_FREQS; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable2[i];
        }
      }
      else if ( state -> SigType == SIGTYPE_DRD_CHAR_CAB )
      {
        state -> SensorType = DRDPersTable.SensorType1 & 0xFF;

        // gSensDetect set by actual detection since either
        // shunt is allowed and a scale factor is applied.

        state -> DemodState = DRD_CHAR_CAB_STATE_NOCARRIER;
        state -> LastDemodState = DRD_CHAR_CAB_STATE_NOCARRIER;

        state -> DemodThreshold = DRD_CHAR_CAB_MIN_DETECT;
        state -> DemodThresholdHyst =  (DRD_CHAR_CAB_MIN_DETECT * 95) / 100;     // about 5% lower low going threshold 

        state -> NumFreqs = DRDPersTable.NumFreqs1;

        // populate calibration and offset tables with correct values
        // Note this will need modified for units with more than one frequency table

        state -> aAmpsGainCalTab = DRDConfig -> AmpsCalibration;
        state -> aAmpsOffsetCalTab = DRDConfig -> AmpsOffset;

        // populate frequency tables with correct values
        // Note this will need modified for units with more than one frequency table

        for( i = 0; i < state -> NumFreqs; i++ )
        {
          state -> aDispFreqTab[i] = DRDPersTable.DispFreqTable1[i];
        }
      }
    }
    break;
  } // end switch(gDRDPersonality)
} // end PersonalityInit()

/*******************************************************************************
* Function: CalculateNewSettings
*
* Summary:  Calculates all variables affected by a frequency change.
*
* Details:
* Call this after gPresFreqIdx or gMode or gType changes.
* It re-computes all variables that are dependent on those.
*******************************************************************************/
void CalculateNewSettings( struct _drd_state_ *state, int FilterInit )
{
  int    num_avgs;
  uint32_t val32;
  uint32_t val32a;
  uint32_t rem32;
  uint32_t timer_reload;

  //iprintf("\nPresIdx= %ld",gPresFreqIdx);

  // get correct cal and offset values for present frequency index
  state -> PresAmpsGainCal = state -> aAmpsGainCalTab;
  state -> PresAmpsOffsetCal = state -> aAmpsOffsetCalTab;

  if ( FilterInit )
  {
    state -> PeakDemodAvg = 0.0;
    state -> PeakSquaredAvg = 0.0;
    state -> PeakSquaredAvg1[0] = 0.0;
    state -> PeakSquaredAvg1[1] = 0.0;
    state -> PeakSquaredAvg1[2] = 0.0;
    InitializeFilters();
  }
    
  // get correct display frequency for present frequency index
  state -> PresDispFreq = state -> aDispFreqTab[state -> PresFreqIdx];

  val32 = state -> PeakSampRate;

  switch ( state -> DRDPersonality )
  {
    case DRD_PERSONALITY_BART:      // BART CIRCA
    {
      float32_t frequency, frequency0;

      if ( state -> aDispFreqTab[state -> PresFreqIdx] & 1 )
      {
        frequency = state -> aDispFreqTab[state -> PresFreqIdx];

        frequency0 = state -> aDispFreqTab[state -> PresFreqIdx ^ 1];
      }
      else
      {
        frequency = state -> aDispFreqTab[state -> PresFreqIdx | 1];

        frequency0 = state -> aDispFreqTab[state -> PresFreqIdx];
      }

      InitializeDFTFilters( frequency, frequency0, SAMPLE_FREQUENCY );

      // Originally the display update rate was exactly one second.
      //
      // At one second, the update is always in the same bit and apparently stable readings
      // are obtained when modulation is present.  The readings are actually invalid.
      // By making it 60 msecs less than 1 sec this aliasing will not occur and readings
      // during modulation will fluctuate so user will know they are measuring something
      // they shouldn't be measuring.
      //
      // make BART display update rate about 60 msecs (little longer than one bit time)
      // less than one second to prevent aliasing of AMPs readings into exact bit
      // time positions repeatedly
      /////////////////////////////////////////////////////////////////////////////////

      state -> SampCount = 0;

      // used by demodulation to detect constant carrier or no signal

      state -> DemodMaxOnCounts = ((state -> PeakSampRate / 3) * 1100) /1000;      // 10% longer than a second
      state -> DemodMaxOffCounts = state -> DemodMaxOnCounts;

      // BART speed code data is at 18 Hz.  These variables
      // are used to decode the speed code data.
	
      state -> DemodHalfBit = state -> PeakSampRate / 36;      // time for 1/2 of 18 Hz (1/2 bit time)
      state -> DemodOneBit = state -> PeakSampRate / 18;      // time for 18 Hz (1 bit time)
      state -> DemodShiftCount = 0;
      state -> DemodShiftState = 0;

      // Now determine the size of the secondary peak averaging buffer
      // for the present frequency.  It should yield an overall averaging
      // window of between .5 and 1 seconds with the primary and secondary
      // averages.
      ////////////////////////////////////////////////////////////////////

      val32 = state -> PresDispFreq >> 6;      // this is the greatly simplified formula for the number of
                                        // secondary averages to produce a total window of .5 to 1 sec
                                        // really: NUM_AVGS = 1 / ( (1/2xFREQ) x 128)
      num_avgs = 1;

      // now shift until only 1 bit remains and count
      // the number of shifts and calc the number of
      // averages

      while(val32 > 1)
      {
        val32 >>= 1;
        num_avgs <<= 1;
      }
    }
    break;

    case DRD_PERSONALITY_DRD11:     // Denver RTD
    {
      float32_t frequency;

      frequency = state -> aDispFreqTab[state -> PresFreqIdx];

      InitializeProcData( state, frequency );

      state -> DemodDuty = 0;

      state -> SampCount = 0;
      state -> DemodMaxOnCounts = (((state -> PeakSampRate * 5) / 18) * 1100) /1000;
      state -> DemodMaxOffCounts = state -> PeakSampRate;
    
      // 75 CODE     68.68   1.144667   .8736     50%      31-69%
      // 180 CODE    178.57  2.976167   .3360     50%      22-78%
      // 270 CODE    267.86  4.464333   .2240     50%      16-84%
      state -> CabTimes.DRD11.Cab270MinPer = (state -> PeakSampRate * 202) / 1000;    // SR * .2240 * .9  = SR * .202
      state -> CabTimes.DRD11.Cab270MaxPer = (state -> PeakSampRate * 264) / 1000;    // SR * .2240 * 1.1 = SR * .264
      state -> CabTimes.DRD11.Cab180MinPer = (state -> PeakSampRate * 302) / 1000;    // SR * .3360 * .9  = SR * .302
      state -> CabTimes.DRD11.Cab180MaxPer = (state -> PeakSampRate * 370) / 1000;    // SR * .3360 * 1.1 = SR * .370
      state -> CabTimes.DRD11.Cab075MinPer = (state -> PeakSampRate * 786) / 1000;    // SR * .8736 * .9  = SR * .786
      state -> CabTimes.DRD11.Cab075MaxPer = (state -> PeakSampRate * 961) / 1000;    // SR * .8736 * 1.1 = SR * .961
        
      state -> CabTimes.DRD11.Cab270MinOn =  (state -> PeakSampRate *  36) / 1000;    // SR * .2240 * .16 = SR * .036
      state -> CabTimes.DRD11.Cab270MaxOn =  (state -> PeakSampRate * 188) / 1000;    // SR * .2240 * .84 = SR * .188
      state -> CabTimes.DRD11.Cab180MinOn =  (state -> PeakSampRate *  74) / 1000;    // SR * .3360 * .22 = SR * .074
      state -> CabTimes.DRD11.Cab180MaxOn =  (state -> PeakSampRate * 262) / 1000;    // SR * .3360 * .78 = SR * .262
      state -> CabTimes.DRD11.Cab075MinOn =  (state -> PeakSampRate * 271) / 1000;    // SR * .8736 * .31 = SR * .271
      state -> CabTimes.DRD11.Cab075MaxOn =  (state -> PeakSampRate * 603) / 1000;    // SR * .8736 * .69 = SR * .603
    }
    break;

    case DRD_PERSONALITY_DRD_ROW:
    {
      float32_t frequency;

      frequency = state -> aDispFreqTab[state -> PresFreqIdx];

      InitializeProcData( state, frequency );

      state -> DemodDuty = 0;
      state -> SampCount = 0;

      val32 = state -> PeakSampRate;

      /////////////////////////////////////////////
      // Split here between CAB & AFTC
      /////////////////////////////////////////////

      if ( state -> SigType == SIGTYPE_DRD_ROW_AFTC )
      {
        //   Mod A ==> On Time = 300mS, Period = 360mS
        //   Mod B ==> On Time = 400mS, Period = 460mS
        //   Mod C ==> On Time = 500mS, Period = 560mS

        // 10% longer than 500 ms = 550 ms
	
        state -> DemodMaxOnCounts = (val32 * 550) / 1000;

        // 10% longer than 60 ms = 66 ms
	
        state -> DemodMaxOffCounts = (val32 * 66) / 1000;

        // NOTE: All MaxOn times were set to 15% high instead of 10% high
        // because the filter has rise and fall times that skew them upwards.
	
        state -> AFTC_A_MinPer = (val32 * 324) / 1000;    // SR * .360 *  .9 = SR * .324
        state -> AFTC_A_MaxPer = (val32 * 396) / 1000;    // SR * .360 * 1.1 = SR * .396
        state -> AFTC_B_MinPer = (val32 * 414) / 1000;    // SR * .460 *  .9 = SR * .414
        state -> AFTC_B_MaxPer = (val32 * 506) / 1000;    // SR * .460 * 1.1 = SR * .506
        state -> AFTC_C_MinPer = (val32 * 504) / 1000;    // SR * .560 *  .9 = SR * .504
        state -> AFTC_C_MaxPer = (val32 * 616) / 1000;    // SR * .560 * 1.1 = SR * .616
        state -> AFTC_A_MinOn =  (val32 * 270) / 1000;    // SR * .300 *  .9 = SR * .270
        state -> AFTC_A_MaxOn =  (val32 * 345) / 1000;    // SR * .300 * 1.15 = SR * .345
        state -> AFTC_B_MinOn =  (val32 * 360) / 1000;    // SR * .400 *  .9 = SR * .360
        state -> AFTC_B_MaxOn =  (val32 * 460) / 1000;    // SR * .400 * 1.15 = SR * .460
        state -> AFTC_C_MinOn =  (val32 * 470) / 1000;    // SR * .500 *  .9 = SR * .450 (set a little higher so no overlap with B_MaxOn)
        state -> AFTC_C_MaxOn =  (val32 * 575) / 1000;    // SR * .500 * 1.15 = SR * .575

        state -> DemodState = AFTC_STATE_NOCARRIER;
      }
      else // (gSigType == SIGTYPE_DRD_ROW_CAB)
      {
        state -> CabTimes.ROW.DRD_ROW_CAB_MaxOn  = (val32 * 555) / 1000;
        state -> CabTimes.ROW.DRD_ROW_CAB_MaxOff = (val32 * 2368) / 1000;
        state -> CabTimes.ROW.DRD_ROW_70_MinPer = (val32 * 90) / 1000;
        state -> CabTimes.ROW.DRD_ROW_70_MaxPer = (val32 * 101) / 1000;
        state -> CabTimes.ROW.DRD_ROW_65_MinPer = (val32 * 106) / 1000;
        state -> CabTimes.ROW.DRD_ROW_65_MaxPer = (val32 * 118) / 1000;
        state -> CabTimes.ROW.DRD_ROW_60_MinPer = (val32 * 123) / 1000;
        state -> CabTimes.ROW.DRD_ROW_60_MaxPer = (val32 * 136) / 1000;
        state -> CabTimes.ROW.DRD_ROW_50_MinPer = (val32 * 150) / 1000;
        state -> CabTimes.ROW.DRD_ROW_50_MaxPer = (val32 * 170) / 1000;
        state -> CabTimes.ROW.DRD_ROW_50LOS_MinPer = (val32 * 180) / 1000;
        state -> CabTimes.ROW.DRD_ROW_50LOS_MaxPer = (val32 * 204) / 1000;
        state -> CabTimes.ROW.DRD_ROW_40_MinPer = (val32 * 241) / 1000;
        state -> CabTimes.ROW.DRD_ROW_40_MaxPer = (val32 * 271) / 1000;
        state -> CabTimes.ROW.DRD_ROW_30_MinPer = (val32 * 278) / 1000;
        state -> CabTimes.ROW.DRD_ROW_30_MaxPer = (val32 * 313) / 1000;
        state -> CabTimes.ROW.DRD_ROW_20_MinPer = (val32 * 318) / 1000;
        state -> CabTimes.ROW.DRD_ROW_20_MaxPer = (val32 * 352) / 1000;
        state -> CabTimes.ROW.DRD_ROW_15_MinPer = (val32 * 357) / 1000;
        state -> CabTimes.ROW.DRD_ROW_15_MaxPer = (val32 * 393) / 1000;
        state -> CabTimes.ROW.DRD_ROW_10_MinPer = (val32 * 398) / 1000;
        state -> CabTimes.ROW.DRD_ROW_10_MaxPer = (val32 * 433) / 1000;
        state -> CabTimes.ROW.DRD_ROW_10LOS_MinPer = (val32 * 438) / 1000;
        state -> CabTimes.ROW.DRD_ROW_10LOS_MaxPer = (val32 * 477) / 1000;
        state -> CabTimes.ROW.DRD_ROW_0_MinPer = (val32 * 482) / 1000;
        state -> CabTimes.ROW.DRD_ROW_0_MaxPer = (val32 * 534) / 1000;

        // Setup a short window (~5ms) for peak detection during modulation
        // that starts ~10ms after a logic '1' is decoded.  This allows
        // current to be measured during modulation.

        state -> DemodState = DRD_ROW_CAB_STATE_UNKNOWN;
      }
      break;

      case DRD_PERSONALITY_DRD_MFOR:    // Sound Transit
      {
        float32_t frequency;

        frequency = state -> aDispFreqTab[state -> PresFreqIdx];

        InitializeProcData( state, frequency );

        /////////////////////////////////////////////
        // Split here between CAB & AFTC
        /////////////////////////////////////////////
	
        if ( state -> SigType == SIGTYPE_DRD_MFOR_AFTC )
        {
          //   Mod A ==> On Time = 300mS, Period = 360mS
          //   Mod B ==> On Time = 400mS, Period = 460mS
          //   Mod C ==> On Time = 500mS, Period = 560mS

          // 10% longer than 500 ms = 550 ms

          state -> DemodMaxOnCounts = (val32 * 550) / 1000;

          // 10% longer than 60 ms = 66 ms

          state -> DemodMaxOffCounts = (val32 * 66) / 1000;

          // NOTE: All MaxOn times were set to 15% high instead of 10% high
          // because the filter has rise and fall times that skew them upwards.

          state -> AFTC_A_MinPer = (val32 * 270) / 1000;    // SR * .300 *  .9 = SR * .270
          state -> AFTC_A_MaxPer = (val32 * 330) / 1000;    // SR * .300 * 1.1 = SR * .330
          state -> AFTC_B_MinPer = (val32 * 360) / 1000;    // SR * .400 *  .9 = SR * .360
          state -> AFTC_B_MaxPer = (val32 * 440) / 1000;    // SR * .400 * 1.1 = SR * .440
          state -> AFTC_C_MinPer = (val32 * 450) / 1000;    // SR * .500 *  .9 = SR * .450
          state -> AFTC_C_MaxPer = (val32 * 550) / 1000;    // SR * .500 * 1.1 = SR * .550
          state -> AFTC_A_MinOn =  (val32 * 216) / 1000;    // SR * .240 *  .9 = SR * .216
          state -> AFTC_A_MaxOn =  (val32 * 276) / 1000;    // SR * .240 * 1.15 = SR * .276
          state -> AFTC_B_MinOn =  (val32 * 306) / 1000;    // SR * .340 *  .9 = SR * .306
          state -> AFTC_B_MaxOn =  (val32 * 391) / 1000;    // SR * .340 * 1.15 = SR * .391
          state -> AFTC_C_MinOn =  (val32 * 396) / 1000;    // SR * .440 *  .9 = SR * .396 (set a little higher so no overlap with B_MaxOn)
          state -> AFTC_C_MaxOn =  (val32 * 506) / 1000;    // SR * .440 * 1.15 = SR * .506

          state -> DemodState = AFTC_STATE_NOCARRIER;
        }
        else // (gSigType == SIGTYPE_DRD_MFOR_CAB)
        {
          state -> CabTimes.MFOR.DRD_MFOR_CAB_MaxOn  = (val32 * 555) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_CAB_MaxOff = (val32 * 1598) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_80_MinPer = (val32 * 58) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_80_MaxPer = (val32 * 71) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_70_MinPer = (val32 * 72) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_70_MaxPer = (val32 * 88) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_60_MinPer = (val32 * 101) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_60_MaxPer = (val32 * 123) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_50_MinPer = (val32 * 130) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_50_MaxPer = (val32 * 159) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_40_MinPer = (val32 * 173) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_40_MaxPer = (val32 * 211) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_25_MinPer = (val32 * 230) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_25_MaxPer = (val32 * 282) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_10_MinPer = (val32 * 288) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_10_MaxPer = (val32 * 352) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_0_MinPer = (val32 * 403) / 1000;
          state -> CabTimes.MFOR.DRD_MFOR_0_MaxPer = (val32 * 493) / 1000;

          // Setup a short window (~20ms) for peak detection during modulation
          // that starts ~2ms after a logic '1' is decoded.  This allows
          // current to be measured during modulation.

          state -> DemodState = DRD_MFOR_CAB_STATE_UNKNOWN;
        }
      }
      break;

      case DRD_PERSONALITY_DRD_STL:    // Sound Transit
      {
        float32_t frequency;

        frequency = state -> aDispFreqTab[state -> PresFreqIdx];

        InitializeProcData( state, frequency );

        /////////////////////////////////////////////
        // Split here between CAB & AFTC
        /////////////////////////////////////////////
	
        if ( state -> SigType == SIGTYPE_DRD_STL_AFTC )
        {
          //   Mod A ==> On Time = 300mS, Period = 360mS
          //   Mod B ==> On Time = 400mS, Period = 460mS
          //   Mod C ==> On Time = 500mS, Period = 560mS

          // 10% longer than 500 ms = 550 ms

          state -> DemodMaxOnCounts = (val32 * 550) / 1000;

          // 10% longer than 60 ms = 66 ms

          state -> DemodMaxOffCounts = (val32 * 66) / 1000;

          // NOTE: All MaxOn times were set to 15% high instead of 10% high
          // because the filter has rise and fall times that skew them upwards.

          state -> AFTC_A_MinPer = (val32 * 270) / 1000;    // SR * .300 *  .9 = SR * .270
          state -> AFTC_A_MaxPer = (val32 * 330) / 1000;    // SR * .300 * 1.1 = SR * .330
          state -> AFTC_B_MinPer = (val32 * 360) / 1000;    // SR * .400 *  .9 = SR * .360
          state -> AFTC_B_MaxPer = (val32 * 440) / 1000;    // SR * .400 * 1.1 = SR * .440
          state -> AFTC_C_MinPer = (val32 * 450) / 1000;    // SR * .500 *  .9 = SR * .450
          state -> AFTC_C_MaxPer = (val32 * 550) / 1000;    // SR * .500 * 1.1 = SR * .550
          state -> AFTC_A_MinOn =  (val32 * 216) / 1000;    // SR * .240 *  .9 = SR * .216
          state -> AFTC_A_MaxOn =  (val32 * 276) / 1000;    // SR * .240 * 1.15 = SR * .276
          state -> AFTC_B_MinOn =  (val32 * 306) / 1000;    // SR * .340 *  .9 = SR * .306
          state -> AFTC_B_MaxOn =  (val32 * 391) / 1000;    // SR * .340 * 1.15 = SR * .391
          state -> AFTC_C_MinOn =  (val32 * 396) / 1000;    // SR * .440 *  .9 = SR * .396 (set a little higher so no overlap with B_MaxOn)
          state -> AFTC_C_MaxOn =  (val32 * 506) / 1000;    // SR * .440 * 1.15 = SR * .506

          state -> DemodState = AFTC_STATE_NOCARRIER;
        }
        else // (gSigType == SIGTYPE_DRD_STL_CAB)
        {
          state -> CabTimes.STL.DRD_STL_CAB_MaxOn  = (val32 * 555) / 1000;
          state -> CabTimes.STL.DRD_STL_CAB_MaxOff = (val32 * 1598) / 1000;
          state -> CabTimes.STL.DRD_STL_55_MinPer = (val32 * 58) / 1000;
          state -> CabTimes.STL.DRD_STL_55_MaxPer = (val32 * 71) / 1000;
          state -> CabTimes.STL.DRD_STL_45_MinPer = (val32 * 72) / 1000;
          state -> CabTimes.STL.DRD_STL_45_MaxPer = (val32 * 88) / 1000;
          state -> CabTimes.STL.DRD_STL_STREET_MinPer = (val32 * 101) / 1000;
          state -> CabTimes.STL.DRD_STL_STREET_MaxPer = (val32 * 123) / 1000;
          state -> CabTimes.STL.DRD_STL_35_MinPer = (val32 * 130) / 1000;
          state -> CabTimes.STL.DRD_STL_35_MaxPer = (val32 * 159) / 1000;
          state -> CabTimes.STL.DRD_STL_25_MinPer = (val32 * 173) / 1000;
          state -> CabTimes.STL.DRD_STL_25_MaxPer = (val32 * 211) / 1000;
          state -> CabTimes.STL.DRD_STL_YARD_MinPer = (val32 * 230) / 1000;
          state -> CabTimes.STL.DRD_STL_YARD_MaxPer = (val32 * 282) / 1000;
          state -> CabTimes.STL.DRD_STL_15_MinPer = (val32 * 288) / 1000;
          state -> CabTimes.STL.DRD_STL_15_MaxPer = (val32 * 352) / 1000;
          state -> CabTimes.STL.DRD_STL_5_MinPer = (val32 * 403) / 1000;
          state -> CabTimes.STL.DRD_STL_5_MaxPer = (val32 * 493) / 1000;

          // Setup a short window (~20ms) for peak detection during modulation
          // that starts ~2ms after a logic '1' is decoded.  This allows
          // current to be measured during modulation.

          state -> DemodState = DRD_STL_CAB_STATE_UNKNOWN;
        }
      }
      break;

      case DRD_PERSONALITY_DRD_RET:    // Sound Transit
      {
        float32_t frequency, frequency0;
        int idx = state -> PresFreqIdx + 1;

        if ( idx > state -> NumFreqs )
        {
          idx = 0;
        }

        state -> PresFreq1Idx = (state -> PresFreqIdx + 1) % state -> NumFreqs;

        frequency = state -> aDispFreqTab[state -> PresFreqIdx];
        frequency0 = state -> aDispFreqTab[idx];
        
        InitializeProcData( state, frequency );

        InitializeDFTFilters( frequency, frequency0, SAMPLE_FREQUENCY/8.0 );

        /////////////////////////////////////////////
        // Split here between CAB & AFTC
        /////////////////////////////////////////////
	
        if ( state -> SigType == SIGTYPE_DRD_RET_AFTC )
        {
          //   Mod A ==> On Time = 300mS, Period = 360mS
          //   Mod B ==> On Time = 400mS, Period = 460mS
          //   Mod C ==> On Time = 500mS, Period = 560mS

          // 10% longer than 500 ms = 550 ms

          state -> DemodMaxOnCounts = (val32 * 550) / 1000;

          // 10% longer than 60 ms = 66 ms

          state -> DemodMaxOffCounts = (val32 * 66) / 1000;

          // NOTE: All MaxOn times were set to 15% high instead of 10% high
          // because the filter has rise and fall times that skew them upwards.

          state -> AFTC_A_MinPer = (val32 * 324) / 1000;    // SR * .360 *  .9 = SR * .324
          state -> AFTC_A_MaxPer = (val32 * 396) / 1000;    // SR * .360 * 1.1 = SR * .396
          state -> AFTC_B_MinPer = (val32 * 414) / 1000;    // SR * .460 *  .9 = SR * .414
          state -> AFTC_B_MaxPer = (val32 * 504) / 1000;    // SR * .460 * 1.1 = SR * .506
          state -> AFTC_C_MinPer = (val32 * 506) / 1000;    // SR * .560 *  .9 = SR * .504
          state -> AFTC_C_MaxPer = (val32 * 616) / 1000;    // SR * .560 * 1.1 = SR * .616
          state -> AFTC_A_MinOn =  (val32 * 270) / 1000;    // SR * .300 *  .9 = SR * .270
          state -> AFTC_A_MaxOn =  (val32 * 330) / 1000;    // SR * .300 * 1.15 = SR * .330
          state -> AFTC_B_MinOn =  (val32 * 360) / 1000;    // SR * .400 *  .9 = SR * .360
          state -> AFTC_B_MaxOn =  (val32 * 440) / 1000;    // SR * .400 * 1.15 = SR * .440
          state -> AFTC_C_MinOn =  (val32 * 450) / 1000;    // SR * .500 *  .9 = SR * .450 (set a little higher so no overlap with B_MaxOn)
          state -> AFTC_C_MaxOn =  (val32 * 550) / 1000;    // SR * .500 * 1.15 = SR * .550

          state -> DemodState = AFTC_STATE_NOCARRIER;
        }
        else // (gSigType == SIGTYPE_DRD_RET_CAB)
        {
          // Setup a short window (~20ms) for peak detection during modulation
          // that starts ~2ms after a logic '1' is decoded.  This allows
          // current to be measured during modulation.

          state -> DemodState = DRD_RET_CAB_STATE_UNKNOWN;
        }
      }
      break;

      case DRD_PERSONALITY_DRD_CHAR:    // Sound Transit
      {
        float32_t frequency;

        frequency = state -> aDispFreqTab[state -> PresFreqIdx];

        InitializeProcData( state, frequency );

        /////////////////////////////////////////////
        // Split here between CAB & AFTC
        /////////////////////////////////////////////
	
        if ( state -> SigType == SIGTYPE_DRD_CHAR_AFTC )
        {
          //   Mod A ==> On Time = 300mS, Period = 360mS
          //   Mod B ==> On Time = 400mS, Period = 460mS
          //   Mod C ==> On Time = 500mS, Period = 560mS

          // 10% longer than 500 ms = 550 ms

          state -> DemodMaxOnCounts = (val32 * 550) / 1000;

          // 10% longer than 60 ms = 66 ms

          state -> DemodMaxOffCounts = (val32 * 66) / 1000;

          // NOTE: All MaxOn times were set to 15% high instead of 10% high
          // because the filter has rise and fall times that skew them upwards.

          state -> AFTC_A_MinPer = (val32 * 270) / 1000;    // SR * .300 *  .9 = SR * .270
          state -> AFTC_A_MaxPer = (val32 * 330) / 1000;    // SR * .300 * 1.1 = SR * .330
          state -> AFTC_B_MinPer = (val32 * 360) / 1000;    // SR * .400 *  .9 = SR * .360
          state -> AFTC_B_MaxPer = (val32 * 440) / 1000;    // SR * .400 * 1.1 = SR * .440
          state -> AFTC_C_MinPer = (val32 * 450) / 1000;    // SR * .500 *  .9 = SR * .450
          state -> AFTC_C_MaxPer = (val32 * 550) / 1000;    // SR * .500 * 1.1 = SR * .550
          state -> AFTC_A_MinOn =  (val32 * 216) / 1000;    // SR * .240 *  .9 = SR * .216
          state -> AFTC_A_MaxOn =  (val32 * 276) / 1000;    // SR * .240 * 1.15 = SR * .276
          state -> AFTC_B_MinOn =  (val32 * 306) / 1000;    // SR * .340 *  .9 = SR * .306
          state -> AFTC_B_MaxOn =  (val32 * 391) / 1000;    // SR * .340 * 1.15 = SR * .391
          state -> AFTC_C_MinOn =  (val32 * 396) / 1000;    // SR * .440 *  .9 = SR * .396 (set a little higher so no overlap with B_MaxOn)
          state -> AFTC_C_MaxOn =  (val32 * 506) / 1000;    // SR * .440 * 1.15 = SR * .506

          state -> DemodState = AFTC_STATE_NOCARRIER;
        }
        else // (gSigType == SIGTYPE_DRD_CHAR_CAB)
        {
          state -> CabTimes.CHAR.DRD_CHAR_CAB_MaxOn  = (val32 * 555) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_CAB_MaxOff = (val32 * 1598) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_55_MinPer = (val32 * 82) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_55_MaxPer = (val32 * 102) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_45_MinPer = (val32 * 200) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_45_MaxPer = (val32 * 244) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_35_MinPer = (val32 * 300) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_35_MaxPer = (val32 * 368) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_25_MinPer = (val32 * 450) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_25_MaxPer = (val32 * 550) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_15_MinPer = (val32 * 720) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_15_MaxPer = (val32 * 880) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_05_MinPer = (val32 * 132) / 1000;
          state -> CabTimes.CHAR.DRD_CHAR_05_MaxPer = (val32 * 160) / 1000;

          // Setup a short window (~20msCHACHAReak detection during modulation
          // that starts ~2ms after a logic '1' is decoded.  This allows
          // current to be measured during modulation.

          state -> DemodState = DRD_CHAR_CAB_STATE_UNKNOWN;
        }
      }
      break;
    }
  } // end switch(gDRDPersonality)
} // end CalculateNewSettings()

void DoBatteryTest( struct _drd_state_ *state )
{
  uint8_t   val8;
  float32_t fval32;
  float32_t pct;
  char lcdbuf[32];

  // Check for request to disable power-save features.

  if ( !Chip_GPIO_GetPinState( LPC_GPIO, 0, 5 ) )
  {
    state -> PowerSaveEnable = 0;

    LCD_Clrscr( state );
    sprintf( lcdbuf, "Power Save OFF" );
    LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

    while( !Chip_GPIO_GetPinState( LPC_GPIO, 0, 5 ) );
  }

  // Put up splash screen on LCD

  LCD_Clrscr( state );
  sprintf(lcdbuf, "%s   %s", DRDPersTable.Title, VERSION);

  sprintf(output, "\r\n%s",lcdbuf);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
    
  // Read the average from the ADC

  fval32 = GetADCVoltage(ADC_BAT_MONV);

  pct = fval32 - 4.0f;

  if (pct > 2.25f)
  {
    pct = 2.25f;
  }

  if (pct < 0.0)
  {
    pct = 0.0;
  }

  pct /= 2.25;

  pct = pct*100.0f;

  LCD_Gotoxy(state, 1, 2);

  sprintf(lcdbuf,"%s %4.1f%%", "BATT LVL:", pct);

  LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

  sprintf(output, "\r\n%s", lcdbuf);
  SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

  DelayMSecs( 1500 );

  if ( fval32 >= 4.25f )          // about 4.25V
  {
    LCD_Gotoxy(state, 1, 2);

    sprintf(lcdbuf, "BATTERY OK      ");

    sprintf(output, "\r\n%s\r\n", lcdbuf);
    SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

    LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
  }
  else if( fval32 >= 4.0f )     // about 4.0V
  {
    LCD_Gotoxy(state, 1, 2);

    sprintf(lcdbuf, "BATTERY LOW     ");

    sprintf(output, "\r\n%s\r\n",lcdbuf);
    SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

    LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );
  }
  else
  {
    LCD_Gotoxy(state, 1, 1);

    sprintf(lcdbuf, "BATTERY TOO LOW ");

    sprintf(output, "\r\n%s\r\n",lcdbuf);
    SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

    LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

    LCD_Gotoxy(state, 1, 2);

    sprintf( lcdbuf, "POWERING DOWN...");

    sprintf(output, "\r\n%s\r\n", lcdbuf);
    SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

    LCD_Puts( &LCDTransmitRingBuffer, lcdbuf, strlen(lcdbuf) );

    PowerDown();
  }

  DelayMSecs( 1500 );
} // end DoBatteryTest()

void PrintState(struct _drd_state_ *state, int State)
{
  switch ( state -> DRDPersonality )
    {
      case DRD_PERSONALITY_BART:
      {
        switch( State )
        {
          case BART_STATE_NOCARRIER:
            sprintf(output, "NC");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case BART_STATE_CONSTANT:
            sprintf(output, "CC");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
           break;

          case BART_STATE_UNKNOWN_1:
            sprintf(output, "UNK_1");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case BART_STATE_UNKNOWN_0:
            sprintf(output, "UNK_0");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case BART_STATE_1:
            sprintf(output, "S1");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case BART_STATE_0:
            sprintf(output, "S0");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          default:
            sprintf(output, "UH_OH");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;
        }
      }
      break;

      case DRD_PERSONALITY_DRD11:
      {
        switch( State )
        {
          case CAB_STATE_NOCARRIER:
            sprintf(output, "CAB_NOCAR");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_UNKNOWN_1:
            sprintf(output, "CAB_UNK_1");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_UNKNOWN_0:
            sprintf(output, "CAB_UNK_0");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_CONSTANT:
            sprintf(output, "CAB_CONST");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_75_1:
            sprintf(output, "CAB_75_1");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_75_0:
            sprintf(output, "CAB_75_0");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_180_1:
            sprintf(output, "CAB_180_1");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_180_0:
            sprintf(output, "CAB_180_0");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_270_1:
            sprintf(output, "CAB_270_1");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          case CAB_STATE_270_0:
            sprintf(output, "CAB_270_0");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;

          default:
            sprintf(output, "CAB_UH_OH");
            SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
          break;
        }
      }
      break;

      case DRD_PERSONALITY_DRD_ROW:
      {
        if ( state -> SigType == SIGTYPE_DRD_ROW_AFTC)
        {
          switch( State )
          {
            case AFTC_STATE_NOCARRIER:
              sprintf(output, "AFTC_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_1:
              sprintf(output, "AFTC_UNK_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_0:
              sprintf(output, "AFTC_UNK_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_CONSTANT:
              sprintf(output, "AFTC_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_1:
              sprintf(output, "AFTC_C_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_0:
              sprintf(output, "AFTC_C_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_1:
              sprintf(output, "AFTC_B_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_0:
              sprintf(output, "AFTC_B_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_1:
              sprintf(output, "AFTC_A_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_0:
              sprintf(output, "AFTC_A_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "AFTC_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
        else if ( state -> SigType == SIGTYPE_DRD_ROW_CAB)
        {
          switch( State )
          {
            case DRD_ROW_CAB_STATE_CONSTANT:
              sprintf(output, "CAB_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_NOCARRIER:
              sprintf(output, "CAB_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_UNKNOWN:
              sprintf(output, "CAB_UNK");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_70:
              sprintf(output, "CAB_70");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_65:
              sprintf(output, "CAB_65");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_60:
              sprintf(output, "CAB_60");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_50:
              sprintf(output, "CAB_50");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_50LOS:
              sprintf(output, "CAB_50_LOS");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_40:
              sprintf(output, "CAB_40");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_30:
              sprintf(output, "CAB_30");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_20:
              sprintf(output, "CAB_20");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_15:
              sprintf(output, "CAB_15");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_10:
              sprintf(output, "CAB_10");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_10LOS:
              sprintf(output, "CAB_10_LOS");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_ROW_CAB_STATE_0:
              sprintf(output, "CAB_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
	    
            default:
              sprintf(output, "CAB_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
      }
      break;

      case DRD_PERSONALITY_DRD_MFOR:
      {
        if ( state -> SigType == SIGTYPE_DRD_MFOR_AFTC )
        {
          switch( State )
          {
            case AFTC_STATE_NOCARRIER:
              sprintf(output, "AFTC_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_1:
              sprintf(output, "AFTC_UNK_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_0:
              sprintf(output, "AFTC_UNK_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_CONSTANT:
              sprintf(output, "AFTC_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_1:
              sprintf(output, "AFTC_C_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_0:
              sprintf(output, "AFTC_C_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_1:
              sprintf(output, "AFTC_B_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_0:
              sprintf(output, "AFTC_B_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_1:
              sprintf(output, "AFTC_A_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_0:
              sprintf(output, "AFTC_A_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "AFTC_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
        else if ( state -> SigType == SIGTYPE_DRD_MFOR_CAB )
        {
          switch( State )
          {
            case DRD_MFOR_CAB_STATE_CONSTANT:
              sprintf(output, "CAB_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_NOCARRIER:
              sprintf(output, "CAB_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_UNKNOWN:
              sprintf(output, "CAB_UNK");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_80:
              sprintf(output, "CAB_80");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_70:
              sprintf(output, "CAB_70");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_60:
              sprintf(output, "CAB_60");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_50:
              sprintf(output, "CAB_50");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_40:
              sprintf(output, "CAB_40");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_25:
              sprintf(output, "CAB_25");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_10:
              sprintf(output, "CAB_10");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_MFOR_CAB_STATE_0:
              sprintf(output, "CAB_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "CAB_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
      }
      break;

      case DRD_PERSONALITY_DRD_STL:
      {
        if ( state -> SigType == SIGTYPE_DRD_STL_AFTC )
        {
          switch( State )
          {
            case AFTC_STATE_NOCARRIER:
              sprintf(output, "AFTC_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_1:
              sprintf(output, "AFTC_UNK_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_0:
              sprintf(output, "AFTC_UNK_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_CONSTANT:
              sprintf(output, "AFTC_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_1:
              sprintf(output, "AFTC_C_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_0:
              sprintf(output, "AFTC_C_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_1:
              sprintf(output, "AFTC_B_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_0:
              sprintf(output, "AFTC_B_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_1:
              sprintf(output, "AFTC_A_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_0:
              sprintf(output, "AFTC_A_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "AFTC_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
        else if ( state -> SigType == SIGTYPE_DRD_STL_CAB )
        {
          switch( State )
          {
            case DRD_STL_CAB_STATE_CONSTANT:
              sprintf(output, "CAB_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_NOCARRIER:
              sprintf(output, "CAB_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_UNKNOWN:
              sprintf(output, "CAB_UNK");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_55:
              sprintf(output, "CAB_55");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_45:
              sprintf(output, "CAB_45");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_STREET:
              sprintf(output, "CAB_STREET");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_35:
              sprintf(output, "CAB_35");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_25:
              sprintf(output, "CAB_25");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_YARD:
              sprintf(output, "CAB_YARD");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_15:
              sprintf(output, "CAB_15");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_STL_CAB_STATE_5:
              sprintf(output, "CAB_5");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "CAB_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
      }
      break;

      case DRD_PERSONALITY_DRD_RET:
      {
        if ( state -> SigType == SIGTYPE_DRD_RET_AFTC )
        {
          switch( State )
          {
            case AFTC_STATE_NOCARRIER:
              sprintf(output, "AFTC_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_1:
              sprintf(output, "AFTC_UNK_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_0:
              sprintf(output, "AFTC_UNK_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_CONSTANT:
              sprintf(output, "AFTC_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_1:
              sprintf(output, "AFTC_C_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_0:
              sprintf(output, "AFTC_C_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_1:
              sprintf(output, "AFTC_B_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_0:
              sprintf(output, "AFTC_B_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_1:
              sprintf(output, "AFTC_A_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_0:
              sprintf(output, "AFTC_A_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "AFTC_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
        else if ( state -> SigType == SIGTYPE_DRD_RET_CAB )
        {
          switch( State )
          {
            case DRD_RET_CAB_STATE_MULTIPLE:
              sprintf(output, "ATP_MULTI");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_NOCARRIER:
              sprintf(output, "ATP_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_UNKNOWN:
              sprintf(output, "ATP_UNK");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_0A:
              sprintf(output, "ATP_0A");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_10A:
              sprintf(output, "ATP_10A");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_20P:
              sprintf(output, "ATP_20P");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_60G:
              sprintf(output, "ATP_60G");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_35G:
              sprintf(output, "ATP_35G");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_50ROZ:
              sprintf(output, "ATP_50ROZ");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_50ST:
              sprintf(output, "ATP_50ST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_70G:
              sprintf(output, "ATP_70G");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_RET_CAB_STATE_80G:
              sprintf(output, "ATP_80G");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "ATP_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
      }
      break;

      case DRD_PERSONALITY_DRD_CHAR:
      {
        if ( state -> SigType == SIGTYPE_DRD_CHAR_AFTC )
        {
          switch( State )
          {
            case AFTC_STATE_NOCARRIER:
              sprintf(output, "AFTC_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_1:
              sprintf(output, "AFTC_UNK_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_UNKNOWN_0:
              sprintf(output, "AFTC_UNK_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_CONSTANT:
              sprintf(output, "AFTC_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_1:
              sprintf(output, "AFTC_C_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_C_0:
              sprintf(output, "AFTC_C_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_1:
              sprintf(output, "AFTC_B_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_B_0:
              sprintf(output, "AFTC_B_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_1:
              sprintf(output, "AFTC_A_1");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case AFTC_STATE_A_0:
              sprintf(output, "AFTC_A_0");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "AFTC_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
        else if ( state -> SigType == SIGTYPE_DRD_CHAR_CAB )
        {
          switch( State )
          {
            case DRD_CHAR_CAB_STATE_CONSTANT:
              sprintf(output, "CAB_CONST");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_CHAR_CAB_STATE_NOCARRIER:
              sprintf(output, "CAB_NOCAR");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_CHAR_CAB_STATE_UNKNOWN:
              sprintf(output, "CAB_UNK");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_CHAR_CAB_STATE_55:
              sprintf(output, "CAB_55");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_CHAR_CAB_STATE_45:
              sprintf(output, "CAB_45");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_CHAR_CAB_STATE_35:
              sprintf(output, "CAB_35");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_CHAR_CAB_STATE_25:
              sprintf(output, "CAB_25");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_CHAR_CAB_STATE_15:
              sprintf(output, "CAB_15");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            case DRD_CHAR_CAB_STATE_05:
              sprintf(output, "CAB_05");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;

            default:
              sprintf(output, "CAB_UH_OH");
              SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
            break;
          }
        }
      }
      break;

    } // end switch(gDRDPersonality)
} // end PrintState()

uint32_t CalcDutyCycle(uint32_t OnTime, uint32_t TotTime)
{
  uint32_t val32;
  uint32_t rem32;

  if ( TotTime == 0 )
  {
    return 100;
  }
    
  val32 = OnTime * 100;
  rem32 = val32 % TotTime;
  val32 /= TotTime;
    
  if ( rem32 >= (TotTime >> 1) )
  {
    val32++;
  }

  return val32;
} // end CalcDutyCycle()

////////////////////////////////////////////////////////////////////////////////
// Function: GetCorrectAmpsForDisplay
//
// Summary:  Calculates the correct amps value and places an ASCII version into
//           dispbuf.  This is where offset and gain corrections are applied as
//           well as the correct algorithm for converting ADC units to amps.
////////////////////////////////////////////////////////////////////////////////

float32_t GetCorrectAmpsForDisplay( struct _drd_state_ *state, char *dispbuf, uint32_t ampsval )
{
  uint8_t numbuf[12];
  uint32_t val32 = 0;
  uint32_t rem32;
  float val32f;
  int i;

  switch ( state -> DRDPersonality )
  {
    case DRD_PERSONALITY_BART:
    {
      // BART needs 1000 mApp dynamic range or 500 mA pk
      // the 14-bit ADC has +/- 4V out of its +/- 5V range useable or 80%
      // so if a p-p input of 1000 mA is set to enter the ADC at +/- 4V or 8V p-p
      // then 1000 mA pp should produce (16384 X .8) ADC units = 13107.2
      // the peak detection in uni-polar so gPeakAvg will be half of that for 1000 mApp
      // or 13107.2 / 2 = 6553.6 = 500 mA pk
      // so adjust input gain so 500 mA pk is about 3.66V at the ADC
      // then the scaling from ADC units to mA pk will be:
      //     (3.66 / 5V ) X 8192 ADCU = 6000 ADCU for 500 mA pk or 1000 mA pp
      // to get the 1000 mA pp from 6000 just divide by 6
      //
      // So if 3.66V represents 1000 mA pp or 500 mA pk then the required gain must
      // be 3.66V / .05V = 73.2.
      //
      // Bart uses peak-to-peak.  The prototype rail sensor's scale factor is about
      // 0.11 V/A, reference attached. 0.5 App would yield a 55 mVpp signal. 
      // Sam is now saying he will round this all to .1 V/A.
      ////////////////////////////////////////////////////////////////////////////

      // Apply offset

      if ( ampsval >= state -> PresAmpsOffsetCal )
      {
        ampsval -= state -> PresAmpsOffsetCal;
      }
      else
      {
        ampsval = 0;
      }

      if ( state -> SigType == SIGTYPE_BARTSC )
      {
        val32 = ampsval * 1917;      // 1844;
        rem32 = val32 % 10000;
        val32 /= 10000;

        if ( rem32 >= 5000 )
        {
          val32++;
        }

        //val32= ampsval / 6;
        //rem32= ampsval % 6;
        //if(rem32 >= 3)      // round
        //    val32++;

        // scale by calibration factor
	
        val32f = (val32 * state -> PresAmpsGainCal) / 1000.0;

        //isprintf(dispbuf,"%4d %s", val32, gPersTable[gDRDPersonality].AmpsUnits1);
        sprintf(dispbuf, "%3.3f", val32f);
      }
      else if ( state -> SigType == SIGTYPE_BARTMV )
      {
        //val32= ampsval * 928;     // yields approx mVpk
        val32 = ampsval * 653;       // yields approx mVrms
        rem32 = val32 % 10000;
        val32 /= 10000;

        if ( rem32 >= 5000 )
        {
          val32++;
        }

        // scale by calibration factor
	
        val32f = (val32 * state -> PresAmpsGainCal) / 1000.0;

        //isprintf(dispbuf,"%4d %s", val32, gPersTable[gDRDPersonality].AmpsUnits1);
        //isprintf(dispbuf,"%4d", val32);
        //UIF32ToAsciiFloat(val32, numbuf, 5, 1, 1);
        //isprintf(dispbuf,"%s",numbuf);
	sprintf( dispbuf, "%3.3", val32f );
      }
    }
    break;

    case DRD_PERSONALITY_DRD11:
    {
      // DRD-11 uses RMS (of peak).  The rail sensors are calibrated to .01V/A so
      // a 10A RMS signal will be 14.1A pk or .141V pk (14.1A X .01 = .141V pk).
      //
      // Must accommodate 21.5A dynamic range.  This is from 2 10A carriers that
      // could sum to 20A + another 1dB which is around 21.5A.
      // 21.5A RMS is 30.41A pk.  This is an input signal level of .3041V pk from
      // the sensor.  Front end gain will be fixed at 12.1 so the signal level
      // entering the filters is .3041 X 12.1 = 3.68V pk.  The available dynamic
      // range is around 3.8V pk so this allows still a little headroom.
      //
      // 3.68V after the filters would be (3.68V / 5V) X 8192 = 6029 ADC units.
      // Multiply ADC units by .344 to convert to amps RMS so
      //   6029 * .344 = 20.74 A rms
      // The SW calibration is used to get this to exactly 21.5A.
      //
      // Calibration of rail sensors will be done likely at 5A rms.
      // 5A rms = 7.072 A pk  :  This will be .07072V pk at the inputs.
      // .07072V pk X 12.1 gain = .8557V thru the filters and into the ADC.
      // (.8557V / 5.000V) X 8192 = 1402 ADC units.
      // 1402 ADCU X .344 = 482 which represents 4.82 A rms and SW cal will fix
      // it to 5.00A.  .357 might be a better number than .344.
      ////////////////////////////////////////////////////////////////////////////

      // Apply offset

      if ( ampsval >= state -> PresAmpsOffsetCal )
      {
        ampsval -= state -> PresAmpsOffsetCal;
      }
      else
      {
        ampsval = 0;
      }

      //val32= (state -> PeakAvg * 2533) / 10000;
      val32 = ampsval * 344;    // divide ADC by 2.907 (mult by .344)
      rem32 = val32 % 1000;
      val32 = val32 / 1000;

      if ( rem32 >= 500 )        // see if needs rounded up
      {
        val32++;
      }
        
      // scale by calibration factor

      val32f = (val32 * state -> PresAmpsGainCal) / 1000.0;

      //if(val32 < 10)
      //    val32= 0;

      // for DRD11 (RTD) displayed amps will be from 0.00 to 10.00
      // this is represented by 0 to 1000 in val32
      //UIF32ToAsciiFloat(val32, numbuf, 5, 2, 1);
      //isprintf(lcdbuf,"%4d Arms",val32);
      //isprintf(dispbuf,"%s %s",numbuf, gPersTable[gDRDPersonality].AmpsUnits1);
      //isprintf(dispbuf,"%s",numbuf);

      sprintf(dispbuf, "%3.3f", val32f );
    }
    break;

    case DRD_PERSONALITY_DRD_ROW:
    {
      // The ADC has a full scale range of +-5V. The sample value passed in (ampsval)
      // represents the averaged value of the rectified peak voltage from the ADC.
      // 
      // The ADC is 14-bits with the MSB being a sign bit, therefore each
      // bit from the A/D represents 0.610mV of peak voltage. ==> +5V / (2^13 - 1)
      //
      // This system uses a .2ohm shunt. (in parallel with 0.5ohm for an equivalent 
      // resistance of 0.1429ohm.)
      //
      // Vpeak * 0.7071 = Vrms
      //
      // ampsval * 0.61 * 0.707  = Vrms,   (Vrms / 0.2) = Scaled Irms
      // ampsval * 2.16 = Irms
      //
      ////////////////////////////////////////////////////////////////////////////


      // Apply offset

      if ( ampsval >= state -> PresAmpsOffsetCal )
      {
        ampsval -= state -> PresAmpsOffsetCal;
      }
      else
      {
        ampsval = 0;
      }

      // Scale value here is primarily based on above formula and then
      // adjusted as needed to get the desired measurement in the lab.

      val32 = ampsval * 216;    // for 0.20 Shunt.

      // The DRD-ROW signals are gained by 4 to attain desired decoding
      // thresholds. (need to adjust the displayed value back down.)

      val32 = val32 / 4;

      rem32 = val32 % 100;
      val32 = val32 / 100;

      if ( rem32 >= 50 )        // see if needs rounded up
      {
        val32++;
      }

      // scale by calibration factor

      val32f = (val32 * state -> PresAmpsGainCal) / 1000.0;

      // for DRD-ROWT displayed amps will be from 0.00 to 10.00
      // this is represented by 0 to 1000 in val32
      
      // UIF32ToAsciiFloat(val32, numbuf,6, 3, 1);
      // isprintf(dispbuf,"%s",numbuf);
      sprintf( dispbuf, "3.3f", val32f );
    }
    break;

    case DRD_PERSONALITY_DRD_MFOR:
    {
      // The ADC has a full scale range of +-5V. The sample value passed in (ampsval)
      // represents the averaged value of the rectified peak voltage from the ADC.
      // 
      // The ADC is 14-bits with the MSB being a sign bit, therefore each
      // bit from the A/D represents 0.610mV of peak voltage. ==> +5V / (2^13 - 1)
      //
      // This system uses a .2ohm shunt. (in parallel with 0.5ohm for an equivalent 
      // resistance of 0.1429ohm.)
      //
      // Vpeak * 0.7071 = Vrms
      //
      // ampsval * 0.61 * 0.707  = Vrms,   (Vrms / 0.2) = Scaled Irms
      // ampsval * 2.16 = Irms
      //
      ////////////////////////////////////////////////////////////////////////////

      if ( !state -> Calibrating && (((state -> SigType == SIGTYPE_DRD_MFOR_AFTC) && ((state -> DemodState == AFTC_STATE_NOCARRIER) ||
	 				                                (state -> DemodState == AFTC_STATE_UNKNOWN_0) ||
						                        (state -> DemodState == AFTC_STATE_UNKNOWN_1))) ||

                               ((state -> SigType == SIGTYPE_DRD_MFOR_CAB) && ((state -> DemodState == DRD_MFOR_CAB_STATE_UNKNOWN) ||
	                                                               (state -> DemodState == DRD_MFOR_CAB_STATE_NOCARRIER)))) )
      {
        val32 = 0;

	sprintf( dispbuf, "UNKNOWN" );
      }
      else
      {
        // Apply offset
	
        if ( ampsval >= state -> PresAmpsOffsetCal)
        {
          ampsval -= state -> PresAmpsOffsetCal;
        }
        else
        {
          ampsval = 0;
        }

        // Scale value here is primarily based on above formula and then
        // adjusted as needed to get the desired measurement in the lab.
	
        val32 = ampsval * 206;    // for 0.20 Shunt.

        // The DRD-MFOR signals are gained by 4 to attain desired decoding
        // thresholds. (need to adjust the displayed value back down.)
	
        val32 = val32 / 4;

        rem32 = val32 % 100;
        val32 = val32 / 100;

        if ( rem32 >= 50 )        // see if needs rounded up
        {
          val32++;
        }

        // scale by calibration factor
	
        val32f = (val32 * state -> PresAmpsGainCal)/1000.0;

	if ( val32f >= 2.600 )
	{
          sprintf( dispbuf, " OVER  " );
	}
	else
	{
          // for DRD-MFOR displayed amps will be from 0.00 to 10.00
          // this is represented by 0 to 1000 in val32
          //UIF32ToAsciiFloat(val32, numbuf,6, 3, 1);
          //isprintf(dispbuf,"%s ",numbuf);

	  sprintf( dispbuf, "%3.3f  ", val32f );
	}
      }
    }
    break;

    case DRD_PERSONALITY_DRD_STL:
    {
      // The ADC has a full scale range of +-5V. The sample value passed in (ampsval)
      // represents the averaged value of the rectified peak voltage from the ADC.
      // 
      // The ADC is 14-bits with the MSB being a sign bit, therefore each
      // bit from the A/D represents 0.610mV of peak voltage. ==> +5V / (2^13 - 1)
      //
      // This system uses a .2ohm shunt. (in parallel with 0.5ohm for an equivalent 
      // resistance of 0.1429ohm.)
      //
      // Vpeak * 0.7071 = Vrms
      //
      // ampsval * 0.61 * 0.707  = Vrms,   (Vrms / 0.2) = Scaled Irms
      // ampsval * 2.16 = Irms
      //
      ////////////////////////////////////////////////////////////////////////////

      i = ReadShuntType();

      if ( i != SENSDET_200OHM )
      {
        sprintf( dispbuf, "Check Shunt      " );
      }
      else if ( !state -> Calibrating && (((state -> SigType == SIGTYPE_DRD_STL_AFTC) && ((state -> DemodState == AFTC_STATE_NOCARRIER) ||
	 				                                (state -> DemodState == AFTC_STATE_UNKNOWN_0) ||
						                        (state -> DemodState == AFTC_STATE_UNKNOWN_1))) ||

                               ((state -> SigType == SIGTYPE_DRD_STL_CAB) && ((state -> DemodState == DRD_STL_CAB_STATE_UNKNOWN) ||
	                                                               (state -> DemodState == DRD_STL_CAB_STATE_NOCARRIER)))) )
      {
        val32 = 0;

	sprintf( dispbuf, "UNKNOWN    " );
      }
      else
      {
        // Apply offset
	
        BuildBaseDisplay( state, MODE_MEAS, 1, dispbuf );

        if ( ampsval >= state -> PresAmpsOffsetCal)
        {
          ampsval -= state -> PresAmpsOffsetCal;
        }
        else
        {
          ampsval = 0;
        }

        // Scale value here is primarily based on above formula and then
        // adjusted as needed to get the desired measurement in the lab.
	
        val32f = ampsval * 687;    // for 0.20 Shunt.

        // The DRD-STL signals are gained by 4 to attain desired decoding
        // thresholds. (need to adjust the displayed value back down.)
	
        val32f = val32f / 400;

        // scale by calibration factor
	
        val32f = (val32f * state -> PresAmpsGainCal)/1000.0;

	if ( val32f >= 6.000 )
	{
          sprintf( dispbuf, " OVER " );
	}
	else
	{
          // for DRD-STL displayed amps will be from 0.00 to 10.00
          // this is represented by 0 to 1000 in val32
          //UIF32ToAsciiFloat(val32, numbuf,6, 3, 1);
          //isprintf(dispbuf,"%s ",numbuf);

	  sprintf( dispbuf, "%3.3f  ", val32f );
	}
      }

      dispbuf[strlen(dispbuf)] = ' ';
    }
    break;

    case DRD_PERSONALITY_DRD_RET:
    {
      // The ADC has a full scale range of +-5V. The sample value passed in (ampsval)
      // represents the averaged value of the rectified peak voltage from the ADC.
      // 
      // The ADC is 14-bits with the MSB being a sign bit, therefore each
      // bit from the A/D represents 0.610mV of peak voltage. ==> +5V / (2^13 - 1)
      //
      // This system uses a .2ohm shunt. (in parallel with 0.5ohm for an equivalent 
      // resistance of 0.1429ohm.)
      //
      // Vpeak * 0.7071 = Vrms
      //
      // ampsval * 0.61 * 0.707  = Vrms,   (Vrms / 0.2) = Scaled Irms
      // ampsval * 2.16 = Irms
      //
      ////////////////////////////////////////////////////////////////////////////

      if ( !state -> Calibrating && (((state -> SigType == SIGTYPE_DRD_RET_AFTC) && ((state -> DemodState == AFTC_STATE_NOCARRIER) ||
	 				                                             (state -> DemodState == AFTC_STATE_UNKNOWN_0) ||
						                                     (state -> DemodState == AFTC_STATE_UNKNOWN_1))) ||

                               ((state -> SigType == SIGTYPE_DRD_RET_CAB) && (state -> DemodState == DRD_RET_CAB_STATE_NOCARRIER))) )
      {
        val32 = 0;

	sprintf( dispbuf, "0.00   " );
      }
      else
      {
        // Apply offset
	
        if ( ampsval >= state -> PresAmpsOffsetCal)
        {
          ampsval -= state -> PresAmpsOffsetCal;
        }
        else
        {
          ampsval = 0;
        }

        // Scale value here is primarily based on above formula and then
        // adjusted as needed to get the desired measurement in the lab.
	
        
        if ( state -> SigType == SIGTYPE_DRD_RET_CAB )
        {
          val32f = ampsval * 575;    // for 0.20 Shunt.
        }
        else
        {
          val32f = ampsval * 282;    // for 0.20 Shunt.
        }

        // The DRD-STL signals are gained by 4 to attain desired decoding
        // thresholds. (need to adjust the displayed value back down.)
	
        val32f = val32f / 400;

        // scale by calibration factor
	
        val32f = (val32f * state -> PresAmpsGainCal)/1000.0;

        if ( state -> SigType == SIGTYPE_DRD_RET_CAB )
        {
	  if ( val32f > 20.000 )
	  {
            sprintf( dispbuf, " OVER  " );
	  }
	  else
	  {
            // for DRD-RET displayed amps will be from 0.00 to 10.00
            // this is represented by 0 to 1000 in val32
            //UIF32ToAsciiFloat(val32, numbuf,6, 3, 1);
            //isprintf(dispbuf,"%s ",numbuf);

	    sprintf( dispbuf, "%3.2f   ", val32f );
	  }
        }
        else
        {
	  if ( val32f > 10.000 )
	  {
            sprintf( dispbuf, " OVER  " );
	  }
	  else
	  {
            // for DRD-RET displayed amps will be from 0.00 to 10.00
            // this is represented by 0 to 1000 in val32
            //UIF32ToAsciiFloat(val32, numbuf,6, 3, 1);
            //isprintf(dispbuf,"%s ",numbuf);

	    sprintf( dispbuf, "%3.2f   ", val32f );
	  }
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_CHAR:
    {
      // The ADC has a full scale range of +-5V. The sample value passed in (ampsval)
      // represents the averaged value of the rectified peak voltage from the ADC.
      // 
      // The ADC is 14-bits with the MSB being a sign bit, therefore each
      // bit from the A/D represents 0.610mV of peak voltage. ==> +5V / (2^13 - 1)
      //
      // This system uses a .2ohm shunt. (in parallel with 0.5ohm for an equivalent 
      // resistance of 0.1429ohm.)
      //
      // Vpeak * 0.7071 = Vrms
      //
      // ampsval * 0.61 * 0.707  = Vrms,   (Vrms / 0.2) = Scaled Irms
      // ampsval * 2.16 = Irms
      //
      ////////////////////////////////////////////////////////////////////////////

      if ( !state -> Calibrating && (((state -> SigType == SIGTYPE_DRD_CHAR_AFTC) && ((state -> DemodState == AFTC_STATE_NOCARRIER) ||
	 				                                (state -> DemodState == AFTC_STATE_UNKNOWN_0) ||
						                        (state -> DemodState == AFTC_STATE_UNKNOWN_1))) ||

                               ((state -> SigType == SIGTYPE_DRD_CHAR_CAB) && ((state -> DemodState == DRD_CHAR_CAB_STATE_UNKNOWN) ||
	                                                               (state -> DemodState == DRD_CHAR_CAB_STATE_NOCARRIER)))) )
      {
        val32 = 0;

	sprintf( dispbuf, "UNKNOWN" );
      }
      else
      {
        // Apply offset
	
        if ( ampsval >= state -> PresAmpsOffsetCal)
        {
          ampsval -= state -> PresAmpsOffsetCal;
        }
        else
        {
          ampsval = 0;
        }

        // Scale value here is primarily based on above formula and then
        // adjusted as needed to get the desired measurement in the lab.
	
        val32 = ampsval * 165;    // for 0.25 Shunt.

        // The DRD-CHAR signals are gained by 4 to attain desired decoding
        // thresholds. (need to adjust the displayed value back down.)
	
        val32 = val32 / 4;

        rem32 = val32 % 100;
        val32 = val32 / 100;

        if ( rem32 >= 50 )        // see if needs rounded up
        {
          val32++;
        }

        // scale by calibration factor
	
        val32f = (val32 * state -> PresAmpsGainCal)/1000.0;

	if ( val32f >= 2.600 )
	{
          sprintf( dispbuf, " OVER  " );
	}
	else
	{
          // for DRD-CHAR displayed amps will be from 0.00 to 10.00
          // this is represented by 0 to 1000 in val32
          //UIF32ToAsciiFloat(val32, numbuf,6, 3, 1);
          //isprintf(dispbuf,"%s ",numbuf);

	  sprintf( dispbuf, "%3.3f  ", val32f );
	}
      }
    }
    break;
  } // end switch(gDRDPersonality)

  return val32f;
} // end GetCorrectAmpsForDisplay()

////////////////////////////////////////////////////////////////////////////////
// Function: GetCorrectCodeForDisplay
//
// Summary:  Places the correct modulation code into dispbuf.
//
////////////////////////////////////////////////////////////////////////////////

void GetCorrectCodeForDisplay( struct _drd_state_ *state, char *dispbuf )
{
  char   c;
  int    old_ipl;
  int    i;
//    int    f1;
//    int    f2;
//    int    idx;
  uint32_t demodsr;
//    uint32_t mask;

  switch( state -> DRDPersonality )
  {
    // BART CIRCA
    case DRD_PERSONALITY_BART:
    {
      // Lowest 4 freqs do not have modulation

      if ( state -> PresFreqIdx < 4 )
      {
        sprintf(dispbuf, "                ");
        return;
      }

      // if on a 0's freq, must invert

      if ( (state -> PresFreqIdx > 3) && ((state -> PresFreqIdx & 1) == 0) )
      {
        demodsr = ~demodsr;
      }

      // only care about 6 bits

      demodsr &= 0x3F;

      // look-up the speed code

      c = BART_SpeedCodeLUT[demodsr];
      i = c;

      if ( state -> DemodState == BART_STATE_CONSTANT )          // if there is a constant carrier, no phase flips
      {
        if ( (state -> PresFreqIdx & 1) == 0 )                 //     if on a 0's freq
        {
          sprintf(dispbuf, "ALL 0's NO PHASE" );
        }
        else                                        //     else if on a 1's freq
        {
          sprintf(dispbuf, "ALL 1's NO PHASE" );
        }
      }
      else if ( state -> DemodState == BART_STATE_NOCARRIER )    // else if there is no carrier
      {
        sprintf(dispbuf, "NO SIGNAL       " );
      }
      else if ( i == -1 )                                // else if speed code does not match know code
      {
        sprintf(dispbuf, "UNKNOWN CODE    ");
      }
      else if ( i == -2 )                                // else if all 1's on a 0's freq or all 0's on a 1's freq
      {
        sprintf(dispbuf, "ALL 0's PHASE ON" );
      }
      else if ( i == -3 )                                // else if all 0's on a 0's freq or all 1's on a 1's freq
      {
        sprintf(dispbuf, "ALL 1's PHASE ON" );
      }
      else                                            // else display the actual speed code
      {
        sprintf(dispbuf, "%2ld MPH          ", i );
      }
    }
    break;

    // DRD-11 (Denver RTD)

    case DRD_PERSONALITY_DRD11:
    {
      if ( state -> DemodState == CAB_STATE_CONSTANT )
      {
        sprintf(dispbuf, "CC " );
      }
      else if ( state -> DemodState == CAB_STATE_NOCARRIER )
      {
        sprintf(dispbuf, "NC " );
      }
      else if ( state -> DemodState & CAB_STATE_UNKNOWN_0 )
      {
        sprintf(dispbuf, "UN " );
      }
      else if ( state -> DemodState & CAB_STATE_75_0 )
      {
        sprintf(dispbuf, " 75 " );
      }
      else if ( state -> DemodState & CAB_STATE_180_0 )
      {
        sprintf(dispbuf, "180" );
      }
      else if ( state -> DemodState & CAB_STATE_270_0 )
      {
        sprintf(dispbuf, "270" );
      }
      else
      {
        sprintf(dispbuf, "ST=%02lX", state -> DemodState );   // should never happen, would like to know if it does!
      }
    }
    break;

    // DRD-ROW

    case DRD_PERSONALITY_DRD_ROW:
    {
      if (state -> SigType == SIGTYPE_DRD_ROW_AFTC )
      {
        if ( state -> DemodState & AFTC_STATE_A_0 )
        {
          sprintf(dispbuf, "MOD A");
        }
        else if ( state -> DemodState & AFTC_STATE_B_0 )
        {
          sprintf(dispbuf, "MOD B");
        }
        else if ( state -> DemodState & AFTC_STATE_C_0 )
        {
          sprintf(dispbuf, "MOD C");
        }
        else
        {
          sprintf(dispbuf, "Error");
        }
      }
      else // (gSigType == SIGTYPE_DRD_ROW_CAB)
      {
        switch (state -> DemodState) // Cab LCD Display
        {
          case DRD_ROW_CAB_STATE_CONSTANT:
            sprintf(dispbuf, "CONSTANT  ");
          break;

          case DRD_ROW_CAB_STATE_NOCARRIER:
            sprintf(dispbuf, "NO CARRIER");
          break;

          case DRD_ROW_CAB_STATE_UNKNOWN:
            sprintf(dispbuf, "UNKNOWN   ");
          break;

          case DRD_ROW_CAB_STATE_70:
            sprintf(dispbuf, "70 KPH    ");
          break;

          case DRD_ROW_CAB_STATE_65:
            sprintf(dispbuf, "65 KPH    ");
          break;

          case DRD_ROW_CAB_STATE_60:
            sprintf(dispbuf, "60 KPH    ");
          break;
	  
          case DRD_ROW_CAB_STATE_50:
            sprintf(dispbuf, "50 KPH    ");
          break;

          case DRD_ROW_CAB_STATE_50LOS:
            sprintf(dispbuf, "50 LOS    ");
          break;

          case DRD_ROW_CAB_STATE_40:
            sprintf(dispbuf, "40 KPH    ");
          break;

          case DRD_ROW_CAB_STATE_30:
            sprintf(dispbuf, "30 KPH    ");
          break;

          case DRD_ROW_CAB_STATE_20:
            sprintf(dispbuf, "20 KPH    ");
          break;

          case DRD_ROW_CAB_STATE_15:
            sprintf(dispbuf, "15 KPH    ");
          break;

          case DRD_ROW_CAB_STATE_10:
            sprintf(dispbuf, "10 KPH    ");
          break;

          case DRD_ROW_CAB_STATE_10LOS:
            sprintf(dispbuf, "10 LOS    ");
          break;

          case DRD_ROW_CAB_STATE_0:
            sprintf(dispbuf, "0 KPH     ");
          break;

          default:
            sprintf(dispbuf, "ST=%X    ", state -> DemodState);   // should never happen, would like to know if it does!
          break;
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_MFOR:
    {
      if ( state -> SigType == SIGTYPE_DRD_MFOR_AFTC )
      {
        if ( state -> DemodState & AFTC_STATE_A_0 )
        {
          sprintf(dispbuf, "MOD A   ");
        }
        else if ( state -> DemodState & AFTC_STATE_B_0 )
        {
          sprintf(dispbuf, "MOD B   ");
        }
        else if ( state -> DemodState & AFTC_STATE_C_0 )
        {
          sprintf(dispbuf, "MOD C   ");
        }
	else if ( state -> DemodState == AFTC_STATE_CONSTANT )
	{
          sprintf(dispbuf, "CONSTANT  ");
	}
        else
        {
          sprintf(dispbuf, "UNKNOWN   ");
        }
      }
      else // (gSigType == SIGTYPE_DRD_MFOR_CAB)
      {
        switch ( state -> DemodState ) // Cab LCD Display
        {
          case DRD_MFOR_CAB_STATE_CONSTANT:
            sprintf(dispbuf, "CONSTANT  ");
          break;

          case DRD_MFOR_CAB_STATE_NOCARRIER:
            sprintf(dispbuf, "NO CODE   ");
          break;

          case DRD_MFOR_CAB_STATE_UNKNOWN:
            sprintf(dispbuf, "UNKNOWN   ");
          break;

          case DRD_MFOR_CAB_STATE_80:
            sprintf(dispbuf, "80 KPH    ");
          break;

          case DRD_MFOR_CAB_STATE_70:
            sprintf(dispbuf, "70 KPH    ");
          break;

          case DRD_MFOR_CAB_STATE_60:
            sprintf(dispbuf, "60 KPH    ");
          break;

          case DRD_MFOR_CAB_STATE_50:
            sprintf(dispbuf, "50 KPH    ");
          break;

          case DRD_MFOR_CAB_STATE_40:
            sprintf(dispbuf, "40 KPH    ");
          break;

          case DRD_MFOR_CAB_STATE_25:
            sprintf(dispbuf, "25 KPH    ");
          break;

          case DRD_MFOR_CAB_STATE_10:
            sprintf(dispbuf, "10 KPH    ");
          break;

          case DRD_MFOR_CAB_STATE_0:
            sprintf(dispbuf, "0 KPH     ");
          break;

          default:
            sprintf(dispbuf, "ST=%X    ", state -> DemodState);   // should never happen, would like to know if it does!
          break;
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_STL:
    {
      if ( state -> SigType == SIGTYPE_DRD_STL_AFTC )
      {
        if ( state -> DemodState & AFTC_STATE_A_0 )
        {
          sprintf(dispbuf, "MOD A   ");
        }
        else if ( state -> DemodState & AFTC_STATE_B_0 )
        {
          sprintf(dispbuf, "MOD B   ");
        }
        else if ( state -> DemodState & AFTC_STATE_C_0 )
        {
          sprintf(dispbuf, "MOD C   ");
        }
	else if ( state -> DemodState == AFTC_STATE_CONSTANT )
	{
          sprintf(dispbuf, "CONSTANT  ");
	}
        else
        {
          sprintf(dispbuf, "UNKNOWN   ");
        }
      }
      else // (gSigType == SIGTYPE_DRD_STL_CAB)
      {
        switch ( state -> DemodState ) // Cab LCD Display
        {
          case DRD_STL_CAB_STATE_CONSTANT:
            sprintf(dispbuf, "CONSTANT  ");
          break;

          case DRD_STL_CAB_STATE_NOCARRIER:
            sprintf(dispbuf, "NO CODE   ");
          break;

          case DRD_STL_CAB_STATE_UNKNOWN:
            sprintf(dispbuf, "UNKNOWN   ");
          break;

          case DRD_STL_CAB_STATE_55:
            sprintf(dispbuf, "55 MPH    ");
          break;

          case DRD_STL_CAB_STATE_45:
            sprintf(dispbuf, "45 MPH    ");
          break;

          case DRD_STL_CAB_STATE_STREET:
            sprintf(dispbuf, "STREET    ");
          break;

          case DRD_STL_CAB_STATE_35:
            sprintf(dispbuf, "35 MPH    ");
          break;

          case DRD_STL_CAB_STATE_25:
            sprintf(dispbuf, "25 MPH    ");
          break;

          case DRD_STL_CAB_STATE_YARD:
            sprintf(dispbuf, "YARD      ");
          break;

          case DRD_STL_CAB_STATE_15:
            sprintf(dispbuf, "15 MPH    ");
          break;

          case DRD_STL_CAB_STATE_5:
            sprintf(dispbuf, "5 MPH     ");
          break;

          default:
            sprintf(dispbuf, "ST=%X    ", state -> DemodState);   // should never happen, would like to know if it does!
          break;
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_RET:
    {
      if ( state -> SigType == SIGTYPE_DRD_RET_AFTC )
      {
        if ( state -> DemodState & AFTC_STATE_A_0 )
        {
          sprintf(dispbuf, "MOD A   ");
        }
        else if ( state -> DemodState & AFTC_STATE_B_0 )
        {
          sprintf(dispbuf, "MOD B   ");
        }
        else if ( state -> DemodState & AFTC_STATE_C_0 )
        {
          sprintf(dispbuf, "MOD C   ");
        }
	else if ( state -> DemodState == AFTC_STATE_CONSTANT )
	{
          sprintf(dispbuf, "Error     ");
	}
        else
        {
          sprintf(dispbuf, "Error     ");
        }
      }
      else // (gSigType == SIGTYPE_DRD_RET_CAB)
      {
        state -> PresDispFreq = state -> aDispFreqTab[state -> PresFreqIdx];
        state -> PresDispFreq1 = state -> aDispFreqTab[state -> PresFreq1Idx];

        switch ( state -> DemodState ) // Cab LCD Display
        {
          case DRD_RET_CAB_STATE_MULTIPLE:
            sprintf(dispbuf, "-MC-            ", state -> PresDispFreq, state -> PresDispFreq1 );
          break;

          case DRD_RET_CAB_STATE_NOCARRIER:
            sprintf(dispbuf, "---- 000HZ 000HZ", state -> PresDispFreq, state -> PresDispFreq1 );
          break;

          case DRD_RET_CAB_STATE_PRIMARY:
            sprintf(dispbuf, "---- %3dHZ 000HZ", state -> PresDispFreq );
          break;

          case DRD_RET_CAB_STATE_UNKNOWN:
            sprintf(dispbuf, "---- 000HZ 000HZ", state -> PresDispFreq, state -> PresDispFreq1 );
          break;

          case DRD_RET_CAB_STATE_0A:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-01- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-01- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_10A:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-02- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-02- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_20P:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-03- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-03- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_60G:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-04- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-04- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_35G:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-05- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-05- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_50ROZ:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-06- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-06- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_50ST:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-07- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-07- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_50G:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-08- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-08- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_70G:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-09- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-09- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          case DRD_RET_CAB_STATE_80G:
            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf(dispbuf, "-10- %3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq );
            }
            else
            {
              sprintf(dispbuf, "-10- %3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1 );
            }
          break;

          default:
            sprintf(dispbuf, "ST=%X    ", state -> DemodState);   // should never happen, would like to know if it does!
          break;
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_CHAR:
    {
      if ( state -> SigType == SIGTYPE_DRD_CHAR_AFTC )
      {
        if ( state -> DemodState & AFTC_STATE_A_0 )
        {
          sprintf(dispbuf, "MOD A   ");
        }
        else if ( state -> DemodState & AFTC_STATE_B_0 )
        {
          sprintf(dispbuf, "MOD B   ");
        }
        else if ( state -> DemodState & AFTC_STATE_C_0 )
        {
          sprintf(dispbuf, "MOD C   ");
        }
	else if ( state -> DemodState == AFTC_STATE_CONSTANT )
	{
          sprintf(dispbuf, "CONSTANT  ");
	}
        else
        {
          sprintf(dispbuf, "UNKNOWN   ");
        }
      }
      else // (gSigType == SIGTYPE_DRD_CHAR_CAB)
      {
        switch ( state -> DemodState ) // Cab LCD Display
        {
          case DRD_CHAR_CAB_STATE_CONSTANT:
            sprintf(dispbuf, "CONSTANT  ");
          break;

          case DRD_CHAR_CAB_STATE_NOCARRIER:
            sprintf(dispbuf, "NO CODE   ");
          break;

          case DRD_CHAR_CAB_STATE_UNKNOWN:
            sprintf(dispbuf, "UNKNOWN   ");
          break;

          case DRD_CHAR_CAB_STATE_55:
            sprintf(dispbuf, "55 MPH    ");
          break;

          case DRD_CHAR_CAB_STATE_45:
            sprintf(dispbuf, "45 MPH    ");
          break;

          case DRD_CHAR_CAB_STATE_35:
            sprintf(dispbuf, "35 MPH    ");
          break;

          case DRD_CHAR_CAB_STATE_25:
            sprintf(dispbuf, "25 MPH    ");
          break;

          case DRD_CHAR_CAB_STATE_15:
            sprintf(dispbuf, "15 MPH    ");
          break;

          case DRD_CHAR_CAB_STATE_05:
            sprintf(dispbuf, " 5 MPH    ");
          break;

          default:
            sprintf(dispbuf, "ST=%X    ", state -> DemodState);   // should never happen, would like to know if it does!
          break;
        }
      }
    }
    break;
  } // end switch(gDRDPersonality)
} // end GetCorrectCodeForDisplay()

void GetCorrectDutyForDisplay( struct _drd_state_ *state, char *dispbuf )
{
  switch ( state -> DRDPersonality )
  {
    case DRD_PERSONALITY_BART:
    {
      // BART does not display duty cycle

      *dispbuf = 0;
    }
    break;

    case DRD_PERSONALITY_DRD11:
    {
      sprintf(dispbuf, "%3d", state -> DemodDuty);
    }
    break;

    case DRD_PERSONALITY_DRD_ROW:
    {
      // DRD_ROW does not display duty cycle.

      *dispbuf = 0;
    }
    break;

    case DRD_PERSONALITY_DRD_CHAR:
    case DRD_PERSONALITY_DRD_MFOR:
    {
      // DRD_MFOR does not display duty cycle.

      *dispbuf = 0;
    }
    break;

    case DRD_PERSONALITY_DRD_STL:
    {
      // DRD_STL does not display duty cycle.

      *dispbuf = 0;
    }
    break;

    case DRD_PERSONALITY_DRD_RET:
    {
      if ( (state -> Mode == MODE_CODE) && (state -> SigType != SIGTYPE_DRD_RET_AFTC) )
      {
        // DRD_RET does not display duty cycle.

        switch ( state -> DemodState ) // Cab LCD Display
        {
          case DRD_RET_CAB_STATE_MULTIPLE:
            sprintf(dispbuf, "     ");
          break;

          case DRD_RET_CAB_STATE_NOCARRIER:
            sprintf(dispbuf, "UNKWN");
          break;

          case DRD_RET_CAB_STATE_PRIMARY:
          case DRD_RET_CAB_STATE_UNKNOWN:
            sprintf(dispbuf, "UNKWN");
          break;

          case DRD_RET_CAB_STATE_0A:
            sprintf(dispbuf, "0A   ");
          break;

          case DRD_RET_CAB_STATE_10A:
            sprintf(dispbuf, "10A  ");
          break;

          case DRD_RET_CAB_STATE_20P:
            sprintf(dispbuf, "20P  ");
          break;

          case DRD_RET_CAB_STATE_60G:
            sprintf(dispbuf, "60G  ");
          break;

          case DRD_RET_CAB_STATE_35G:
            sprintf(dispbuf, "35G  ");
          break;

          case DRD_RET_CAB_STATE_50ROZ:
            sprintf(dispbuf, "50ROZ");
          break;

          case DRD_RET_CAB_STATE_50ST:
            sprintf(dispbuf, "50ST ");
          break;

          case DRD_RET_CAB_STATE_50G:
            sprintf(dispbuf, "50G   ");
          break;

          case DRD_RET_CAB_STATE_70G:
            sprintf(dispbuf, "70G   ");
          break;

          case DRD_RET_CAB_STATE_80G:
            sprintf(dispbuf, "80G   ");
          break;

          default:
            sprintf(dispbuf, "ST=%X    ", state -> DemodState);   // should never happen, would like to know if it does!
          break;
        }
      }
      else
      {
        dispbuf[0] = 0;
      }
    }
    break;
  } // end switch(gDRDPersonality)
} // end GetCorrectDutyForDisplay()

void GetFreqForDisplay( struct _drd_state_ *state, char *dispbuf )
{
  state -> PresDispFreq = state -> aDispFreqTab[state -> PresFreqIdx];
    
  switch ( state -> DRDPersonality )
  {
    case DRD_PERSONALITY_BART:
    {
      switch( state -> PresFreqIdx )
      {
        case 0:
        case 1:
        case 2:
        case 3:
          sprintf(dispbuf, "%4dHZ   ", state -> PresDispFreq);
        break;

        case 4:
          sprintf(dispbuf, "A0:%4dHZ", state -> PresDispFreq);
        break;

        case 5:
          sprintf(dispbuf, "A1:%4dHZ", state -> PresDispFreq);
        break;

        case 6:
          sprintf(dispbuf, "B0:%4dHZ", state -> PresDispFreq);
        break;

        case 7:
          sprintf(dispbuf, "B1:%4dHZ", state -> PresDispFreq);
        break;

        case 8:
          sprintf(dispbuf, "C0:%4dHZ", state -> PresDispFreq);
        break;

        case 9:
          sprintf(dispbuf, "C1:%4dHZ", state -> PresDispFreq);
        break;

        case 10:
          sprintf(dispbuf, "F0:%4dHZ", state -> PresDispFreq);
        break;

        case 11:
          sprintf(dispbuf, "F1:%4dHZ", state -> PresDispFreq);
        break;
      }
    }
    break;

    case DRD_PERSONALITY_DRD11:
    {
      sprintf(dispbuf, "%4dHZ", state -> PresDispFreq);
    }
    break;

    case DRD_PERSONALITY_DRD_ROW:
    {
      sprintf(dispbuf, "%4dHZ", state -> PresDispFreq);
    }
    break;

    case DRD_PERSONALITY_DRD_CHAR:
    case DRD_PERSONALITY_DRD_MFOR:
    {
      sprintf(dispbuf, "%4dHZ", state -> PresDispFreq);
    }
    break;

    case DRD_PERSONALITY_DRD_STL:
    {
      sprintf(dispbuf, "%4dHZ", state -> PresDispFreq);
    }
    break;

    case DRD_PERSONALITY_DRD_RET:
    {
      dispbuf[0] = 0;

      if ( (state -> Mode == MODE_MEAS) || (state -> SigType == SIGTYPE_DRD_RET_AFTC) )
      {
        state -> PresDispFreq = state -> aDispFreqTab[state -> PresFreqIdx];

        sprintf(dispbuf, "%3dHZ ", state -> PresDispFreq);
      }
    }
    break;
  } // end switch(gDRDPersonality)
} // end GetFreqForDisplay()

void BuildBaseDisplay( struct _drd_state_ *state, int Mode, int Line, char *dispbuf )
{
  char tbuf[15];

  switch ( state -> DRDPersonality )
  {
    case DRD_PERSONALITY_BART:
    {
      //   1234567890123456
      // --------------------
      // | 18 mph           |
      // | A0-7776HZ   CODE |
      // --------------------

      if ( Mode == MODE_CODE )   // code
      {
        if ( Line == 1 )
        {
          sprintf(dispbuf, "   mph          ");
        }
        else
        {
          GetFreqForDisplay(state, tbuf);
          sprintf(dispbuf, "%s   CODE", tbuf);
        }
      }

      //   1234567890123456
      // --------------------
      // | 10.00 Arms       |
      // | A0-7776HZ   AMPS |
      // --------------------
      else                    // amps
      {
        if ( Line == 1 )
        {
          if ( state -> SigType == SIGTYPE_BARTSC )
          {
            sprintf(dispbuf, "     %s       ", DRDPersTable.AmpsUnits1 );
          }
          else if ( state -> SigType == SIGTYPE_BARTMV )
          {
            sprintf(dispbuf, "     %s       ", DRDPersTable.AmpsUnits2);
          }
        }
        else
        {
          GetFreqForDisplay(state, tbuf);

          if ( state -> SigType == SIGTYPE_BARTSC )
          {
            sprintf(dispbuf, "%s   AMPS", tbuf);
          }
          else
          {
            sprintf(dispbuf, "%s  VOLTS", tbuf);
          }
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD11:
    {
      //   1234567890123456
      // --------------------
      // | 270    DUTY= 53% |
      // |  100HZ  CAB CODE |
      // --------------------
      if ( Mode == MODE_CODE )   // code
      {
        if ( Line == 1 )
        {
          sprintf(dispbuf, "        DUTY=   ");
        }
        else
        {
          GetFreqForDisplay(state, tbuf);
          
          if ( state -> PowerSaveEnable == 1 )
          {
            sprintf(dispbuf, "%s  CAB CODE", tbuf);
          }
          else
          {
            if ( state -> PowerSaveEnable == 2 )
            {
              sprintf(dispbuf, "%s  CAB#CODE", tbuf);
            }
            else
            {
              sprintf(dispbuf, "%s  CAB*CODE", tbuf);
            }
          }
        }
      }
      //   1234567890123456
      // --------------------
      // | 10.00 Arms       |
      // |  100HZ  CAB AMPS |
      // --------------------
      else                    // amps
      {
        if ( Line == 1 )
        {
          sprintf(dispbuf, "       %s      ", DRDPersTable.AmpsUnits1);
        }
        else
        {
          GetFreqForDisplay(state, tbuf);

          if ( state -> PowerSaveEnable == 1 )
          {
            sprintf(dispbuf, "%s  CAB AMPS", tbuf);
          }
          else
          {
            if ( state -> PowerSaveEnable == 2 )
            {
              sprintf(dispbuf, "%s  CAB#AMPS", tbuf);
            }
            else
            {
              sprintf(dispbuf, "%s  CAB*AMPS", tbuf);
            }
          }
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_ROW:
    {
      if ( Mode == MODE_CODE )
      {
        if ( state -> SigType == SIGTYPE_DRD_ROW_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s AFTC CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s AFTC#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s AFTC*CODE", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_ROW_CAB)
        {
          //   1234567890123456
          // --------------------
          // | NO CARRIER       |
          // | 2340HZ CAB CODE  |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  CAB CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  CAB#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  CAB*CODE", tbuf);
              }
            }
          }
        }
      }
      else // (Mode == MODE_MEAS)
      {
        if ( state -> SigType == SIGTYPE_DRD_ROW_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "       %s      ", DRDPersTable.AmpsUnits2);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s AFTC AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s AFTC#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s AFTC*AMPS", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_ROW_CAB)
        {
          //   1234567890123456
          // --------------------
          // | 10.00 Arms       |
          // | 2340HZ  CAB AMPS |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "       %s     ", DRDPersTable.AmpsUnits1);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);
	    
            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  CAB AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  CAB#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  CAB*AMPS", tbuf);
              }
            }
          }
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_MFOR:
    {
      if ( Mode == MODE_CODE )
      {
        if ( state -> SigType == SIGTYPE_DRD_MFOR_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s AFTC CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s AFTC#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s AFTC*CODE", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_MFOR_CAB)
        {
          //   1234567890123456
          // --------------------
          // | NO CARRIER       |
          // | 2340HZ CAB CODE  |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  CAB CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  CAB#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  CAB*CODE", tbuf);
              }
            }
          }
        }
      }
      else // (Mode == MODE_MEAS)
      {
        if ( state -> SigType == SIGTYPE_DRD_MFOR_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "        %s    ", DRDPersTable.AmpsUnits2);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s AFTC AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s AFTC#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s AFTC*AMPS", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_MFOR_CAB)
        {
          //   1234567890123456
          // --------------------
          // | 10.00 Arms       |
          // | 2340HZ  CAB AMPS |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "        %s   ", DRDPersTable.AmpsUnits1);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  CAB AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  CAB#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  CAB*AMPS", tbuf);
              }
            }
          }
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_STL:
    {
      if ( Mode == MODE_CODE )
      {
        if ( state -> SigType == SIGTYPE_DRD_STL_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s AFTC CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s AFTC#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s AFTC*CODE", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_STL_CAB)
        {
          //   1234567890123456
          // --------------------
          // | NO CARRIER       |
          // | 2340HZ CAB CODE  |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  CAB CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  CAB#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  CAB*CODE", tbuf);
              }
            }
          }
        }
      }
      else // (Mode == MODE_MEAS)
      {
        if ( state -> SigType == SIGTYPE_DRD_STL_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "        %s     ", DRDPersTable.AmpsUnits2);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s AFTC AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s AFTC#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s AFTC*AMPS", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_STL_CAB)
        {
          //   1234567890123456
          // --------------------
          // | 10.00 Arms       |
          // | 2340HZ  CAB AMPS |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "        %s     ", DRDPersTable.AmpsUnits1);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  CAB AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  CAB#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  CAB*AMPS", tbuf);
              }
            }
          }
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_RET:
    {
      if ( Mode == MODE_CODE )
      {
        if ( state -> SigType == SIGTYPE_DRD_RET_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%sAFTC CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%sAFTC#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%sAFTC*CODE", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_RET_CAB)
        {
          //   1234567890123456
          // --------------------
          // | NO CARRIER       |
          // | 2340HZ CAB CODE  |
          // --------------------

          if ( Line == 1 )
          {
            state -> PresDispFreq = state -> aDispFreqTab[state -> PresFreqIdx];
            state -> PresDispFreq1 = state -> aDispFreqTab[state -> PresFreq1Idx];

            if ( state -> PresDispFreq > state -> PresDispFreq1 )
            {
              sprintf( tbuf, "%3dHZ %3dHZ", state -> PresDispFreq1, state -> PresDispFreq);
            }
            else
            {
              sprintf( tbuf, "%3dHZ %3dHZ", state -> PresDispFreq, state -> PresDispFreq1);
            }

            sprintf(dispbuf, "---- %s", tbuf);
          }
          else
          {
            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "UNKWN  ATP  CODE");
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "UNKWN  ATP #CODE");
              }
              else
              {
                sprintf(dispbuf, "UNKWN  ATP *CODE");
              }
            }
          }
        }
      }
      else // (Mode == MODE_MEAS)
      {
        if ( state -> SigType == SIGTYPE_DRD_RET_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "        %s     ", DRDPersTable.AmpsUnits2);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%sAFTC AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%sAFTC#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%sAFTC*AMPS", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_RET_CAB)
        {
          //   1234567890123456
          // --------------------
          // | 10.00 Arms       |
          // | 2340HZ  CAB AMPS |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "        %s   ", DRDPersTable.AmpsUnits1);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  ATP AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  ATP#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  ATP*AMPS", tbuf);
              }
            }
          }
        }
      }
    }
    break;

    case DRD_PERSONALITY_DRD_CHAR:
    {
      if ( Mode == MODE_CODE )
      {
        if ( state -> SigType == SIGTYPE_DRD_CHAR_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s AFTC CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s AFTC#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s AFTC*CODE", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_CHAR_CAB)
        {
          //   1234567890123456
          // --------------------
          // | NO CARRIER       |
          // | 2340HZ CAB CODE  |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "                ");
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  CAB CODE", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  CAB#CODE", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  CAB*CODE", tbuf);
              }
            }
          }
        }
      }
      else // (Mode == MODE_MEAS)
      {
        if ( state -> SigType == SIGTYPE_DRD_CHAR_AFTC )
        {
          if ( Line == 1 )
          {
            sprintf(dispbuf, "        %s    ", DRDPersTable.AmpsUnits2);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s AFTC AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s AFTC#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s AFTC*AMPS", tbuf);
              }
            }
          }
        }
        else // (gSigType == SIGTYPE_DRD_CHAR_CAB)
        {
          //   1234567890123456
          // --------------------
          // | 10.00 Arms       |
          // | 2340HZ  CAB AMPS |
          // --------------------

          if ( Line == 1 )
          {
            sprintf(dispbuf, "        %s   ", DRDPersTable.AmpsUnits1);
          }
          else
          {
            GetFreqForDisplay(state, tbuf);

            if ( state -> PowerSaveEnable == 1 )
            {
              sprintf(dispbuf, "%s  CAB AMPS", tbuf);
            }
            else
            {
              if ( state -> PowerSaveEnable == 2 )
              {
                sprintf(dispbuf, "%s  CAB#AMPS", tbuf);
              }
              else
              {
                sprintf(dispbuf, "%s  CAB*AMPS", tbuf);
              }
            }
          }
        }
      }
    }
    break;
  } // end switch(gDRDPersonality)
} // end BuildBaseDisplay()

void GetMeasUnits( struct _drd_state_ *state, char *textbuf )
{
  switch( state -> DRDPersonality )
  {
    case DRD_PERSONALITY_BART:
    {
      if ( state -> SigType == SIGTYPE_BARTSC )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits1 );
      }
      else if ( state -> SigType == SIGTYPE_BARTMV )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits2);
      }
    }
    break;

    case DRD_PERSONALITY_DRD11:
    {
      sprintf(textbuf, "%s", DRDPersTable.AmpsUnits1);
    }
    break;

    case DRD_PERSONALITY_DRD_ROW:
    {
      if ( state -> SigType == SIGTYPE_DRD_ROW_AFTC )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits2);
      }
      else if ( state -> SigType == SIGTYPE_DRD_ROW_CAB )
      {
      }

      sprintf(textbuf, "%s", DRDPersTable.AmpsUnits1);
    }
    break;

    case DRD_PERSONALITY_DRD_MFOR:
    {
      if ( state -> SigType == SIGTYPE_DRD_MFOR_AFTC )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits2);
      }
      else if ( state -> SigType == SIGTYPE_DRD_MFOR_CAB )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits1);
      }
    }
    break;

    case DRD_PERSONALITY_DRD_STL:
    {
      if ( state -> SigType == SIGTYPE_DRD_STL_AFTC )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits2);
      }
      else if ( state -> SigType == SIGTYPE_DRD_STL_CAB )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits1);
      }
    }
    break;

    case DRD_PERSONALITY_DRD_RET:
    {
      if ( state -> SigType == SIGTYPE_DRD_RET_AFTC )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits2);
      }
      else if ( state -> SigType == SIGTYPE_DRD_RET_CAB )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits1);
      }
    }
    break;

    case DRD_PERSONALITY_DRD_CHAR:
    {
      if ( state -> SigType == SIGTYPE_DRD_CHAR_AFTC )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits2);
      }
      else if ( state -> SigType == SIGTYPE_DRD_CHAR_CAB )
      {
        sprintf(textbuf, "%s", DRDPersTable.AmpsUnits1);
      }
    }
    break;
  } // end switch(dDRDPersonality)
} // end GetMeasUnits()
