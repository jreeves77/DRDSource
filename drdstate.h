#ifndef _DRDSTATE_H_

#define _DRDSTATE_H_

#include "arm_math.h"

// #define _TEST_

#ifdef _TEST_
#define VERSION "T2.25"
#else
#define VERSION "V2.25"
#endif

#define SAMPLE_FREQUENCY (6000000/256.0)
#define BLOCK_SIZE 256
#define DFT_BLOCK_SIZE (64*25)

#define POWERSAVE_STAGE1_TIME 439
#define POWERSAVE_STAGE2_TIME 549
#define NOPOWERSAVE_TIME      3296

#define LCD_RING_BUFFER_SIZE 256
#define LCD_RS 0x100
#define LCD_RW 0x200


#define BUTTON_FD   1
#define BUTTON_FU   2
#define BUTTON_TYPE 4
#define BUTTON_MODE 8
#define BUTTON_BL   16

#define MODE_MEAS      0
#define MODE_CODE      1

#define SENSTYPE_NONE       0
#define SENSTYPE_RAILSENS   1
#define SENSTYPE_SHUNT      2
#define SENSTYPE_DIRECT     4

#define SENSDET_SHORT    0x0100
#define SENSDET_200OHM   0x0200
#define SENSDET_500OHM   0x0400

#define BART_STATE_CONSTANT  0x21    // 10_0001  0x21
#define BART_STATE_NOCARRIER 0x20    // 10_0000  0x20
#define BART_STATE_UNKNOWN_1 0x11    // 01_0001  0x11
#define BART_STATE_UNKNOWN_0 0x10    // 01_0000  0x10
#define BART_STATE_1         0x03    // 00_0011  0x03
#define BART_STATE_0         0x02    // 00_0010  0x02

// DRD11 CAB Codes Detection MIN/MAX thresholds, decoding states, etc.
#define CAB_STATE_CONSTANT  0x21    // 10_0001  0x21
#define CAB_STATE_NOCARRIER 0x20    // 10_0000  0x20
#define CAB_STATE_UNKNOWN_1 0x11    // 01_0001  0x11
#define CAB_STATE_UNKNOWN_0 0x10    // 01_0000  0x10
#define CAB_STATE_75_1      0x09    // 00_1001  0x09
#define CAB_STATE_75_0      0x08    // 00_1000  0x08
#define CAB_STATE_180_1     0x05    // 00_0101  0x05
#define CAB_STATE_180_0     0x04    // 00_0100  0x04
#define CAB_STATE_270_1     0x03    // 00_0011  0x03
#define CAB_STATE_270_0     0x02    // 00_0010  0x02

#define BART_MIN_DETECT_SC      1000000
#define BART_MIN_DETECT_MV      67600 // Decode in mV mode uses a higher value to obtain the desired 50mVpp thresh

#define DRD11_MIN_DETECT        33124

#define DRD_ROW_CAB_MIN_DETECT  11025  // 11025  // around .125 Scaled Arms w/ equiv 0.14-ohm & 4x HW gain on input.
#define DRD_ROW_AFTC_MIN_DETECT 10000  // 4225  // around .080 Scaled Arms w/ equiv 0.14-ohm shunt 

#define DRD_MFOR_CAB_MIN_DETECT 11025  // around .125 Scaled Arms w/ equiv 0.14-ohm & 4x HW gain on input.
#define DRD_MFOR_AFTC_MIN_DETECT 10000 // 4225  // around .080 Scaled Arms w/ equiv 0.14-ohm shunt 

#define DRD_STL_CAB_MIN_DETECT 1024  // around .125 Scaled Arms w/ equiv 0.14-ohm & 4x HW gain on input.
#define DRD_STL_AFTC_MIN_DETECT 931 // 4225  // around .080 Scaled Arms w/ equiv 0.14-ohm shunt 

#define DRD_RET_CAB_MIN_DETECT  11025  // around .125 Scaled Arms w/ equiv 0.14-ohm & 4x HW gain on input.
#define DRD_RET_AFTC_MIN_DETECT  10000  // 4225 around .080 Scaled Arms w/ equiv 0.14-ohm shunt 

// DRD-ROW / DRD-MFOR AFTC Decoding States
#define AFTC_STATE_CONSTANT  0x21    // 10_0001  0x21
#define AFTC_STATE_NOCARRIER 0x20    // 10_0000  0x20
#define AFTC_STATE_UNKNOWN_1 0x11    // 01_0001  0x11
#define AFTC_STATE_UNKNOWN_0 0x10    // 01_0000  0x10
#define AFTC_STATE_A_1       0x09    // 00_1001  0x09
#define AFTC_STATE_A_0       0x08    // 00_1000  0x08
#define AFTC_STATE_B_1       0x05    // 00_0101  0x05
#define AFTC_STATE_B_0       0x04    // 00_0100  0x04
#define AFTC_STATE_C_1       0x03    // 00_0011  0x03
#define AFTC_STATE_C_0       0x02    // 00_0010  0x02

// DRD_ROW CAB Codes Detection MIN/MAX thresholds, decoding states, etc.
#define DRD_ROW_CAB_STATE_CONSTANT  1
#define DRD_ROW_CAB_STATE_NOCARRIER 2
#define DRD_ROW_CAB_STATE_UNKNOWN   3
#define DRD_ROW_CAB_STATE_70        4
#define DRD_ROW_CAB_STATE_65        5
#define DRD_ROW_CAB_STATE_60        6
#define DRD_ROW_CAB_STATE_50        7
#define DRD_ROW_CAB_STATE_50LOS     8
#define DRD_ROW_CAB_STATE_40        9
#define DRD_ROW_CAB_STATE_30        10
#define DRD_ROW_CAB_STATE_20        11
#define DRD_ROW_CAB_STATE_15        12
#define DRD_ROW_CAB_STATE_10        13
#define DRD_ROW_CAB_STATE_10LOS     14
#define DRD_ROW_CAB_STATE_0         15

// DRD_MFOR CAB Codes Detection MIN/MAX thresholds, decoding states, etc.
#define DRD_MFOR_CAB_STATE_CONSTANT  1
#define DRD_MFOR_CAB_STATE_NOCARRIER 2
#define DRD_MFOR_CAB_STATE_UNKNOWN   3
#define DRD_MFOR_CAB_STATE_80        4
#define DRD_MFOR_CAB_STATE_70        5
#define DRD_MFOR_CAB_STATE_60        6
#define DRD_MFOR_CAB_STATE_50        7
#define DRD_MFOR_CAB_STATE_40        8
#define DRD_MFOR_CAB_STATE_25        9
#define DRD_MFOR_CAB_STATE_10        10
#define DRD_MFOR_CAB_STATE_0         11

// DRD_RET CAB Codes Detection MIN/MAX thresholds, decoding states, etc.
#define DRD_RET_CAB_STATE_MULTIPLE   1
#define DRD_RET_CAB_STATE_NOCARRIER  2
#define DRD_RET_CAB_STATE_UNKNOWN    3
#define DRD_RET_CAB_STATE_0A         4
#define DRD_RET_CAB_STATE_10A        5
#define DRD_RET_CAB_STATE_20P        6
#define DRD_RET_CAB_STATE_60G        7
#define DRD_RET_CAB_STATE_35G        8
#define DRD_RET_CAB_STATE_50ROZ      9
#define DRD_RET_CAB_STATE_50ST      10
#define DRD_RET_CAB_STATE_50G       11
#define DRD_RET_CAB_STATE_70G       12
#define DRD_RET_CAB_STATE_80G       13

#define DRD_STL_CAB_STATE_CONSTANT  1
#define DRD_STL_CAB_STATE_NOCARRIER 2
#define DRD_STL_CAB_STATE_UNKNOWN   3
#define DRD_STL_CAB_STATE_55        4
#define DRD_STL_CAB_STATE_45        5
#define DRD_STL_CAB_STATE_STREET    6
#define DRD_STL_CAB_STATE_35        7
#define DRD_STL_CAB_STATE_25        8
#define DRD_STL_CAB_STATE_YARD      9
#define DRD_STL_CAB_STATE_15       10
#define DRD_STL_CAB_STATE_5        11

#define MAX_CHARACTERS 80
#define MAX_NUM_FREQS   20
#define RING_BUFFER_SIZE 512

#define NUM_INT_AD_CHANNELS     5

#define DRD_PERSONALITY_NONE    -1

typedef enum
{
  SIGTYPE_NONE,
  SIGTYPE_BARTSC,
  SIGTYPE_DRD11_CAB,
  SIGTYPE_BARTMV,
  SIGTYPE_DRD_ROW_AFTC,
  SIGTYPE_DRD_ROW_CAB,
  SIGTYPE_DRD_MFOR_AFTC,
  SIGTYPE_DRD_MFOR_CAB,
  SIGTYPE_DRD_RET_CAB,
  SIGTYPE_DRD_RET_AFTC,
  SIGTYPE_DRD_STL_CAB,
  SIGTYPE_DRD_STL_AFTC,
  NUMBER_OF_SIGNAL_TYPES
} DRDSignalTypes;

typedef enum
{
  DRD_PERSONALITY_BART,
  DRD_PERSONALITY_DRD11,    // Denver
  DRD_PERSONALITY_DRD_ROW,  // Region Of Waterloo
  DRD_PERSONALITY_DRD_MFOR, // Metro
  DRD_PERSONALITY_DRD_RET,  // Rotterdam
  DRD_PERSONALITY_DRD_STL,
  PERS_TAB_SZ
} DRDPersonalities;

typedef enum
{
  MENU_TOP_LEVEL,
  MENU_MAIN_INPUT_LEVEL,
  MENU_TESTS_LEVEL,
  MENU_GAIN_CAL_LEVEL,
  MENU_AMPS_OFFSET_LEVEL,
  MENU_THRESHOLD_LEVEL,
  MENU_DEBUG_LEVEL,
  MENU_DEBUG_INPUT_LEVEL,
  MENU_DEBUG_LEVEL1,
  MENU_CALIB_LEVEL,
  NUMBER_OF_LEVELS
} MenuStates;

typedef enum
{
  TOP_CALIB_MENU_LEVEL,
  MAIN_CALIB_MENU_LEVEL,
  INPUT_CALIB_MENU_LEVEL,
  INPUT1_CALIB_MENU_LEVEL,
  INPUT2_CALIB_MENU_LEVEL,
  INPUT3_CALIB_MENU_LEVEL,
  INPUT4_CALIB_MENU_LEVEL,
  INPUT5_CALIB_MENU_LEVEL,
  INPUT6_CALIB_MENU_LEVEL,
  INPUT7_CALIB_MENU_LEVEL,
  INPUT8_CALIB_MENU_LEVEL,
  INPUT9_CALIB_MENU_LEVEL,
  INPUT10_CALIB_MENU_LEVEL,
  INPUT11_CALIB_MENU_LEVEL,
  INPUT12_CALIB_MENU_LEVEL,
  NUMBER_OF_CALIB_LEVELS
} CalibMenuLevels;

typedef enum
{
  TESTS_TOP_MENU_LEVEL,
  TESTS_MAIN_INPUT_MENU_LEVEL,
  TESTS_INPUT1_MENU_LEVEL,
  TESTS_INPUT2_MENU_LEVEL,
  TESTS_INPUT3_MENU_LEVEL,
  NUMBER_OF_TESTS_LEVELS
} TestsMenuLevels;

typedef enum
{
  ADC_5_5V,
  ADC_3_3V,
  ADC_SHUNTMONV,
  ADC_1_8V,
  ADC_BAT_MONV
} AdcChannels;

struct ring_buffer
{
  short size;
  short writeIndex;
  short readIndex;
  unsigned char *buffer;
};

struct lcd_ring_buffer
{
  short size;
  short writeIndex;
  short readIndex;
  unsigned short *buffer;
};

// CONFIG_HEADER_T
typedef struct _config {
    uint32_t  ValidCode;          // 0x12345678 if valid
    uint32_t  Personality;        // what type of DRD is this
    float32_t AmpsCalibration;    // number that will divide by 10000 so scale Amps readings for each frequency in table 1
    uint32_t  AmpsOffset;         // number subtracted from amps readings for each frequency in table 1
} CONFIG_HEADER_T;

// PERS_TAB_T
typedef struct _personality {
    int      PersID;                  // DRD_PERSONALITY_xxx
    char     Title[12];               // CIRCA , DRD-11 , DRD-RET, etc...
    char     LoBatMsg[12];            // Message to display for low battery
    char     AmpsUnits1[8];           // A Arms mApp etc... for signalling type 1
    char     AmpsUnits2[8];           // A Arms mApp etc... for signalling type 2
    char     AmpsUnits3[8];           // A Arms mApp etc... for signalling type 3
    int      CalFregHopMsecs1;        // msec delay when changing freq during freq cal for sig type 1
    int      CalFregHopMsecs2;        // msec delay when changing freq during freq cal for sig type 2
    int      CalFregHopMsecs3;        // msec delay when changing freq during freq cal for sig type 3
    uint32_t CalFreqHopStep1;         // step size for cal freq tuning in tenths of hz for sig type 1
    uint32_t CalFreqHopStep2;         // step size for cal freq tuning in tenths of hz for sig type 2
    uint32_t CalFreqHopStep3;         // step size for cal freq tuning in tenths of hz for sig type 3
    int      InitMode;                // 0=Current  1=CODE
    int      InitSigType;             // initial signalling type (1, 2 or 3)
    uint32_t InitFreqIdx;             // initial index into freq table
    int      SigType1;                // Signalling type 1 (SIGTYPE_NONE, SIGTYPE_BARTSC, SIGTYPE_DRD11_CAB, etc...)
    int      SigType2;                // Signalling type 2 (SIGTYPE_NONE, SIGTYPE_BARTSC, SIGTYPE_DRD11_CAB, etc...)
    int      SigType3;                // Signalling type 3 (SIGTYPE_NONE, SIGTYPE_BARTSC, SIGTYPE_DRD11_CAB, etc...)
    int      SensorType1;             // Sensor type for Sig mode 1 (bit encoded as type is bits 7:0 and detection as bits 10:8)
    int      SensorType2;             // Sensor type for Sig mode 2 (bit encoded as type is bits 7:0 and detection as bits 10:8)
    int      SensorType3;             // Sensor type for Sig mode 3 (bit encoded as type is bits 7:0 and detection as bits 10:8)
    int      HWGain1;                 // Gain for sig type 1 (0 or 1, this is not the gain but the analog switch setting)
    int      HWGain2;                 // Gain for sig type 2 (0 or 1, this is not the gain but the analog switch setting)
    int      HWGain3;                 // Gain for sig type 3 (0 or 1, this is not the gain but the analog switch setting)
    int      AntiAliasFilt1;          // Anti-Aliasing filter selection for sig type 1 (0 or 1)
    int      AntiAliasFilt2;          // Anti-Aliasing filter selection for sig type 2 (0 or 1)
    int      AntiAliasFilt3;          // Anti-Aliasing filter selection for sig type 3 (0 or 1)
    int      NumFreqs1;               // Number of frequencies for Sig Type 1
    int      NumFreqs2;               // Number of frequencies for Sig Type 2
    int      NumFreqs3;               // Number of frequencies for Sig Type 3
    uint32_t DispFreqTable1[MAX_NUM_FREQS];      // Display Freq for Sig Type 1
    uint32_t DispFreqTable2[MAX_NUM_FREQS];      // Display Freq for Sig Type 2
    uint32_t DispFreqTable3[MAX_NUM_FREQS];      // Display Freq for Sig Type 3
} PERS_TAB_T;

struct _drd11_times
{
  uint16_t Cab270MinPer;
  uint16_t Cab270MaxPer;
  uint16_t Cab180MinPer;
  uint16_t Cab180MaxPer;
  uint16_t Cab075MinPer;
  uint16_t Cab075MaxPer;
  uint16_t Cab270MinOn;
  uint16_t Cab270MaxOn;
  uint16_t Cab180MinOn;
  uint16_t Cab180MaxOn;
  uint16_t Cab075MinOn;
  uint16_t Cab075MaxOn;
};

struct _row_times
{
  uint16_t DRD_ROW_CAB_MaxOn;
  uint16_t DRD_ROW_CAB_MaxOff;
  uint16_t DRD_ROW_70_MinPer;
  uint16_t DRD_ROW_70_MaxPer;
  uint16_t DRD_ROW_65_MinPer;
  uint16_t DRD_ROW_65_MaxPer;
  uint16_t DRD_ROW_60_MinPer;
  uint16_t DRD_ROW_60_MaxPer;
  uint16_t DRD_ROW_50_MinPer;
  uint16_t DRD_ROW_50_MaxPer;
  uint16_t DRD_ROW_50LOS_MinPer;
  uint16_t DRD_ROW_50LOS_MaxPer;
  uint16_t DRD_ROW_40_MinPer;
  uint16_t DRD_ROW_40_MaxPer;
  uint16_t DRD_ROW_30_MinPer;
  uint16_t DRD_ROW_30_MaxPer;
  uint16_t DRD_ROW_20_MinPer;
  uint16_t DRD_ROW_20_MaxPer;
  uint16_t DRD_ROW_15_MinPer;
  uint16_t DRD_ROW_15_MaxPer;
  uint16_t DRD_ROW_10_MinPer;
  uint16_t DRD_ROW_10_MaxPer;
  uint16_t DRD_ROW_10LOS_MinPer;
  uint16_t DRD_ROW_10LOS_MaxPer;
  uint16_t DRD_ROW_0_MinPer;
  uint16_t DRD_ROW_0_MaxPer;
};

struct _mfor_times
{
  uint16_t DRD_MFOR_CAB_MaxOn;
  uint16_t DRD_MFOR_CAB_MaxOff;
  uint16_t DRD_MFOR_80_MinPer;
  uint16_t DRD_MFOR_80_MaxPer;
  uint16_t DRD_MFOR_70_MinPer;
  uint16_t DRD_MFOR_70_MaxPer;
  uint16_t DRD_MFOR_60_MinPer;
  uint16_t DRD_MFOR_60_MaxPer;
  uint16_t DRD_MFOR_50_MinPer;
  uint16_t DRD_MFOR_50_MaxPer;
  uint16_t DRD_MFOR_40_MinPer;
  uint16_t DRD_MFOR_40_MaxPer;
  uint16_t DRD_MFOR_25_MinPer;
  uint16_t DRD_MFOR_25_MaxPer;
  uint16_t DRD_MFOR_10_MinPer;
  uint16_t DRD_MFOR_10_MaxPer;
  uint16_t DRD_MFOR_0_MinPer;
  uint16_t DRD_MFOR_0_MaxPer;
};

struct _stl_times
{
  uint16_t DRD_STL_CAB_MaxOn;
  uint16_t DRD_STL_CAB_MaxOff;
  uint16_t DRD_STL_55_MinPer;
  uint16_t DRD_STL_55_MaxPer;
  uint16_t DRD_STL_45_MinPer;
  uint16_t DRD_STL_45_MaxPer;
  uint16_t DRD_STL_STREET_MinPer;
  uint16_t DRD_STL_STREET_MaxPer;
  uint16_t DRD_STL_35_MinPer;
  uint16_t DRD_STL_35_MaxPer;
  uint16_t DRD_STL_25_MinPer;
  uint16_t DRD_STL_25_MaxPer;
  uint16_t DRD_STL_YARD_MinPer;
  uint16_t DRD_STL_YARD_MaxPer;
  uint16_t DRD_STL_15_MinPer;
  uint16_t DRD_STL_15_MaxPer;
  uint16_t DRD_STL_5_MinPer;
  uint16_t DRD_STL_5_MaxPer;
};

struct _drd_state_
{
  int SecondsCounter;
  int TenthSecondsCounter;
  int DebugFlags;
  int ActiveDebugFlags;
  int DebugLength;
  int LCDPresent;
  int LCD_Curx;
  int LCD_Cury;
  uint16_t DRDPersonality;
  uint16_t SigType;
  uint16_t SensorType;
  uint16_t SensDetect;
  uint16_t Mode;
  uint16_t ButtonHistory[2];
  uint16_t ScanForPeaks;
  uint16_t LineMax;
  uint16_t LineSize;
  uint16_t LinePosition;
  uint8_t  LineString[MAX_CHARACTERS];
  uint16_t MenuState;
  uint16_t CalibMenuLevel;
  uint16_t TestsMenuLevel;
  uint16_t ButtonEvent;
  uint16_t PresFreqIdx;
  uint16_t PresFreq1Idx;
  uint16_t NumFreqs;
  uint16_t BLightDuty;
  uint16_t PowerSaveEnable;
  uint16_t DemodState;
  uint16_t LastDemodState;
  uint16_t PresFreqLock;
  float32_t aAmpsGainCalTab;
  uint16_t aAmpsOffsetCalTab;
  uint16_t aDispFreqTab[MAX_NUM_FREQS];
  float32_t PresAmpsGainCal;
  uint16_t PresAmpsOffsetCal;
  uint16_t PresDispFreq;
  uint16_t PresDispFreq1;
  uint16_t PeakSampRate;
  uint16_t SampCount;
  uint16_t DemodMaxOnCounts;
  uint16_t DemodOnCounts;
  uint16_t DemodMaxOffCounts;
  uint16_t DemodHalfBit;
  uint16_t DemodOneBit;
  uint16_t DemodShiftCount;
  uint16_t DemodShiftState;
  uint16_t DemodShiftReg;
  uint16_t DemodShiftIdx;
  uint16_t DemodNextBitTime;
  uint16_t DemodDuty;
  uint16_t Calibrating;
  union _cab_times
  {
    struct _drd11_times DRD11;
    struct _row_times   ROW;
    struct _mfor_times  MFOR;
    struct _stl_times   STL;
  } CabTimes;
  uint16_t AFTC_A_MinPer;
  uint16_t AFTC_A_MaxPer;
  uint16_t AFTC_B_MinPer;
  uint16_t AFTC_B_MaxPer;
  uint16_t AFTC_C_MinPer;
  uint16_t AFTC_C_MaxPer;
  uint16_t AFTC_A_MinOn;
  uint16_t AFTC_A_MaxOn;
  uint16_t AFTC_B_MinOn;
  uint16_t AFTC_B_MaxOn;
  uint16_t AFTC_C_MinOn;
  uint16_t AFTC_C_MaxOn;
  uint16_t StartPeakDetect;
  uint16_t StopPeakDetect;
  uint32_t Period;
  uint32_t OnCounts;
  uint16_t last_cab_on_time;
  uint16_t last_cab_off_time;
  char signal_state;
  char check_period;
  float32_t DemodThreshold;
  float32_t DemodThresholdHyst;
  float32_t DemodAvg;
  float32_t PeakDemodAvg;
  float32_t PeakAvg;
  float32_t PeakSquaredAvg;
  float32_t PeakSquaredAvg1[3];
  float32_t BattSampling;
  float32_t sinef32;
  float32_t cosinef32;
  float32_t currentSineF32;
  float32_t currentCosineF32;
};

void InitializeProcData( struct _drd_state_ *state, float32_t frequency );

void DRD_MFOR_CABDecoder( struct _drd_state_ *state, int16_t *data, uint16_t size );
void DRD_AFTCDecoder( struct _drd_state_ *state, int16_t *data, uint16_t size );
void DRD11_Decoder( struct _drd_state_ *state, int16_t *data, uint16_t size );
void DRD_ROW_CABDecoder( struct _drd_state_ *state, int16_t *data, uint16_t size );
void BART_Decoder( struct _drd_state_ *state, short *buff, uint16_t size );

void PersonalityInit( struct _drd_state_ *state );

int SendBytes( LPC_USART_T *uart, struct ring_buffer *rPointer, unsigned char *buffer, int count );

uint32_t CalcDutyCycle(uint32_t OnTime, uint32_t TotTime);

void CalculateNewSettings( struct _drd_state_ *state, int FilterInit );

float GetADCVoltage( uint16_t channelIndex );

void GetMeasUnits( struct _drd_state_ *state, char *textbuf );

void GetFreqForDisplay( struct _drd_state_ *state, char *dispbuf );

float32_t GetCorrectAmpsForDisplay( struct _drd_state_ *state, char *dispbuf, uint32_t ampsval );

int SaveParameters( void );

void LCD_Gotoxy( struct _drd_state_ *state, int x, int y);

int LCD_Puts( struct lcd_ring_buffer *rPointer, unsigned char *buffer, int count );

float GetADCVoltage( uint16_t channelIndex );

int LCD_IsBusy(void);

void DoBatteryTest( struct _drd_state_ *state );

void LCD_Clrscr( struct _drd_state_ *state );

int SensorDetect(void);

int ReadShuntType(void);

int CheckSensorMatch( struct _drd_state_ *state );

void GetCorrectDutyForDisplay( struct _drd_state_ *state, char *dispbuf );

void BuildBaseDisplay( struct _drd_state_ *state, int Mode, int Line, char *dispbuf );

void GetCorrectCodeForDisplay( struct _drd_state_ *state, char *dispbuf );

void PowerDown( void );

void Beeper(int onoff);

void Backlight_SetDuty(int Duty);

void MenuRoutine( struct _drd_state_ *state, unsigned char *point, int charPresent );

void Calib_DoCalibration( struct _drd_state_ *state, unsigned char *point, int charPresent );

void InitializeFilters( void );

void InitializeDFTFilters( float32_t frequency, float32_t frequency0, float32_t sample_frequency );

#endif
