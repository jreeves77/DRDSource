#define SAMPLES_PER_MILLISECOND (SAMPLE_FREQUENCY/1000)
#define MILLISECONDS_PER_BLOCK 1

#define TRANSFER_SIZE BLOCK_SIZE
#define FILTER_LENGTH 8
#define WINDOW_FREQUENCY (SAMPLE_FREQUENCY/((float)DFT_BLOCK_SIZE))

#define SAMPLE_GAIN      1.00000f

#include "arm_math.h"

#include "sensorhub.h"
#include "drdstate.h"

#define AFTC_FILTER_CONSTANT (1.0)
#define LOW_PASS_GAIN (.991f)
#define FILTER_COEFF_1 (2.0f*LOW_PASS_GAIN)
#define FILTER_COEFF_2 (LOW_PASS_GAIN*LOW_PASS_GAIN)

#define RET_CAB_CONSTANT .96

struct matched_filter
{
  bool readingAvailable;
  int number;
  int index;
  short samples[DFT_BLOCK_SIZE];
  float DFTSine;
  float DFTCosine;
  float windowDFTPlusSine;
  float windowDFTMinusSine;
  float windowDFTPlusCosine;
  float windowDFTMinusCosine;
  float currentDFTSine;
  float currentDFTCosine;
  float reverseDFTSine;
  float reverseDFTCosine;
  float currentWindowDFTPlusSine;
  float currentWindowDFTMinusSine;
  float reverseWindowDFTPlusSine;
  float reverseWindowDFTMinusSine;
  float currentWindowDFTPlusCosine;
  float currentWindowDFTMinusCosine;
  float reverseWindowDFTPlusCosine;
  float reverseWindowDFTMinusCosine;
  float currentDFTSineResult;
  float currentDFTCosineResult;
  float currentWindowDFTPlusSineResult;
  float currentWindowDFTMinusSineResult;
  float currentWindowDFTPlusCosineResult;
  float currentWindowDFTMinusCosineResult;
  float resetDFTSineResult;
  float resetDFTCosineResult;
  float resetWindowDFTPlusSineResult;
  float resetWindowDFTMinusSineResult;
  float resetWindowDFTPlusCosineResult;
  float resetWindowDFTMinusCosineResult;
} Voltage2MatchedFilter;

struct matched_filter Voltage1MatchedFilter;

struct moving_average
{
  int number;
  int index;
  bool readingAvailable;
  float samples[FILTER_LENGTH];
  float sum;
  float resetSum;
} Magnitude1Filter;

struct moving_average Magnitude2Filter;

static float arctanTable[101] =
{
  .0000000000f, .0099996667f, .0199973340f, .0299910049f, .0399786871f, /* .00 -> .04 */
  .0499583957f, .0599281551f, .0698860016f, .0798299857f, .0897581742f, /* .05 -> .09 */
  .0996686525f, .1095595268f, .1194289260f, .1292750040f, .1390959415f, /* .10 -> .14 */
  .1488899476f, .1586552622f, .1683901571f, .1780929382f, .1877619465f, /* .15 -> .19 */
  .1973955599f, .2069921942f, .2165503050f, .2260683880f, .2355449807f, /* .20 -> .24 */
  .2449786631f, .2543680586f, .2637118345f, .2730087031f, .2822574220f, /* .25 -> .29 */
  .2914567945f, .3006056700f, .3097029445f, .3187475604f, .3277385068f, /* .30 -> .34 */
  .3366748194f, .3455555806f, .3543799191f, .3631470099f, .3718560738f, /* .35 -> .39 */
  .3805063771f, .3890972311f, .3976279915f, .4060980583f, .4145068746f, /* .40 -> .44 */
  .4228539261f, .4311387407f, .4393608873f, .4475199752f, .4556156532f, /* .45 -> .49 */
  .4636476090f, .4716155679f, .4795192920f, .4873585795f, .4951332635f, /* .50 -> .54 */
  .5028432109f, .5104883219f, .5180685285f, .5255837936f, .5330341102f, /* .55 -> .59 */
  .5404195003f, .5477400137f, .5549957273f, .5621867439f, .5693131911f, /* .60 -> .64 */
  .5763752206f, .5833730070f, .5903067469f, .5971766581f, .6039829783f, /* .65 -> .69 */
  .6107259644f, .6174058918f, .6240230530f, .6305777572f, .6370703293f, /* .70 -> .74 */
  .6435011088f, .6498704494f, .6561787180f, .6624262938f, .6686135679f, /* .75 -> .79 */
  .6747409422f, .6808088289f, .6868176498f, .6927678354f, .6986598247f, /* .80 -> .84 */
  .7044940642f, .7102710075f, .7159911144f, .7216548509f, .7272626880f, /* .85 -> .89 */
  .7328151018f, .7383125725f, .7437555843f, .7491446246f, .7544801838f, /* .90 -> .94 */
  .7597627549f, .7649928327f, .7701709140f, .7752974968f, .7803730801f, /* .95 -> .99 */
  .7853981634f                                                          /* 1.00 */
};

#define _TARGET_

#ifdef _TARGET_
#include "filtercoefs.h"
#include "filter1coefs.h"
#include "filterRETcoeffs.h"
#else 
#define FilterCoeffLength 41
extern float32_t FilterCoeff[41];
#define Filter1CoeffLength 61
extern float32_t Filter1Coeff[61];
#endif

static uint16_t RETCodes[10] = { DRD_RET_CAB_STATE_80G, DRD_RET_CAB_STATE_35G, DRD_RET_CAB_STATE_50ROZ, DRD_RET_CAB_STATE_70G, DRD_RET_CAB_STATE_50G,
                                 DRD_RET_CAB_STATE_50ST, DRD_RET_CAB_STATE_60G, DRD_RET_CAB_STATE_0A, DRD_RET_CAB_STATE_10A, DRD_RET_CAB_STATE_20P};

static int m, scanFreqIdx, scanFreq1Idx, scanExtraCount;
unsigned short traceCount, traceArray[32];

static arm_fir_decimate_instance_f32 FirstSineInstance, FirstCosineInstance, SecondSineInstance, SecondCosineInstance, RETInstance;
static arm_fir_instance_f32 ThirdSineInstance, ThirdCosineInstance;
static float32_t FirstInstanceSineState[FilterCoeffLength + BLOCK_SIZE - 1], FirstInstanceCosineState[FilterCoeffLength + BLOCK_SIZE - 1], RETInstanceState[FilterRETCoeffsLength + BLOCK_SIZE - 1];;
static float32_t SecondInstanceSineState[FilterCoeffLength + BLOCK_SIZE/8 - 1], SecondInstanceCosineState[FilterCoeffLength + BLOCK_SIZE/8 - 1];
static float32_t ThirdInstanceSineState[Filter1CoeffLength + BLOCK_SIZE/64 - 1], ThirdInstanceCosineState[Filter1CoeffLength + BLOCK_SIZE/64 - 1];
static float32_t FirstInstanceSineOutputs[BLOCK_SIZE/8], SecondInstanceSineOutputs[BLOCK_SIZE/64], ThirdInstanceSineOutputs[BLOCK_SIZE/64];
static float32_t FirstInstanceCosineOutputs[BLOCK_SIZE/8], SecondInstanceCosineOutputs[BLOCK_SIZE/64], ThirdInstanceCosineOutputs[BLOCK_SIZE/64];

static float32_t SineSamples[BLOCK_SIZE];
static float32_t CosineSamples[BLOCK_SIZE];
static float32_t frequency;

float Voltage1RealOutputs[TRANSFER_SIZE]; // Storage for the real voltage results from the DFT
float Voltage1ImagOutputs[TRANSFER_SIZE]; // Storage for the imaginary voltage results from the DFT
float Voltage2RealOutputs[TRANSFER_SIZE]; // Storage for the real current results from the DFT
float Voltage2ImagOutputs[TRANSFER_SIZE]; // Storage for the imaginary current results from the DFT

extern PERS_TAB_T DRDPersTable;

static void InitializeDFT( struct matched_filter *filter, float32_t frequency, float32_t sample_frequency )
{
  filter -> readingAvailable = false;

  filter -> number = 0;

  filter -> index = 0;

  filter -> DFTSine = sinf((float) (2.0f*((float) PI)*frequency/sample_frequency));

  filter -> DFTCosine = cosf((float) (2.0f*((float) PI)*frequency/sample_frequency));

  filter -> reverseDFTSine = (float) sinf(2.0f*((float) PI)*(frequency*DFT_BLOCK_SIZE)/sample_frequency);

  filter -> reverseDFTCosine = (float) cosf(2.0f*((float) PI)*(frequency*DFT_BLOCK_SIZE)/sample_frequency);

  filter -> windowDFTPlusSine = sinf((float) (2.0f*((float) PI)*(frequency + WINDOW_FREQUENCY)/sample_frequency));

  filter -> windowDFTMinusSine = sinf((float) (2.0f*((float) PI)*(frequency - WINDOW_FREQUENCY)/sample_frequency));

  filter -> reverseWindowDFTPlusSine = (float) sinf(2.0f*((float) PI)*((frequency + WINDOW_FREQUENCY)*DFT_BLOCK_SIZE)/sample_frequency);

  filter -> reverseWindowDFTMinusSine = (float) sinf(2.0f*((float) PI)*((frequency - WINDOW_FREQUENCY)*DFT_BLOCK_SIZE)/sample_frequency);

  filter -> windowDFTPlusCosine = cosf((float) (2.0f*((float) PI)*(frequency + WINDOW_FREQUENCY)/sample_frequency));

  filter -> windowDFTMinusCosine = cosf((float) (2.0f*((float) PI)*(frequency - WINDOW_FREQUENCY)/sample_frequency));

  filter -> reverseWindowDFTPlusCosine = (float) cosf(2.0f*((float) PI)*((frequency + WINDOW_FREQUENCY)*DFT_BLOCK_SIZE)/sample_frequency);

  filter -> reverseWindowDFTMinusCosine = (float) cosf(2.0f*((float) PI)*((frequency - WINDOW_FREQUENCY)*DFT_BLOCK_SIZE)/sample_frequency);

  filter -> currentDFTSine = 0.0f;

  filter -> currentDFTCosine = 1.0f;

  filter -> currentWindowDFTPlusSine = 0.0f;

  filter -> currentWindowDFTMinusSine = 0.0f;

  filter -> currentWindowDFTPlusCosine = 1.0f;

  filter -> currentWindowDFTMinusCosine = 1.0f;

  filter -> currentDFTSineResult = 0.0f;

  filter -> currentDFTCosineResult = 0.0f;

  filter -> currentWindowDFTPlusSineResult = 0.0f;

  filter -> currentWindowDFTMinusSineResult = 0.0f;

  filter -> currentWindowDFTPlusCosineResult = 0.0f;

  filter -> currentWindowDFTMinusCosineResult = 0.0f;

  filter -> resetDFTSineResult = 0.0f;

  filter -> resetDFTCosineResult = 0.0f;

  filter -> resetWindowDFTPlusSineResult = 0.0f;

  filter -> resetWindowDFTMinusSineResult = 0.0f;

  filter -> resetWindowDFTPlusCosineResult = 0.0f;

  filter -> resetWindowDFTMinusCosineResult = 0.0f;
}

static bool SingleLineDFT( struct matched_filter *filter, short *inputs, float *outputsReal, float *outputsImag, int countIn, int *countOut )
{
  int i;
  float tempSine, tempCosine, temp;

  countOut[0] = 0;

  for( i = 0;i < countIn;i++ )
  {
    if ( filter -> number < DFT_BLOCK_SIZE )
    {
      temp = (float) inputs[i];

      filter -> currentDFTSineResult += temp*filter -> currentDFTSine;

      filter -> currentDFTCosineResult += temp*filter -> currentDFTCosine;

      filter -> currentWindowDFTPlusSineResult += temp*filter -> currentWindowDFTPlusSine;

      filter -> currentWindowDFTMinusSineResult += temp*filter -> currentWindowDFTMinusSine;

      filter -> currentWindowDFTPlusCosineResult += temp*filter -> currentWindowDFTPlusCosine;

      filter -> currentWindowDFTMinusCosineResult += temp*filter -> currentWindowDFTMinusCosine;


      tempSine = (filter -> currentDFTSine*filter -> DFTCosine + filter -> currentDFTCosine*filter -> DFTSine);
      tempCosine = (filter -> currentDFTCosine*filter -> DFTCosine - filter -> currentDFTSine*filter -> DFTSine);

      filter -> currentDFTSine = tempSine;
      filter -> currentDFTCosine = tempCosine;

      tempSine = filter -> reverseDFTSine*filter -> DFTCosine - filter -> reverseDFTCosine*filter -> DFTSine;
      tempCosine = filter -> reverseDFTCosine*filter -> DFTCosine + filter -> reverseDFTSine*filter -> DFTSine;

      filter -> reverseDFTSine = tempSine;
      filter -> reverseDFTCosine = tempCosine;

      tempSine = (filter -> currentWindowDFTPlusSine*filter -> windowDFTPlusCosine + filter -> currentWindowDFTPlusCosine*filter -> windowDFTPlusSine);
      tempCosine = (filter -> currentWindowDFTPlusCosine*filter -> windowDFTPlusCosine - filter -> currentWindowDFTPlusSine*filter -> windowDFTPlusSine);

      filter -> currentWindowDFTPlusSine = tempSine;
      filter -> currentWindowDFTPlusCosine = tempCosine;

      tempSine = filter -> reverseWindowDFTPlusSine*filter -> windowDFTPlusCosine - filter -> reverseWindowDFTPlusCosine*filter -> windowDFTPlusSine;
      tempCosine = filter -> reverseWindowDFTPlusCosine*filter -> windowDFTPlusCosine + filter -> reverseWindowDFTPlusSine*filter -> windowDFTPlusSine;

      filter -> reverseWindowDFTPlusSine = tempSine;
      filter -> reverseWindowDFTPlusCosine = tempCosine;

      tempSine = (filter -> currentWindowDFTMinusSine*filter -> windowDFTMinusCosine + filter -> currentWindowDFTMinusCosine*filter -> windowDFTMinusSine);
      tempCosine = (filter -> currentWindowDFTMinusCosine*filter -> windowDFTMinusCosine - filter -> currentWindowDFTMinusSine*filter -> windowDFTMinusSine);

      filter -> currentWindowDFTMinusSine = tempSine;
      filter -> currentWindowDFTMinusCosine = tempCosine;

      tempSine = filter -> reverseWindowDFTMinusSine*filter -> windowDFTMinusCosine - filter -> reverseWindowDFTMinusCosine*filter -> windowDFTMinusSine;
      tempCosine = filter -> reverseWindowDFTMinusCosine*filter -> windowDFTMinusCosine + filter -> reverseWindowDFTMinusSine*filter -> windowDFTMinusSine;

      filter -> reverseWindowDFTMinusSine = tempSine;
      filter -> reverseWindowDFTMinusCosine = tempCosine;
      filter -> samples[filter -> number] = inputs[i];

      filter -> number += 1;

      if ( filter -> number >= DFT_BLOCK_SIZE )
      {
        filter -> readingAvailable = true;

        outputsReal[countOut[0]] = (float) ((2.0*filter -> currentDFTCosineResult - filter -> currentWindowDFTPlusCosineResult - filter -> currentWindowDFTMinusCosineResult)/DFT_BLOCK_SIZE);

        outputsImag[countOut[0]] = (float) ((2.0*filter -> currentDFTSineResult - filter -> currentWindowDFTPlusSineResult - filter -> currentWindowDFTMinusSineResult)/DFT_BLOCK_SIZE);

        countOut[0] += 1;
      }
    }
    else
    {
      float sampleTemp      = filter -> samples[filter -> index];
      float inputSample     = SAMPLE_GAIN*inputs[i];

      filter -> currentDFTCosineResult += inputSample*filter -> currentDFTCosine - sampleTemp*filter -> reverseDFTCosine;
      filter -> currentDFTSineResult += inputSample*filter -> currentDFTSine - sampleTemp*filter -> reverseDFTSine;

      filter -> currentWindowDFTPlusCosineResult += inputSample*filter -> currentWindowDFTPlusCosine - sampleTemp*filter -> reverseWindowDFTPlusCosine;
      filter -> currentWindowDFTPlusSineResult += inputSample*filter -> currentWindowDFTPlusSine - sampleTemp*filter -> reverseWindowDFTPlusSine;

      filter -> currentWindowDFTMinusCosineResult += inputSample*filter -> currentWindowDFTMinusCosine - sampleTemp*filter -> reverseWindowDFTMinusCosine;
      filter -> currentWindowDFTMinusSineResult += inputSample*filter -> currentWindowDFTMinusSine - sampleTemp*filter -> reverseWindowDFTMinusSine;

      filter -> samples[filter -> index] = inputs[i];

      filter -> index += 1;

      if ( filter -> index >= DFT_BLOCK_SIZE )
      {
        filter -> index = 0;

        filter -> currentDFTSineResult = filter -> resetDFTSineResult;
        filter -> currentDFTCosineResult = filter -> resetDFTCosineResult;

        filter -> currentWindowDFTPlusSineResult = filter -> resetWindowDFTPlusSineResult;
        filter -> currentWindowDFTPlusCosineResult = filter -> resetWindowDFTPlusCosineResult;

        filter -> currentWindowDFTMinusSineResult = filter -> resetWindowDFTMinusSineResult;
        filter -> currentWindowDFTMinusCosineResult = filter -> resetWindowDFTMinusCosineResult;

        filter -> resetDFTSineResult = 0.0;
        filter -> resetDFTCosineResult = 0.0;

        filter -> resetWindowDFTPlusSineResult = 0.0;
        filter -> resetWindowDFTPlusCosineResult = 0.0;

        filter -> resetWindowDFTMinusSineResult = 0.0;
        filter -> resetWindowDFTMinusCosineResult = 0.0;
      }
      else
      {
        filter -> resetDFTCosineResult += inputSample*filter -> currentDFTCosine;
        filter -> resetDFTSineResult += inputSample*filter -> currentDFTSine;

        filter -> resetWindowDFTPlusCosineResult += inputSample*filter -> currentWindowDFTPlusCosine;
        filter -> resetWindowDFTPlusSineResult += inputSample*filter -> currentWindowDFTPlusSine;

        filter -> resetWindowDFTMinusCosineResult += inputSample*filter -> currentWindowDFTMinusCosine;
        filter -> resetWindowDFTMinusSineResult += inputSample*filter -> currentWindowDFTMinusSine;

        tempSine = (filter -> resetDFTSineResult*filter -> DFTCosine - filter -> resetDFTCosineResult*filter -> DFTSine);
        tempCosine = (filter -> resetDFTCosineResult*filter -> DFTCosine + filter -> resetDFTSineResult*filter -> DFTSine);

        filter -> resetDFTSineResult = tempSine;
        filter -> resetDFTCosineResult = tempCosine;

        tempSine = (filter -> resetWindowDFTPlusSineResult*filter -> windowDFTPlusCosine - filter -> resetWindowDFTPlusCosineResult*filter -> windowDFTPlusSine);
        tempCosine = (filter -> resetWindowDFTPlusCosineResult*filter -> windowDFTPlusCosine + filter -> resetWindowDFTPlusSineResult*filter -> windowDFTPlusSine);

        filter -> resetWindowDFTPlusSineResult = tempSine;
        filter -> resetWindowDFTPlusCosineResult = tempCosine;

        tempSine = (filter -> resetWindowDFTMinusSineResult*filter -> windowDFTMinusCosine - filter -> resetWindowDFTMinusCosineResult*filter -> windowDFTMinusSine);
        tempCosine = (filter -> resetWindowDFTMinusCosineResult*filter -> windowDFTMinusCosine + filter -> resetWindowDFTMinusSineResult*filter -> windowDFTMinusSine);

        filter -> resetWindowDFTMinusSineResult = tempSine;
        filter -> resetWindowDFTMinusCosineResult = tempCosine;
      }

      tempSine = (filter -> currentDFTSineResult*filter -> DFTCosine - filter -> currentDFTCosineResult*filter -> DFTSine);
      tempCosine = (filter -> currentDFTCosineResult*filter -> DFTCosine + filter -> currentDFTSineResult*filter -> DFTSine);

      filter -> currentDFTSineResult = tempSine;
      filter -> currentDFTCosineResult = tempCosine;

      tempSine = (filter -> currentWindowDFTPlusSineResult*filter -> windowDFTPlusCosine - filter -> currentWindowDFTPlusCosineResult*filter -> windowDFTPlusSine);
      tempCosine = (filter -> currentWindowDFTPlusCosineResult*filter -> windowDFTPlusCosine + filter -> currentWindowDFTPlusSineResult*filter -> windowDFTPlusSine);

      filter -> currentWindowDFTPlusSineResult = tempSine;
      filter -> currentWindowDFTPlusCosineResult = tempCosine;

      tempSine = (filter -> currentWindowDFTMinusSineResult*filter -> windowDFTMinusCosine - filter -> currentWindowDFTMinusCosineResult*filter -> windowDFTMinusSine);
      tempCosine = (filter -> currentWindowDFTMinusCosineResult*filter -> windowDFTMinusCosine + filter -> currentWindowDFTMinusSineResult*filter -> windowDFTMinusSine);

      filter -> currentWindowDFTMinusSineResult = tempSine;
      filter -> currentWindowDFTMinusCosineResult = tempCosine;

      outputsReal[countOut[0]] = (float) ((2.0*filter -> currentDFTCosineResult - filter -> currentWindowDFTPlusCosineResult - filter -> currentWindowDFTMinusCosineResult)/DFT_BLOCK_SIZE);

      outputsImag[countOut[0]] = (float) ((2.0*filter -> currentDFTSineResult - filter -> currentWindowDFTPlusSineResult - filter -> currentWindowDFTMinusSineResult)/DFT_BLOCK_SIZE);

      countOut[0] += 1;
    }
  }

  return( filter -> readingAvailable );
}

static bool SingleLineDFTFloat( struct matched_filter *filter, float32_t *inputs, float *outputsReal, float *outputsImag, int countIn, int *countOut )
{
  int i;
  float tempSine, tempCosine, temp;

  countOut[0] = 0;

  for( i = 0;i < countIn;i++ )
  {
    if ( filter -> number < DFT_BLOCK_SIZE )
    {
      temp = (float) inputs[i];

      filter -> currentDFTSineResult += temp*filter -> currentDFTSine;

      filter -> currentDFTCosineResult += temp*filter -> currentDFTCosine;

      filter -> currentWindowDFTPlusSineResult += temp*filter -> currentWindowDFTPlusSine;

      filter -> currentWindowDFTMinusSineResult += temp*filter -> currentWindowDFTMinusSine;

      filter -> currentWindowDFTPlusCosineResult += temp*filter -> currentWindowDFTPlusCosine;

      filter -> currentWindowDFTMinusCosineResult += temp*filter -> currentWindowDFTMinusCosine;


      tempSine = (filter -> currentDFTSine*filter -> DFTCosine + filter -> currentDFTCosine*filter -> DFTSine);
      tempCosine = (filter -> currentDFTCosine*filter -> DFTCosine - filter -> currentDFTSine*filter -> DFTSine);

      filter -> currentDFTSine = tempSine;
      filter -> currentDFTCosine = tempCosine;

      tempSine = filter -> reverseDFTSine*filter -> DFTCosine - filter -> reverseDFTCosine*filter -> DFTSine;
      tempCosine = filter -> reverseDFTCosine*filter -> DFTCosine + filter -> reverseDFTSine*filter -> DFTSine;

      filter -> reverseDFTSine = tempSine;
      filter -> reverseDFTCosine = tempCosine;

      tempSine = (filter -> currentWindowDFTPlusSine*filter -> windowDFTPlusCosine + filter -> currentWindowDFTPlusCosine*filter -> windowDFTPlusSine);
      tempCosine = (filter -> currentWindowDFTPlusCosine*filter -> windowDFTPlusCosine - filter -> currentWindowDFTPlusSine*filter -> windowDFTPlusSine);

      filter -> currentWindowDFTPlusSine = tempSine;
      filter -> currentWindowDFTPlusCosine = tempCosine;

      tempSine = filter -> reverseWindowDFTPlusSine*filter -> windowDFTPlusCosine - filter -> reverseWindowDFTPlusCosine*filter -> windowDFTPlusSine;
      tempCosine = filter -> reverseWindowDFTPlusCosine*filter -> windowDFTPlusCosine + filter -> reverseWindowDFTPlusSine*filter -> windowDFTPlusSine;

      filter -> reverseWindowDFTPlusSine = tempSine;
      filter -> reverseWindowDFTPlusCosine = tempCosine;

      tempSine = (filter -> currentWindowDFTMinusSine*filter -> windowDFTMinusCosine + filter -> currentWindowDFTMinusCosine*filter -> windowDFTMinusSine);
      tempCosine = (filter -> currentWindowDFTMinusCosine*filter -> windowDFTMinusCosine - filter -> currentWindowDFTMinusSine*filter -> windowDFTMinusSine);

      filter -> currentWindowDFTMinusSine = tempSine;
      filter -> currentWindowDFTMinusCosine = tempCosine;

      tempSine = filter -> reverseWindowDFTMinusSine*filter -> windowDFTMinusCosine - filter -> reverseWindowDFTMinusCosine*filter -> windowDFTMinusSine;
      tempCosine = filter -> reverseWindowDFTMinusCosine*filter -> windowDFTMinusCosine + filter -> reverseWindowDFTMinusSine*filter -> windowDFTMinusSine;

      filter -> reverseWindowDFTMinusSine = tempSine;
      filter -> reverseWindowDFTMinusCosine = tempCosine;
      filter -> samples[filter -> number] = inputs[i];

      filter -> number += 1;

      if ( filter -> number >= DFT_BLOCK_SIZE )
      {
        filter -> readingAvailable = true;

        outputsReal[countOut[0]] = (float) ((2.0*filter -> currentDFTCosineResult - filter -> currentWindowDFTPlusCosineResult - filter -> currentWindowDFTMinusCosineResult)/DFT_BLOCK_SIZE);

        outputsImag[countOut[0]] = (float) ((2.0*filter -> currentDFTSineResult - filter -> currentWindowDFTPlusSineResult - filter -> currentWindowDFTMinusSineResult)/DFT_BLOCK_SIZE);

        countOut[0] += 1;
      }
    }
    else
    {
      float sampleTemp      = filter -> samples[filter -> index];
      float inputSample     = SAMPLE_GAIN*inputs[i];

      filter -> currentDFTCosineResult += inputSample*filter -> currentDFTCosine - sampleTemp*filter -> reverseDFTCosine;
      filter -> currentDFTSineResult += inputSample*filter -> currentDFTSine - sampleTemp*filter -> reverseDFTSine;

      filter -> currentWindowDFTPlusCosineResult += inputSample*filter -> currentWindowDFTPlusCosine - sampleTemp*filter -> reverseWindowDFTPlusCosine;
      filter -> currentWindowDFTPlusSineResult += inputSample*filter -> currentWindowDFTPlusSine - sampleTemp*filter -> reverseWindowDFTPlusSine;

      filter -> currentWindowDFTMinusCosineResult += inputSample*filter -> currentWindowDFTMinusCosine - sampleTemp*filter -> reverseWindowDFTMinusCosine;
      filter -> currentWindowDFTMinusSineResult += inputSample*filter -> currentWindowDFTMinusSine - sampleTemp*filter -> reverseWindowDFTMinusSine;

      filter -> samples[filter -> index] = inputs[i];

      filter -> index += 1;

      if ( filter -> index >= DFT_BLOCK_SIZE )
      {
        filter -> index = 0;

        filter -> currentDFTSineResult = filter -> resetDFTSineResult;
        filter -> currentDFTCosineResult = filter -> resetDFTCosineResult;

        filter -> currentWindowDFTPlusSineResult = filter -> resetWindowDFTPlusSineResult;
        filter -> currentWindowDFTPlusCosineResult = filter -> resetWindowDFTPlusCosineResult;

        filter -> currentWindowDFTMinusSineResult = filter -> resetWindowDFTMinusSineResult;
        filter -> currentWindowDFTMinusCosineResult = filter -> resetWindowDFTMinusCosineResult;

        filter -> resetDFTSineResult = 0.0;
        filter -> resetDFTCosineResult = 0.0;

        filter -> resetWindowDFTPlusSineResult = 0.0;
        filter -> resetWindowDFTPlusCosineResult = 0.0;

        filter -> resetWindowDFTMinusSineResult = 0.0;
        filter -> resetWindowDFTMinusCosineResult = 0.0;
      }
      else
      {
        filter -> resetDFTCosineResult += inputSample*filter -> currentDFTCosine;
        filter -> resetDFTSineResult += inputSample*filter -> currentDFTSine;

        filter -> resetWindowDFTPlusCosineResult += inputSample*filter -> currentWindowDFTPlusCosine;
        filter -> resetWindowDFTPlusSineResult += inputSample*filter -> currentWindowDFTPlusSine;

        filter -> resetWindowDFTMinusCosineResult += inputSample*filter -> currentWindowDFTMinusCosine;
        filter -> resetWindowDFTMinusSineResult += inputSample*filter -> currentWindowDFTMinusSine;

        tempSine = (filter -> resetDFTSineResult*filter -> DFTCosine - filter -> resetDFTCosineResult*filter -> DFTSine);
        tempCosine = (filter -> resetDFTCosineResult*filter -> DFTCosine + filter -> resetDFTSineResult*filter -> DFTSine);

        filter -> resetDFTSineResult = tempSine;
        filter -> resetDFTCosineResult = tempCosine;

        tempSine = (filter -> resetWindowDFTPlusSineResult*filter -> windowDFTPlusCosine - filter -> resetWindowDFTPlusCosineResult*filter -> windowDFTPlusSine);
        tempCosine = (filter -> resetWindowDFTPlusCosineResult*filter -> windowDFTPlusCosine + filter -> resetWindowDFTPlusSineResult*filter -> windowDFTPlusSine);

        filter -> resetWindowDFTPlusSineResult = tempSine;
        filter -> resetWindowDFTPlusCosineResult = tempCosine;

        tempSine = (filter -> resetWindowDFTMinusSineResult*filter -> windowDFTMinusCosine - filter -> resetWindowDFTMinusCosineResult*filter -> windowDFTMinusSine);
        tempCosine = (filter -> resetWindowDFTMinusCosineResult*filter -> windowDFTMinusCosine + filter -> resetWindowDFTMinusSineResult*filter -> windowDFTMinusSine);

        filter -> resetWindowDFTMinusSineResult = tempSine;
        filter -> resetWindowDFTMinusCosineResult = tempCosine;
      }

      tempSine = (filter -> currentDFTSineResult*filter -> DFTCosine - filter -> currentDFTCosineResult*filter -> DFTSine);
      tempCosine = (filter -> currentDFTCosineResult*filter -> DFTCosine + filter -> currentDFTSineResult*filter -> DFTSine);

      filter -> currentDFTSineResult = tempSine;
      filter -> currentDFTCosineResult = tempCosine;

      tempSine = (filter -> currentWindowDFTPlusSineResult*filter -> windowDFTPlusCosine - filter -> currentWindowDFTPlusCosineResult*filter -> windowDFTPlusSine);
      tempCosine = (filter -> currentWindowDFTPlusCosineResult*filter -> windowDFTPlusCosine + filter -> currentWindowDFTPlusSineResult*filter -> windowDFTPlusSine);

      filter -> currentWindowDFTPlusSineResult = tempSine;
      filter -> currentWindowDFTPlusCosineResult = tempCosine;

      tempSine = (filter -> currentWindowDFTMinusSineResult*filter -> windowDFTMinusCosine - filter -> currentWindowDFTMinusCosineResult*filter -> windowDFTMinusSine);
      tempCosine = (filter -> currentWindowDFTMinusCosineResult*filter -> windowDFTMinusCosine + filter -> currentWindowDFTMinusSineResult*filter -> windowDFTMinusSine);

      filter -> currentWindowDFTMinusSineResult = tempSine;
      filter -> currentWindowDFTMinusCosineResult = tempCosine;

      outputsReal[countOut[0]] = (float) ((2.0*filter -> currentDFTCosineResult - filter -> currentWindowDFTPlusCosineResult - filter -> currentWindowDFTMinusCosineResult)/DFT_BLOCK_SIZE);

      outputsImag[countOut[0]] = (float) ((2.0*filter -> currentDFTSineResult - filter -> currentWindowDFTPlusSineResult - filter -> currentWindowDFTMinusSineResult)/DFT_BLOCK_SIZE);

      countOut[0] += 1;
    }
  }

  return( filter -> readingAvailable );
}

static float PhaseDegrees( float real, float imag )
{
  float tangent, pTangent, pTangentLookup, fracTangent, RetValue, interpValue;
  int tanIndex;

  tangent = imag/real;

  pTangent = fabs(tangent);

  pTangentLookup = pTangent;

  if ( pTangent > 1.0f )
  {
    pTangentLookup = 1/pTangentLookup;
  }

  fracTangent = 100.0f*pTangentLookup;

  tanIndex = (int) fracTangent;

  fracTangent -= tanIndex;

  RetValue = arctanTable[tanIndex];

  if ( fracTangent != 0.0f )
  {
    interpValue = arctanTable[tanIndex + 1] - RetValue;

    RetValue += interpValue*fracTangent;
  }

  if ( pTangent > 1.0f )
  {
    RetValue = (float) (PI/2.0) - RetValue;
  }

  if ( real < 0.0f )
  {
    if ( imag < 0.0f )
    {
      RetValue += (float) PI;
    }
    else
    {
      RetValue = (float) (PI) - RetValue;
    }
  }
  else
  {
    if ( imag < 0.0f )
    {
      RetValue = (float) (2.0*PI - RetValue);
    }
  }

  RetValue *= (float) (180.0/PI);

  return( RetValue );
}

static void InitMovingAverage( struct moving_average *filter )
{
  filter -> number = 0;
  filter -> index = 0;

  filter -> sum = 0.0;
  filter -> resetSum = 0.0;

  filter -> readingAvailable = false;
}

static bool MovingAverage( struct moving_average *filter, float *inputs, float *outputs, int inputCount, int *outputCount )
{
  int i;

  outputCount[0] = 0;

  for( i = 0;i < inputCount;i++ )
  {
    if ( filter -> number < FILTER_LENGTH )
    {
      filter -> sum += inputs[i];

      filter -> samples[filter -> number] = inputs[i];

      filter -> number += 1;

      if ( filter -> number >= FILTER_LENGTH )
      {
        outputs[outputCount[0]] = filter -> sum/FILTER_LENGTH;

        outputCount[0] += 1;

        filter -> readingAvailable = true;
      }
    }
    else
    {
      filter -> sum += (inputs[i] - filter -> samples[filter -> index]);

      filter -> samples[filter -> index] = inputs[i];

      filter -> index += 1;

      if ( filter -> index >= FILTER_LENGTH )
      {
        filter -> index = 0;

	filter -> sum = filter -> resetSum;

	filter -> resetSum = 0.0;
      }
      else
      {
        filter -> resetSum += inputs[i];
      }

      outputs[outputCount[0]] = filter -> sum/FILTER_LENGTH;

      outputCount[0] += 1;
    }
  }

  return( filter -> readingAvailable );
}

void InitializeFilters( void )
{
  arm_fir_decimate_init_f32( &FirstSineInstance, FilterCoeffLength, 8, FilterCoeff, FirstInstanceSineState, BLOCK_SIZE );
  arm_fir_decimate_init_f32( &FirstCosineInstance, FilterCoeffLength, 8, FilterCoeff, FirstInstanceCosineState, BLOCK_SIZE );
  arm_fir_decimate_init_f32( &SecondSineInstance, FilterCoeffLength, 8, FilterCoeff, SecondInstanceSineState, BLOCK_SIZE/8 );
  arm_fir_decimate_init_f32( &SecondCosineInstance, FilterCoeffLength, 8, FilterCoeff, SecondInstanceCosineState, BLOCK_SIZE/8 );
  arm_fir_init_f32( &ThirdSineInstance, Filter1CoeffLength, Filter1Coeff, ThirdInstanceSineState, BLOCK_SIZE/64 );
  arm_fir_init_f32( &ThirdCosineInstance, Filter1CoeffLength, Filter1Coeff, ThirdInstanceCosineState, BLOCK_SIZE/64 );
  arm_fir_decimate_init_f32( &RETInstance, FilterRETCoeffsLength, 8, FilterRETCoeffs, RETInstanceState, BLOCK_SIZE/8 );
}

void InitializeDFTFilters( float32_t frequency, float32_t frequency0, float32_t sample_frequency )
{
  InitializeDFT( &Voltage1MatchedFilter, frequency, sample_frequency );

  InitializeDFT( &Voltage2MatchedFilter, frequency0, sample_frequency );

  InitMovingAverage( &Magnitude1Filter );

  InitMovingAverage( &Magnitude2Filter );
}

void InitializeProcData( struct _drd_state_ *state, float32_t frequency )
{
  state -> sinef32 = (float32_t) (sin(2.0*PI*(frequency)/SAMPLE_FREQUENCY));
  state -> cosinef32 = (float32_t) (cos(2.0*PI*(frequency)/SAMPLE_FREQUENCY));

  state -> currentSineF32 = 0.0;
  state -> currentCosineF32 = 1.0;
}

/*******************************************************************************
* Function: BART_Decoder
*
* Summary:  Processes the demod data to extract BART speed codes.
*
*******************************************************************************/

void BART_Decoder( struct _drd_state_ *state, short *buff, uint16_t size )
{
  uint16_t i;
  int Voltage1CountOut, Voltage2CountOut, Magnitude1CountOut, Magnitude2CountOut;
  float32_t Voltage1MagSquared, Voltage2MagSquared, FilteredVoltage1MagSquared, FilteredVoltage2MagSquared;
  
  SingleLineDFT( &Voltage1MatchedFilter, buff, Voltage1RealOutputs, Voltage1ImagOutputs, size, &Voltage1CountOut );

  SingleLineDFT( &Voltage2MatchedFilter, buff, Voltage2RealOutputs, Voltage2ImagOutputs, size, &Voltage2CountOut );

  if ( Voltage1CountOut >= size )
  {
    for( i = 0;i < size;i++ )
    {
      Voltage1MagSquared = Voltage1RealOutputs[i]*Voltage1RealOutputs[i] + Voltage1ImagOutputs[i]*Voltage1ImagOutputs[i];

      Voltage2MagSquared = Voltage2RealOutputs[i]*Voltage2RealOutputs[i] + Voltage2ImagOutputs[i]*Voltage2ImagOutputs[i];

      MovingAverage( &Magnitude1Filter, &Voltage1MagSquared, &FilteredVoltage1MagSquared, 1, &Magnitude1CountOut );

      MovingAverage( &Magnitude2Filter, &Voltage2MagSquared, &FilteredVoltage2MagSquared, 1, &Magnitude2CountOut );

      if ( Magnitude1CountOut && Magnitude2CountOut )
      {
        state -> DemodShiftCount++;

        state -> SampCount += 1;

	state -> PeakSquaredAvg = FilteredVoltage1MagSquared;

        //
        //           0 MPH   6 MPH   18 MPH  27 MPH  36 MPH  50 MPH  70 MPH  80 MPH
        // --------  ------  ------  ------  ------  ------  ------  ------  ------
        // PHASE 1   100000  100001  101001  100101  100011  101011  100111  101111
        // PHASE 2   010000  110000  110100  110010  110001  110101  110011  110111
        // PHASE 3   001000  011000  011010  011001  111000  111010  111001  111011
        // PHASE 4   000100  001100  001101  101100  011100  011101  111100  111101
        // PHASE 5   000010  000110  100110  010110  001110  101110  011110  111110
        // PHASE 6   000001  000011  010011  001011  000111  010111  001111  011111
        //
        ////////////////////////////////////////////////////////////////////////////////


        // State Machine for BART Decoding
        //////////////////////////////////
        switch( state -> DemodState )
        {
          // STATE NOCARRIER    If '1' seen: GOTO STATE_UNKNOWN_1 (reset period timer)

          case BART_STATE_NOCARRIER:
            if( FilteredVoltage1MagSquared >= state -> DemodThreshold )
            {
              state -> SampCount = 0;
              state -> DemodState = BART_STATE_1;
              state -> DemodShiftReg = 0;
            }
          break;

          // STATE_CONSTANT     If a '0' is seen, GOTO STATE_NOCARRIER

          case BART_STATE_CONSTANT:
            if ( FilteredVoltage2MagSquared >= state -> DemodThreshold )
            {
              state -> DemodState = BART_STATE_NOCARRIER;
              state -> DemodShiftReg = 0;
            }
          break;

          case BART_STATE_1:
            // Check if high too long

            if ( (FilteredVoltage1MagSquared >= state -> DemodThreshold) && (state -> SampCount > state -> DemodMaxOnCounts) )
            {
              state -> DemodState = BART_STATE_CONSTANT;
              state -> DemodShiftReg = 0x3F;
              state -> DemodShiftCount = 0;
              state -> DemodShiftIdx = 0x3F;
              state -> DemodShiftState = 0;
            }

            // detect low

            else if ( FilteredVoltage2MagSquared >= state -> DemodThreshold )
            {
              state -> DemodOnCounts = state -> SampCount;  // grab on time
              state -> SampCount = 0;

              if ( state -> DemodShiftState == 0 )
              {
                state -> DemodShiftState = 1;
                state -> DemodShiftCount = 0;
                state -> DemodShiftReg = 0;
                state -> DemodNextBitTime = state -> DemodHalfBit;
              }

              state -> DemodState = BART_STATE_0;
            }
          break;

          case BART_STATE_0:
            // Check if low too long
            ////////////////////////////////////////////////////////////

            if ( (FilteredVoltage2MagSquared >= state -> DemodThreshold) && (state -> SampCount > state -> DemodMaxOffCounts) )
            {
              state -> DemodState = BART_STATE_NOCARRIER;
              state -> DemodShiftReg = 0;
              state -> DemodShiftCount = 0;
              state -> DemodShiftIdx = 0;
              state -> DemodShiftState = 0;
            }

            // if a logic '1' is detected

            else if ( FilteredVoltage1MagSquared >= state -> DemodThreshold )
            {
              state -> DemodOnCounts = state -> SampCount;  // grab on time
              state -> SampCount = 0;

              if ( state -> DemodShiftState == 0)
              {
                state -> DemodShiftState = 1;
                state -> DemodShiftCount = 0;
                state -> DemodShiftReg = 0;
                state -> DemodNextBitTime = state -> DemodHalfBit;
              }

              state -> DemodState = BART_STATE_1;
            }
          break;

          default:
          break;
        } // end switch(gDemodState)

        if ( state -> DemodShiftState > 0)
        {
          if ( state -> DemodShiftCount >=  state -> DemodNextBitTime )
          {
            if ( state -> DemodState == BART_STATE_0)
            {
              state -> DemodShiftReg <<= 1;
            }
            else if ( state -> DemodState == BART_STATE_1)
            {
              state -> DemodShiftReg <<= 1;
              state -> DemodShiftReg |= 1;
            }

            state -> DemodNextBitTime += state -> DemodOneBit;
            state -> DemodShiftState++;
          }

          if ( state -> DemodShiftState == 7 )
          {
            state -> DemodShiftState = 0;
            state -> DemodShiftCount = 0;
            state -> DemodShiftIdx = state -> DemodShiftReg & 0x3F;
          }
        }
      }
    }

    state -> PeakAvg = sqrtf( state -> PeakSquaredAvg );
  }
} // BART_Decoder()

/*******************************************************************************
// Function: DRD11_Decoder
//
// Summary:  Processes the demod data to extract CAB codes.
//
// Details:
// FOr 100 Hz carrier: every 1 msecs (1000 Hz) Decode Cab Signalling
//
// Cab signalling consists of a carrier at 100 or 150 Hz (that I know of)
// and the modulation is just on/off keying at a particular rate.
// The algorithm used in a real cab will be closely mirrored here.
// The signal is sampled at 10 times the carrier (1000 Hz for 100 Hz).
// It is rectified and boxcar averaged over 20 samples.  This will result
// is a psuedo squarewave of the modulation signal with a small amount of
// ripple in the on times.  By setting a threshold, the on and off can be
// easily detected and the possible codes can be decoded.  The following
// table shows the possible cab states and modulation details.  PPM stands
// for pulses per minute.
//
//                                          NOMINAL  DUTY
// CODE                                     DUTY     CYCLE
// NAME        PPM     FREQUENCY  PERIOD    CYCLE    RANGE
// -------     ------  ---------  --------- -------- --------
// no carrier  NA      NA         NA        0        0
// unknown     NA      NA         NA        NA       NA
// constant    NA      NA         NA        100%     100%
// 75 CODE     68.68   1.144667   .8736     50%      31-69%
// 180 CODE    178.57  2.976167   .3360     50%      22-78%
// 270 CODE    267.86  4.464333   .2240     50%      16-84%
//
// A state machine is used to track the cab state and decode the modulation.
//
*******************************************************************************/
void DRD11_Decoder( struct _drd_state_ *state, int16_t *data, uint16_t size )
{
  float32_t sineTemp, cosineTemp;
  int j;

  for( j = 0;j < BLOCK_SIZE;m++, j++ )
  {
    if ( m >= SAMPLE_FREQUENCY )
    {
      m = 0;

      state -> currentSineF32 = 0.0;
      state -> currentCosineF32 = 1.0;
    }

    SineSamples[j] = data[j]*state -> currentSineF32;

    CosineSamples[j] = data[j]*state -> currentCosineF32;

    sineTemp = state -> currentSineF32*state -> cosinef32 + state -> currentCosineF32*state -> sinef32;

    cosineTemp = state -> currentCosineF32*state -> cosinef32 - state -> currentSineF32*state -> sinef32;

    state -> currentSineF32 = sineTemp;

    state -> currentCosineF32 = cosineTemp;
  }

  arm_fir_decimate_f32( &FirstSineInstance, SineSamples, FirstInstanceSineOutputs, BLOCK_SIZE );
  arm_fir_decimate_f32( &FirstCosineInstance, CosineSamples, FirstInstanceCosineOutputs, BLOCK_SIZE );

  arm_fir_decimate_f32( &SecondSineInstance, FirstInstanceSineOutputs, SecondInstanceSineOutputs, BLOCK_SIZE/8 );
  arm_fir_decimate_f32( &SecondCosineInstance, FirstInstanceCosineOutputs, SecondInstanceCosineOutputs, BLOCK_SIZE/8 );

  arm_fir_f32( &ThirdSineInstance, SecondInstanceSineOutputs, ThirdInstanceSineOutputs, BLOCK_SIZE/64 );
  arm_fir_f32( &ThirdCosineInstance, SecondInstanceCosineOutputs, ThirdInstanceCosineOutputs, BLOCK_SIZE/64 );

  for( j = 0;j < BLOCK_SIZE/64;j++ )
  {
    state -> SampCount += 1;

    state -> DemodAvg = ThirdInstanceSineOutputs[j]*ThirdInstanceSineOutputs[j] + ThirdInstanceCosineOutputs[j]*ThirdInstanceCosineOutputs[j];

    state -> PeakSquaredAvg = .25f*state -> PeakSquaredAvg + .75f*state -> DemodAvg;

#ifdef DEBUG_PRINT
    if (state -> DebugFlags & 3)
    {
      sprintf(output, "\n");
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );
    }

    if ( state -> DebugFlags & 2 )
    {
      sprintf(output, "\n%d ", state -> DemodAvg);
    }
#endif

#ifdef DEBUG_PRINT
    state -> LastDemodState = state -> DemodState;
#endif


    // State Machine for CAB Decoding
    /////////////////////////////////

    switch( state -> DemodState )
    {
      // STATE NOCARRIER    If '1' seen: GOTO STATE_UNKNOWN_1 (reset period timer)

      case CAB_STATE_NOCARRIER:
        if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          state -> SampCount = 0;
          state -> DemodOnCounts = 0;
          state -> Period = 0;
          state -> DemodState = CAB_STATE_UNKNOWN_1;
        }
      break;

      // decoding unknown logic '1'

      case CAB_STATE_UNKNOWN_1:
        // Check if high too long
	
        if ( ( state -> DemodAvg >=  state -> DemodThreshold ) && ( 64*(state -> SampCount) >  state -> CabTimes.DRD11.Cab075MaxOn ) )
        {
          state -> DemodDuty = 100;
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> Period = 64*(state -> SampCount);
          state -> DemodState = CAB_STATE_CONSTANT;
        }

        // detect low
	
        else if ( state -> DemodAvg <  state -> DemodThreshold )
        {
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> Period = 0;
          state -> DemodState = CAB_STATE_UNKNOWN_0;
        }
      break;

      // decoding unknown logic '0'

      case CAB_STATE_UNKNOWN_0:
        // Check if low too long
	
        if ( (state -> DemodAvg < state -> DemodThreshold) && (64*(state -> SampCount) > state -> DemodMaxOffCounts) )
        {
          state -> DemodDuty = 0;
          state -> Period = 64*(state -> SampCount);
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> DemodState = CAB_STATE_NOCARRIER;
        }

        // if a logic '1' is detected

        else if ( state -> DemodAvg >=  state -> DemodThreshold )
        {
          state -> Period = 64*(state -> SampCount);

          // see if previous period and on-time matches code 270

          if ( ( 64*(state -> SampCount) >=  state -> CabTimes.DRD11.Cab270MinPer ) && ( 64*(state -> SampCount) <= state -> CabTimes.DRD11.Cab270MaxPer ) &&
               ( state -> DemodOnCounts >=  state -> CabTimes.DRD11.Cab270MinOn ) && ( state -> DemodOnCounts <=  state -> CabTimes.DRD11.Cab270MaxOn ) )
          {
            state -> DemodDuty = CalcDutyCycle( state -> DemodOnCounts, 64*(state -> SampCount) );
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;
            state -> DemodState = CAB_STATE_270_1;
          }

          // see if previous period and on-time matches code 180

          else if ( ( 64*(state -> SampCount) >= state -> CabTimes.DRD11.Cab180MinPer ) && ( 64*(state -> SampCount) <= state -> CabTimes.DRD11.Cab180MaxPer ) &&
                    ( state -> DemodOnCounts >= state -> CabTimes.DRD11.Cab180MinOn ) && ( state -> DemodOnCounts <= state -> CabTimes.DRD11.Cab180MaxOn ) )
          {
            state -> DemodDuty = CalcDutyCycle( state -> DemodOnCounts, 64*(state -> SampCount) );
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;
            state -> DemodState = CAB_STATE_180_1;
          }

          // see if previous period and on-time matches code 75

          else if ( ( 64*(state -> SampCount) >= state -> CabTimes.DRD11.Cab075MinPer ) && ( 64*(state -> SampCount) <= state -> CabTimes.DRD11.Cab075MaxPer ) &&
                    ( state -> DemodOnCounts >= state -> CabTimes.DRD11.Cab075MinOn ) && ( state -> DemodOnCounts <= state -> CabTimes.DRD11.Cab075MaxOn ) )
          {
            state -> DemodDuty = CalcDutyCycle( state -> DemodOnCounts, 64*(state -> SampCount) );
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;
            state -> DemodState = CAB_STATE_75_1;
          }

          // else no match so unknown

          else
          {
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;
            state -> DemodState = CAB_STATE_UNKNOWN_1;
          }
        }
      break;

      // decoding constant carrier

      case CAB_STATE_CONSTANT:
        // If a '0' is seen, GOTO STATE_NOCARRIER
	
        if ( state -> DemodAvg < state -> DemodThreshold )
        {
          state -> DemodDuty = 0;
          state -> Period = 0;
          state -> DemodOnCounts = 0;  // grab on time
          state -> DemodState = CAB_STATE_NOCARRIER;
        }
      break;

      // decoding code CAB 75 logic '1'

      case CAB_STATE_75_1:
        // Check if high too long

        if ( ( state -> DemodAvg >= state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> CabTimes.DRD11.Cab075MaxOn ) )
        {
          state -> DemodDuty = 100;
          state -> Period = 64*(state -> SampCount);
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> DemodState = CAB_STATE_CONSTANT;
        }

        // detect low
	
        else if ( state -> DemodAvg < state -> DemodThresholdHyst )
        {
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> Period = 0;

          // check that the on time is right for this code

          if ( ( 64*(state -> SampCount) >= state -> CabTimes.DRD11.Cab075MinOn ) && ( 64*(state -> SampCount) <= state -> CabTimes.DRD11.Cab075MaxOn ) )
          {
            state -> DemodState = CAB_STATE_75_0;
          }
          else
          {
            state -> DemodState = CAB_STATE_UNKNOWN_0;
          }
        }
      break;

      // decoding code CAB 75 logic '0'

      case CAB_STATE_75_0:
        // Check if low too long
	
        if ( ( state -> DemodAvg < state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> DemodMaxOffCounts ) )
        {
          state -> Period = 64*(state -> SampCount);
          state -> DemodState = CAB_STATE_UNKNOWN_0;
        }

        // if a logic '1' is detected

        else if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          state -> Period = 64*(state -> SampCount);

          // if the on-time is correct for code 75

          if ( ( 64*(state -> SampCount) >= state -> CabTimes.DRD11.Cab075MinPer ) && ( 64*(state -> SampCount) <= state -> CabTimes.DRD11.Cab075MaxPer ) )
          {
            state -> DemodDuty = CalcDutyCycle( state -> DemodOnCounts,  64*(state -> SampCount) );
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;                        // reset on counts
            state -> DemodState = CAB_STATE_75_1;
          }

          // else does not match code 75 so go to unknown

          else
          {
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;                        // reset on counts
            state -> DemodState = CAB_STATE_UNKNOWN_1;
          }
        }
      break;

      // decoding code 180 logic '1'

      case CAB_STATE_180_1:
        // Check if high too long
	
        if ( ( state -> DemodAvg >= state -> DemodThreshold ) && ( 64*(state -> SampCount) >  state -> CabTimes.DRD11.Cab180MaxOn ) )
        {
          state -> Period = 64*(state -> SampCount);
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> DemodState = CAB_STATE_UNKNOWN_1;
        }

        // detect low
	
        else if ( state -> DemodAvg < state -> DemodThresholdHyst )
        {
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> Period = 0;

          // check that the on time is right for this code

          if ( ( 64*(state -> SampCount) >= state -> CabTimes.DRD11.Cab180MinOn) && ( 64*(state -> SampCount) <= state -> CabTimes.DRD11.Cab180MaxOn ) )
          {
            state -> DemodState = CAB_STATE_180_0;
          }

          // wrong on time so go to unknown

          else
          {
            state -> DemodState = CAB_STATE_UNKNOWN_0;
          }
        }
      break;

      // decoding code 180 logic '0'

      case CAB_STATE_180_0:
        // Check if low too long
	
        if ( ( state -> DemodAvg < state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> CabTimes.DRD11.Cab180MaxPer ) )
        {
          state -> Period = 64*(state -> SampCount);
          state -> DemodState = CAB_STATE_UNKNOWN_0;
        }

        // if a logic '1' is detected
	
        else if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          state -> Period = 64*(state -> SampCount);

          // if the on-time is correct for code 180

          if ( ( 64*(state -> SampCount) >= state -> CabTimes.DRD11.Cab180MinPer ) && ( 64*(state -> SampCount) <= state -> CabTimes.DRD11.Cab180MaxPer ) )
          {
            state -> DemodDuty = CalcDutyCycle( state -> DemodOnCounts, 64*(state -> SampCount) );
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;                        // reset on counts
            state -> DemodState = CAB_STATE_180_1;
          }

          // else does not match code 180 so go to unknown

          else
          {
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;                        // reset on counts
            state -> DemodState = CAB_STATE_UNKNOWN_1;
          }
        }
      break;

      // decoding code CAB 270 logic '1'

      case CAB_STATE_270_1:
        // Check if high too long
	
        if ( ( state -> DemodAvg >= state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> CabTimes.DRD11.Cab270MaxOn ) )
        {
          state -> Period = 64*(state -> SampCount);
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> DemodState = CAB_STATE_UNKNOWN_1;
        }

        // detect low
	
        else if ( state -> DemodAvg < state -> DemodThresholdHyst )
        {
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> Period = 0;

          if ( ( 64*(state -> SampCount) >= state -> CabTimes.DRD11.Cab270MinOn ) && ( 64*(state -> SampCount) <= state -> CabTimes.DRD11.Cab270MaxOn ) )
          {
            state -> DemodState = CAB_STATE_270_0;
          }
          else
          {
            state -> DemodState = CAB_STATE_UNKNOWN_0;
          }
        }
      break;

      // decoding code CAB 270 logic '0'

      case CAB_STATE_270_0:
        // Check if low too long

        if ( ( state -> DemodAvg < state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> CabTimes.DRD11.Cab270MaxPer ) )
        {
          state -> Period = 64*(state -> SampCount);
          state -> DemodState = CAB_STATE_UNKNOWN_0;
        }

        // if a logic '1' is detected
	
        else if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          state -> Period = 64*(state -> SampCount);

          // if the period is correct for code 270

          if ( ( 64*(state -> SampCount) > state -> CabTimes.DRD11.Cab270MinPer ) && ( 64*(state -> SampCount) < state -> CabTimes.DRD11.Cab270MaxPer ) )
          {
            state -> DemodDuty = CalcDutyCycle( state -> DemodOnCounts,  64*(state -> SampCount) );
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;                        // reset on counts
            state -> DemodState = CAB_STATE_270_1;
          }

          // else does not match code 270 so go to unknown

          else
          {
            state -> SampCount = 0;
            state -> DemodOnCounts = 0;                        // reset on counts
            state -> DemodState = CAB_STATE_UNKNOWN_1;
          }
        }
      break;

      default:
      break;
    } // end switch(gDemodState)

#ifdef DEBUG_PRINT
    // for debugging, detect state change
    //if((gDebugFlags & 4) && (gLastDemodState != gDemodState))
    if ( state -> LastDemodState != state -> DemodState )
    {
      sprintf( output, "\n" );
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

      PrintState( state -> LastDemodState);

      sprintf( ouput, " -> " );
      SendBytes( LPC_USART0, &USART0TransmitRingBuffer, output, strlen(output) );

      PrintState( state -> DemodState );

      if ( state -> DemodOnCounts != 0 )
      {
        sprintf( output, "  ON=%ld", state -> DemodOnCounts );
      }

      if ( state -> Period != 0 )
      {
        sprintf( output, "  PER=%ld", state -> Period );
      }
    }
#endif
  }

  state -> PeakAvg = sqrtf( state -> PeakSquaredAvg );
} // end DRD11_Decoder()

/*******************************************************************************
// Function: DRD_AFCTDecoder
//
// Summary:  Decodes AFTC modulation.
//
// Details:
// AFTC signalling consists of a carrier and the modulation is just on/off
// keying at a particular rate and duty cycle.  There are three codes, A, B and
// C.  The following are the details.
//   Mod A ==> On Time = 300mS, Period = 360mS
//   Mod B ==> On Time = 400mS, Period = 460mS
//   Mod C ==> On Time = 500mS, Period = 560mS
//
// Because current measuring must be done with modulation present, 
// Original DRD1-RET threshold was set to around 175mA rms (.05V pk).
//
// Demodulation is done by sampling at 10X the carrier frequency, rectifying the
// samples, and then doing a boxcar average of that.  The sampling and
// rectification are done in the Timer0 ISR.  The averaging is done here.
//
*******************************************************************************/
void DRD_AFTCDecoder( struct _drd_state_ *state, int16_t *data, uint16_t size )
{
  int    j;
  float32_t sineTemp, cosineTemp, temp;

  for( j = 0;j < BLOCK_SIZE;m++, j++ )
  {
    if ( m >= SAMPLE_FREQUENCY )
    {
      m = 0;

      state -> currentSineF32 = 0.0;
      state -> currentCosineF32 = 1.0;
    }

    SineSamples[j] = data[j]*state -> currentSineF32;

    CosineSamples[j] = data[j]*state -> currentCosineF32;

    sineTemp = state -> currentSineF32*state -> cosinef32 + state -> currentCosineF32*state -> sinef32;

    cosineTemp = state -> currentCosineF32*state -> cosinef32 - state -> currentSineF32*state -> sinef32;

    state -> currentSineF32 = sineTemp;

    state -> currentCosineF32 = cosineTemp;
  }

  arm_fir_decimate_f32( &FirstSineInstance, SineSamples, FirstInstanceSineOutputs, BLOCK_SIZE );
  arm_fir_decimate_f32( &FirstCosineInstance, CosineSamples, FirstInstanceCosineOutputs, BLOCK_SIZE );

  arm_fir_decimate_f32( &SecondSineInstance, FirstInstanceSineOutputs, SecondInstanceSineOutputs, BLOCK_SIZE/8 );
  arm_fir_decimate_f32( &SecondCosineInstance, FirstInstanceCosineOutputs, SecondInstanceCosineOutputs, BLOCK_SIZE/8 );

  for( j = 0;j < BLOCK_SIZE/64;j++ )
  {
    state -> SampCount += 1;

    state -> DemodAvg = SecondInstanceSineOutputs[j]*SecondInstanceSineOutputs[j] + SecondInstanceCosineOutputs[j]*SecondInstanceCosineOutputs[j];

    if ( state -> DemodAvg > .998*state -> PeakDemodAvg ) 
    {
      state -> PeakDemodAvg = state -> DemodAvg;
    }

    temp = state -> PeakSquaredAvg1[0];

    state -> PeakSquaredAvg1[0] = FILTER_COEFF_1*state -> PeakSquaredAvg1[0] - FILTER_COEFF_2*state -> PeakSquaredAvg1[1] + (1.0 - LOW_PASS_GAIN)*(1.0 - LOW_PASS_GAIN)*state -> PeakDemodAvg;
    
    state -> PeakSquaredAvg1[1] = temp;

    temp = state -> PeakSquaredAvg;

    state -> PeakSquaredAvg = FILTER_COEFF_1*state -> PeakSquaredAvg - FILTER_COEFF_2*state -> PeakSquaredAvg1[2] + (1.0 - LOW_PASS_GAIN)*(1.0 - LOW_PASS_GAIN)*state -> PeakSquaredAvg1[0];
    
    state -> PeakSquaredAvg1[2] = temp;

    state -> PeakDemodAvg *= .999;

    state -> LastDemodState = state -> DemodState;

    // State Machine for AFTC Decoding
    //////////////////////////////////

    switch( state -> DemodState )
    {
      case AFTC_STATE_NOCARRIER:

        // If '1' seen: GOTO STATE_UNKNOWN_1

        if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          state -> SampCount = 0;                          // zero timer when low to high transition occurs
          state -> DemodOnCounts = 0;
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_UNKNOWN_1;
        }
      break;

      // decoding unknown code logic '1'

      case AFTC_STATE_UNKNOWN_1:

        // Check if high too long

        if ( ( state -> DemodAvg >= state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> DemodMaxOnCounts ) )
        {
          state -> DemodOnCounts = 64*(state -> SampCount); // grab on time
          state -> ScanForPeaks = 1;                        // always scan for peaks with constant carrier
          state -> DemodState = AFTC_STATE_CONSTANT;
        }

        // detect low

        else if ( state -> DemodAvg < state -> DemodThresholdHyst )
        {
          state -> DemodOnCounts = 64*(state -> SampCount); // grab on time every time a low transition occurs
          state -> ScanForPeaks = 0;                        // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_UNKNOWN_0;
        }
      break;

      // decoding unknown code logic '0'

      case AFTC_STATE_UNKNOWN_0:
        // Check if low too long

        //if((gDemodAvg < gDemodThreshold) && (gSampCount > gDemodMaxOffCounts))

        if ( ( state -> DemodAvg < state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> AFTC_C_MaxPer ) )
        {
          state -> DemodOnCounts = 64*(state -> SampCount);             // grab on time every time a low transition occurs
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_NOCARRIER;
        }

        // if a logic '1' is detected

        else if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          // see if previous period and on-time matches code A
  	
          if ( ( 64*(state -> SampCount) >= state -> AFTC_A_MinPer ) && ( 64*(state -> SampCount) <= state -> AFTC_A_MaxPer ) &&
               ( state -> DemodOnCounts >= state -> AFTC_A_MinOn) && (state -> DemodOnCounts <= state -> AFTC_A_MaxOn ) )
          {
            state -> DemodState = AFTC_STATE_A_1;
          }

          // see if previous period and on-time matches code B
	
          else if ( ( 64*(state -> SampCount) >= state -> AFTC_B_MinPer) && ( 64*(state -> SampCount) <= state -> AFTC_B_MaxPer ) &&
                    ( state -> DemodOnCounts >= state -> AFTC_B_MinOn ) && ( state -> DemodOnCounts <= state -> AFTC_B_MaxOn ) )
          {
            state -> DemodState = AFTC_STATE_B_1;
          }

          // see if previous period and on-time matches code C
	
          else if ( ( 64*(state -> SampCount) >= state -> AFTC_C_MinPer ) && ( 64*(state -> SampCount) <= state -> AFTC_C_MaxPer ) &&
                    ( state -> DemodOnCounts >= state -> AFTC_C_MinOn ) && ( state -> DemodOnCounts <= state -> AFTC_C_MaxOn ) )
          {
            state -> DemodState = AFTC_STATE_C_1;
          }

          // else no match so unknown
	
          else
          {
            state -> DemodState = AFTC_STATE_UNKNOWN_1;
          }

          state -> OnCounts = state -> DemodOnCounts;
          state -> Period = 64*(state -> SampCount);
          state -> SampCount = 0;                          // zero timer when low to high transition occurs
          state -> DemodOnCounts = 0;
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present
        }
      break;

      case AFTC_STATE_CONSTANT:
        // If a '0' is seen, GOTO STATE_NOCARRIER

        if ( state -> DemodAvg < state -> DemodThresholdHyst )
        {
          state -> DemodOnCounts = 0;                      // grab on time every time a low transition occurs
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_NOCARRIER;
        }
      break;

      // decoding code C logic '1'

      case AFTC_STATE_C_1:
        // Check if high too long

        if ( ( state -> DemodAvg >= state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> AFTC_C_MaxOn ) )
        {
          state -> DemodOnCounts = 64*(state -> SampCount);  // grab on time
          state -> ScanForPeaks = 1;                       // always scan for peaks with constant carrier
          state -> DemodState = AFTC_STATE_CONSTANT;
        }

        // detect low
      
        else if ( state -> DemodAvg < state -> DemodThresholdHyst )
        {
          state -> DemodOnCounts = 64*(state -> SampCount);             // grab on time every time a low transition occurs
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present

          // check that the on time is right for this code
	
          if ( ( 64*(state -> SampCount) >= state -> AFTC_C_MinOn) && ( 64*(state -> SampCount) <= state -> AFTC_C_MaxOn ) )
          {
            state -> DemodState = AFTC_STATE_C_0;
          }
          else
          {
            state -> DemodState = AFTC_STATE_UNKNOWN_0;
          }
        }
      break;

      // decoding code C logic '0'

      case AFTC_STATE_C_0:
        // Check if low too long

        if ( ( state -> DemodAvg < state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> AFTC_C_MaxPer ) )
        {
          state -> Period = 64*(state -> SampCount);
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_UNKNOWN_0;
        }

        // if a logic '1' is detected

        else if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present

          // if the on-time is correct for code C
	
          if ( ( 64*(state -> SampCount) >= state -> AFTC_C_MinPer ) && ( 64*(state -> SampCount) <= state -> AFTC_C_MaxPer ) )
          {
            state -> DemodState = AFTC_STATE_C_1;
          }

          // else does not match code C so go to unknown
	
          else
          {
            state -> DemodState = AFTC_STATE_UNKNOWN_1;
          }

          state -> OnCounts = state -> DemodOnCounts;
          state -> Period = 64*(state -> SampCount);
          state -> SampCount = 0;                          // zero timer when low to high transition occurs
          state -> DemodOnCounts = 0;
        }
      break;

      // decoding code B logic '1'

      case AFTC_STATE_B_1:
        // Check if high too long

        if ( ( state -> DemodAvg >= state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> AFTC_B_MaxOn ) )
        {
          state -> DemodOnCounts = 64*(state -> SampCount); // grab on time
          state -> ScanForPeaks = 0;                        // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_UNKNOWN_1;
        }

        // detect low

        else if ( state -> DemodAvg < state -> DemodThresholdHyst )
        {
          state -> DemodOnCounts = 64*(state -> SampCount); // grab on time every time a low transition occurs

          // check that the on time is right for this code
	
          if (( 64*(state -> SampCount) >= state -> AFTC_B_MinOn ) && ( 64*(state -> SampCount) <= state -> AFTC_B_MaxOn ) )
          {
            state -> DemodState = AFTC_STATE_B_0;
          }

          // wrong on time so go to unknown
	
          else
          {
            state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present
            state -> DemodState = AFTC_STATE_UNKNOWN_0;
          }
        }
      break;

      // decoding code B logic '0'

      case AFTC_STATE_B_0:
        // Check if low too long

        if ( ( state -> DemodAvg < state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> AFTC_B_MaxPer ) )
        {
          state -> Period = 64*(state -> SampCount);
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_UNKNOWN_0;
        }

        // if a logic '1' is detected

        else if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present

          // if the on-time is correct for code B
	
          if ( ( 64*(state -> SampCount) >= state -> AFTC_B_MinPer ) && ( 64*(state -> SampCount) <= state -> AFTC_B_MaxPer ) )
          {
            state -> DemodState = AFTC_STATE_B_1;
          }

          // else does not match code B so go to unknown
	
          else
          {
            state -> DemodState = AFTC_STATE_UNKNOWN_1;
          }

          state -> OnCounts = state -> DemodOnCounts;
          state -> Period = 64*(state -> SampCount);
          state -> SampCount = 0;                          // zero timer when low to high transition occurs
          state -> DemodOnCounts= 0;
        }
      break;

      // decoding code A logic '1'

      case AFTC_STATE_A_1:
        // Check if high too long

        if ( ( state -> DemodAvg >= state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> AFTC_A_MaxOn ) )
        {
          state -> DemodOnCounts = 64*(state -> SampCount); // grab on time
          state -> ScanForPeaks = 0;                        // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_UNKNOWN_1;
        }

        // detect low

        else if ( state -> DemodAvg < state -> DemodThresholdHyst )
        {
          state -> DemodOnCounts = 64*(state -> SampCount); // grab on time every time a low transition occurs

          if ( ( 64*(state -> SampCount) >= state -> AFTC_A_MinOn ) && ( 64*(state -> SampCount) <= state -> AFTC_A_MaxOn ) )
          {
            state -> DemodState = AFTC_STATE_A_0;
          }
          else
          {
            state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present
            state -> DemodState = AFTC_STATE_UNKNOWN_0;
          }
        }
      break;

      // decoding code A logic '0'

      case AFTC_STATE_A_0:
        // Check if low too long

        if ( ( state -> DemodAvg < state -> DemodThreshold ) && ( 64*(state -> SampCount) > state -> AFTC_A_MaxPer ) )
        {
          state -> Period = 64*(state -> SampCount);
          state -> ScanForPeaks= 0;                       // stop scanning for peaks until a known modulation is present
          state -> DemodState = AFTC_STATE_UNKNOWN_0;
        }

        // if a logic '1' is detected

        else if ( state -> DemodAvg >= state -> DemodThreshold )
        {
          state -> ScanForPeaks = 0;                       // stop scanning for peaks until a known modulation is present

          // if the period is correct for code A
	
          if ( ( 64*(state -> SampCount) > state -> AFTC_A_MinPer ) && ( 64*(state -> SampCount) < state -> AFTC_A_MaxPer ) )
          {
            state -> DemodState= AFTC_STATE_A_1;
          }

          // else does not match code A so go to unknown
	
          else
          {
            state -> DemodState = AFTC_STATE_UNKNOWN_1;
          }

          state -> OnCounts = state -> DemodOnCounts;
          state -> Period = 64*(state -> SampCount);
          state -> SampCount = 0;                          // zero timer when low to high transition occurs
          state -> DemodOnCounts = 0;
        }
      break;

      default:
      break;
    } // end switch(gDemodState)
  }

  state -> PeakAvg = sqrtf( state -> PeakSquaredAvg )/AFTC_FILTER_CONSTANT;
} // end DRD_AFTCDecoder()

/*******************************************************************************
* Function: DRD_ROW_CABDecoder
*
* Summary:  Processes the demod data to extract CAB codes.
*
* Details:
*
* For 2340 Hz carrier: every 42.7 uS (23.4KHz Hz) Decode Cab Signalling
*
* ROW Cab signalling consists of a carrier at 2340Hz and
* modulation is just on/off keying at a particular rate.
* The algorithm used in a real cab will be closely mirrored here.
* The signal is sampled at 10 times the carrier (23.4 KHz for 2340 Hz).
* It is rectified and boxcar averaged over 20 samples.  This will result
* is a psuedo squarewave of the modulation signal with a small amount of
* ripple in the on times.  By setting a threshold, the on and off can be
* easily detected and the possible codes can be decoded.
*
* A state machine is used to track the cab state and decode the modulation.
*
*******************************************************************************/
void DRD_ROW_CABDecoder( struct _drd_state_ *state, int16_t *data, uint16_t size )
{
  int j;
  float32_t sineTemp, cosineTemp;

  for( j = 0;j < BLOCK_SIZE;m++, j++ )
  {
    if ( m >= SAMPLE_FREQUENCY )
    {
      m = 0;

      state -> currentSineF32 = 0.0;
      state -> currentCosineF32 = 1.0;
    }

    SineSamples[j] = data[j]*state -> currentSineF32;

    CosineSamples[j] = data[j]*state -> currentCosineF32;

    sineTemp = state -> currentSineF32*state -> cosinef32 + state -> currentCosineF32*state -> sinef32;

    cosineTemp = state -> currentCosineF32*state -> cosinef32 - state -> currentSineF32*state -> sinef32;

    state -> currentSineF32 = sineTemp;

    state -> currentCosineF32 = cosineTemp;
  }

  arm_fir_decimate_f32( &FirstSineInstance, SineSamples, FirstInstanceSineOutputs, BLOCK_SIZE );
  arm_fir_decimate_f32( &FirstCosineInstance, CosineSamples, FirstInstanceCosineOutputs, BLOCK_SIZE );

  arm_fir_decimate_f32( &SecondSineInstance, FirstInstanceSineOutputs, SecondInstanceSineOutputs, BLOCK_SIZE/8 );
  arm_fir_decimate_f32( &SecondCosineInstance, FirstInstanceCosineOutputs, SecondInstanceCosineOutputs, BLOCK_SIZE/8 );

  for( j = 0;j < BLOCK_SIZE/64;j++ )
  {
    state -> SampCount += 1;

    state -> DemodAvg = SecondInstanceSineOutputs[j]*SecondInstanceSineOutputs[j] + SecondInstanceCosineOutputs[j]*SecondInstanceCosineOutputs[j];

    state -> PeakSquaredAvg = .25f*state -> PeakSquaredAvg + .75f*state -> DemodAvg;

    // IF current signal sample is "ON".

    if ( state -> DemodAvg >= state -> DemodThreshold )
    {
      // Handle change from signal "OFF" to "ON".

      if ( state -> signal_state != 1 )
      {
        state -> signal_state = 1;
        state -> check_period = 1;
        state -> last_cab_off_time = 64*(state -> SampCount - 1);
        state -> SampCount = 1;
      }

      // Check for signal that has been "ON" greater than 555ms.

      if ( 64*(state -> SampCount) >= state -> CabTimes.ROW.DRD_ROW_CAB_MaxOn )
      {
        state -> last_cab_on_time = state -> CabTimes.ROW.DRD_ROW_CAB_MaxOn;
        state -> last_cab_off_time = 0;
        state -> SampCount = 1;
      }
    }
    else if (state -> DemodAvg < state -> DemodThresholdHyst)
    {
      // Handle change from signal "ON" to "OFF".

      if ( state -> signal_state != 0 )
      {
        state -> signal_state = 0;
        state -> last_cab_on_time = 64*(state -> SampCount - 1);
        state -> SampCount = 1;
      }

      // Set state to No Carrier if signal has been "OFF" greater than 2368ms.

      if ( 64*(state -> SampCount) >= state -> CabTimes.ROW.DRD_ROW_CAB_MaxOff )
      {
        state -> last_cab_off_time = state -> CabTimes.ROW.DRD_ROW_CAB_MaxOff;
        state -> last_cab_on_time = 0;
        state -> SampCount = 1;
      }
    }

    state -> LastDemodState = state -> DemodState;

    // Determine the current CAB code state based on the ON/OFF times.

    if ( state -> last_cab_on_time >= state -> CabTimes.ROW.DRD_ROW_CAB_MaxOn )
    {
      state -> ScanForPeaks = 1;  // Always scan for peaks with CC.
      state -> DemodState = DRD_ROW_CAB_STATE_CONSTANT;
    }
    else if ( state -> last_cab_off_time >= state -> CabTimes.ROW.DRD_ROW_CAB_MaxOff )
    {
      // Only clear buffers/states on the transition to NOCARRIER from another state.

      if ( state -> DemodState != DRD_ROW_CAB_STATE_NOCARRIER )
      {
        state -> DemodState = DRD_ROW_CAB_STATE_NOCARRIER;

      }
    }
    else
    {
      if ( state -> check_period )
      {
        state -> check_period = 0;

        state -> Period = state -> last_cab_on_time + state -> last_cab_off_time;

        if (state -> Period > state -> CabTimes.ROW.DRD_ROW_70_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_70_MaxPer)
        {
          state -> DemodState = DRD_ROW_CAB_STATE_70;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_65_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_65_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_65;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_60_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_60_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_60;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_50_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_50_MaxPer)
        {
          state -> DemodState = DRD_ROW_CAB_STATE_50;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_50LOS_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_50LOS_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_50LOS;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_40_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_40_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_40;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_30_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_30_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_30;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_20_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_20_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_20;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_15_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_15_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_15;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_10_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_10_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_10;
        }
        else if ( state -> Period > state -> CabTimes.ROW.DRD_ROW_10LOS_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_10LOS_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_10LOS;
        }
        else if (state -> Period > state -> CabTimes.ROW.DRD_ROW_0_MinPer && state -> Period < state -> CabTimes.ROW.DRD_ROW_0_MaxPer )
        {
          state -> DemodState = DRD_ROW_CAB_STATE_0;
        }
        else 
        {
          state -> DemodState = DRD_ROW_CAB_STATE_UNKNOWN;
          state -> ScanForPeaks = 0;         // stop scanning for peaks until a known modulation is present
        }
      }
    }
  }

  state -> PeakAvg = sqrtf( state -> PeakSquaredAvg );
} // end DRD_ROW_CABDecoder()

/*******************************************************************************
* Function: DRD_MFOR_CABDecoder
*
* Summary:  Processes the demod data to extract CAB codes.
*
* Details: 
*
* For 2340 Hz carrier: every 42.7 uS (23.4KHz Hz) Decode Cab Signalling
*
* MFOR Cab signalling consists of a carrier at 2340Hz and 
* modulation is just on/off keying at a particular rate.
* The algorithm used in a real cab will be closely mirrored here.
* The signal is sampled at 10 times the carrier (23.4 KHz for 2340 Hz).
* It is rectified and boxcar averaged over 20 samples.  This will result
* is a psuedo squarewave of the modulation signal with a small amount of
* ripple in the on times.  By setting a threshold, the on and off can be
* easily detected and the possible codes can be decoded.
*
* A state machine is used to track the cab state and decode the modulation.
*
*******************************************************************************/
void DRD_MFOR_CABDecoder( struct _drd_state_ *state, int16_t *data, uint16_t size )
{
  int j;
  float32_t sineTemp, cosineTemp;

  for( j = 0;j < BLOCK_SIZE;m++, j++ )
  {
    if ( m >= SAMPLE_FREQUENCY )
    {
      m = 0;

      state -> currentSineF32 = 0.0;
      state -> currentCosineF32 = 1.0;
    }

    SineSamples[j] = data[j]*state -> currentSineF32;

    CosineSamples[j] = data[j]*state -> currentCosineF32;

    sineTemp = state -> currentSineF32*state -> cosinef32 + state -> currentCosineF32*state -> sinef32;

    cosineTemp = state -> currentCosineF32*state -> cosinef32 - state -> currentSineF32*state -> sinef32;

    state -> currentSineF32 = sineTemp;

    state -> currentCosineF32 = cosineTemp;
  }

  arm_fir_decimate_f32( &FirstSineInstance, SineSamples, FirstInstanceSineOutputs, BLOCK_SIZE );
  arm_fir_decimate_f32( &FirstCosineInstance, CosineSamples, FirstInstanceCosineOutputs, BLOCK_SIZE );

  arm_fir_decimate_f32( &SecondSineInstance, FirstInstanceSineOutputs, SecondInstanceSineOutputs, BLOCK_SIZE/8 );
  arm_fir_decimate_f32( &SecondCosineInstance, FirstInstanceCosineOutputs, SecondInstanceCosineOutputs, BLOCK_SIZE/8 );

  for( j = 0;j < BLOCK_SIZE/64;j++ )
  {
    state -> SampCount += 1;

    state -> DemodAvg = SecondInstanceSineOutputs[j]*SecondInstanceSineOutputs[j] + SecondInstanceCosineOutputs[j]*SecondInstanceCosineOutputs[j];

    if ( state -> DemodAvg > .998*state -> PeakDemodAvg ) 
    {
      state -> PeakDemodAvg = state -> DemodAvg;
    }

    state -> PeakSquaredAvg = .998f*state -> PeakSquaredAvg + .002f*state -> PeakDemodAvg;

    state -> PeakDemodAvg *= .999;

    // IF current signal sample is "ON".

    if ( state -> DemodAvg >= state -> DemodThreshold )
    {
      // Handle change from signal "OFF" to "ON".

      if ( state -> signal_state != 1 )
      {
        state -> signal_state = 1;
        state -> check_period = 1;
        state -> last_cab_off_time = 64*(state -> SampCount - 1);
        state -> SampCount = 1;
      }

      // Check for signal that has been "ON" greater than 555ms.

      if ( 64*(state -> SampCount) >= state -> CabTimes.MFOR.DRD_MFOR_CAB_MaxOn )
      {
        state -> last_cab_on_time = state -> CabTimes.MFOR.DRD_MFOR_CAB_MaxOn;
        state -> last_cab_off_time = 0;
        state -> SampCount = 1;
      }
    }
    else if ( state -> DemodAvg < state -> DemodThresholdHyst )
    {
      // Handle change from signal "ON" to "OFF".

      if ( state -> signal_state != 0 )
      {
        state -> signal_state = 0;
        state -> last_cab_on_time = 64*(state -> SampCount - 1);
        state -> SampCount = 1;
      }

      // Set state to No Carrier if signal has been "OFF" greater than 2368ms.

      if ( 64*(state -> SampCount) >= state -> CabTimes.MFOR.DRD_MFOR_CAB_MaxOff )
      {
        state -> last_cab_off_time = state -> CabTimes.MFOR.DRD_MFOR_CAB_MaxOff;
        state -> last_cab_on_time = 0;
        state -> SampCount = 1;
      }
    }

    state -> LastDemodState = state -> DemodState;

    // Determine the current CAB code state based on the ON/OFF times.

    if ( state -> last_cab_on_time >= state -> CabTimes.MFOR.DRD_MFOR_CAB_MaxOn )
    {
      state -> ScanForPeaks = 1;  // Always scan for peaks with CC.
      state -> DemodState = DRD_MFOR_CAB_STATE_CONSTANT;
    }
    else if ( state -> last_cab_off_time >= state -> CabTimes.MFOR.DRD_MFOR_CAB_MaxOff )
    {
      // Only clear buffers/states on the transition to NOCARRIER from another state.

      if ( state -> DemodState != DRD_MFOR_CAB_STATE_NOCARRIER )
      {
        state -> DemodState = DRD_MFOR_CAB_STATE_NOCARRIER;
      }
    }
    else
    {
      if ( state -> check_period )
      {
        state -> check_period = 0;

        state -> Period = state -> last_cab_on_time + state -> last_cab_off_time;

        if ( state -> Period > state -> CabTimes.MFOR.DRD_MFOR_80_MinPer && state -> Period < state -> CabTimes.MFOR.DRD_MFOR_80_MaxPer )
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_80;
        }
        else if ( state -> Period > state -> CabTimes.MFOR.DRD_MFOR_70_MinPer && state -> Period < state -> CabTimes.MFOR.DRD_MFOR_70_MaxPer )
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_70;
        }
        else if ( state -> Period > state -> CabTimes.MFOR.DRD_MFOR_60_MinPer && state -> Period < state -> CabTimes.MFOR.DRD_MFOR_60_MaxPer)
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_60;
        }
        else if ( state -> Period > state -> CabTimes.MFOR.DRD_MFOR_50_MinPer && state -> Period < state -> CabTimes.MFOR.DRD_MFOR_50_MaxPer )
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_50;
        }
        else if ( state -> Period > state -> CabTimes.MFOR.DRD_MFOR_40_MinPer && state -> Period < state -> CabTimes.MFOR.DRD_MFOR_40_MaxPer )
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_40;
        }
        else if ( state -> Period > state ->CabTimes.MFOR.DRD_MFOR_25_MinPer && state -> Period < state -> CabTimes.MFOR.DRD_MFOR_25_MaxPer )
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_25;
        }
        else if ( state -> Period > state -> CabTimes.MFOR.DRD_MFOR_10_MinPer && state -> Period < state -> CabTimes.MFOR.DRD_MFOR_10_MaxPer )
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_10;
        }
        else if ( state -> Period > state -> CabTimes.MFOR.DRD_MFOR_0_MinPer && state -> Period < state -> CabTimes.MFOR.DRD_MFOR_0_MaxPer )
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_0;
        }
        else 
        {
          state -> DemodState = DRD_MFOR_CAB_STATE_UNKNOWN;
          state -> ScanForPeaks = 0;         // stop scanning for peaks until a known modulation is present
        }
      }
    }
  }

  state -> PeakAvg = sqrtf( state -> PeakSquaredAvg );
} // end DRD_MFOR_CABDecoder()

/*******************************************************************************
* Function: DRD_STL_CABDecoder
*
* Summary:  Processes the demod data to extract CAB codes.
*
* Details: 
*
* For 2340 Hz carrier: every 42.7 uS (23.4KHz Hz) Decode Cab Signalling
*
* STL Cab signalling consists of a carrier at 2340Hz and 
* modulation is just on/off keying at a particular rate.
* The algorithm used in a real cab will be closely mirrored here.
* The signal is sampled at 10 times the carrier (23.4 KHz for 2340 Hz).
* It is rectified and boxcar averaged over 20 samples.  This will result
* is a psuedo squarewave of the modulation signal with a small amount of
* ripple in the on times.  By setting a threshold, the on and off can be
* easily detected and the possible codes can be decoded.
*
* A state machine is used to track the cab state and decode the modulation.
*
*******************************************************************************/
void DRD_STL_CABDecoder( struct _drd_state_ *state, int16_t *data, uint16_t size )
{
  int j;
  float32_t sineTemp, cosineTemp, temp;


  for( j = 0;j < BLOCK_SIZE;m++, j++ )
  {
    if ( m >= SAMPLE_FREQUENCY )
    {
      m = 0;

      state -> currentSineF32 = 0.0;
      state -> currentCosineF32 = 1.0;
    }

    SineSamples[j] = data[j]*state -> currentSineF32;

    CosineSamples[j] = data[j]*state -> currentCosineF32;

    sineTemp = state -> currentSineF32*state -> cosinef32 + state -> currentCosineF32*state -> sinef32;

    cosineTemp = state -> currentCosineF32*state -> cosinef32 - state -> currentSineF32*state -> sinef32;

    state -> currentSineF32 = sineTemp;

    state -> currentCosineF32 = cosineTemp;
  }

  arm_fir_decimate_f32( &FirstSineInstance, SineSamples, FirstInstanceSineOutputs, BLOCK_SIZE );
  arm_fir_decimate_f32( &FirstCosineInstance, CosineSamples, FirstInstanceCosineOutputs, BLOCK_SIZE );

  arm_fir_decimate_f32( &SecondSineInstance, FirstInstanceSineOutputs, SecondInstanceSineOutputs, BLOCK_SIZE/8 );
  arm_fir_decimate_f32( &SecondCosineInstance, FirstInstanceCosineOutputs, SecondInstanceCosineOutputs, BLOCK_SIZE/8 );

  for( j = 0;j < BLOCK_SIZE/64;j++ )
  {
    state -> SampCount += 1;

    state -> DemodAvg = SecondInstanceSineOutputs[j]*SecondInstanceSineOutputs[j] + SecondInstanceCosineOutputs[j]*SecondInstanceCosineOutputs[j];

    if ( state -> DemodAvg > .998*state -> PeakDemodAvg ) 
    {
      state -> PeakDemodAvg = state -> DemodAvg;
    }

    temp = state -> PeakSquaredAvg1[0];

    state -> PeakSquaredAvg1[0] = FILTER_COEFF_1*state -> PeakSquaredAvg1[0] - FILTER_COEFF_2*state -> PeakSquaredAvg1[1] + (1.0 - LOW_PASS_GAIN)*(1.0 - LOW_PASS_GAIN)*state -> PeakDemodAvg;
    
    state -> PeakSquaredAvg1[1] = temp;

    temp = state -> PeakSquaredAvg;

    state -> PeakSquaredAvg = FILTER_COEFF_1*state -> PeakSquaredAvg - FILTER_COEFF_2*state -> PeakSquaredAvg1[2] + (1.0 - LOW_PASS_GAIN)*(1.0 - LOW_PASS_GAIN)*state -> PeakSquaredAvg1[0];
    
    state -> PeakSquaredAvg1[2] = temp;

    state -> PeakDemodAvg *= .999;

    // IF current signal sample is "ON".

    if ( state -> DemodAvg >= state -> DemodThreshold )
    {
      // Handle change from signal "OFF" to "ON".

      if ( state -> signal_state != 1 )
      {
        state -> signal_state = 1;
        state -> check_period = 1;
        state -> last_cab_off_time = 64*(state -> SampCount - 1);
        state -> SampCount = 1;
      }

      // Check for signal that has been "ON" greater than 555ms.

      if ( 64*(state -> SampCount) >= state -> CabTimes.STL.DRD_STL_CAB_MaxOn )
      {
        state -> last_cab_on_time = state -> CabTimes.STL.DRD_STL_CAB_MaxOn;
        state -> last_cab_off_time = 0;
        state -> SampCount = 1;
      }
    }
    else if ( state -> DemodAvg < state -> DemodThresholdHyst )
    {
      // Handle change from signal "ON" to "OFF".

      if ( state -> signal_state != 0 )
      {
        state -> signal_state = 0;
        state -> last_cab_on_time = 64*(state -> SampCount - 1);
        state -> SampCount = 1;
      }

      // Set state to No Carrier if signal has been "OFF" greater than 2368ms.

      if ( 64*(state -> SampCount) >= state -> CabTimes.STL.DRD_STL_CAB_MaxOff )
      {
        state -> last_cab_off_time = state -> CabTimes.STL.DRD_STL_CAB_MaxOff;
        state -> last_cab_on_time = 0;
        state -> SampCount = 1;
      }
    }

    state -> LastDemodState = state -> DemodState;

    // Determine the current CAB code state based on the ON/OFF times.

    if ( state -> last_cab_on_time >= state -> CabTimes.STL.DRD_STL_CAB_MaxOn )
    {
      state -> ScanForPeaks = 1;  // Always scan for peaks with CC.
      state -> DemodState = DRD_STL_CAB_STATE_CONSTANT;
    }
    else if ( state -> last_cab_off_time >= state -> CabTimes.STL.DRD_STL_CAB_MaxOff )
    {
      // Only clear buffers/states on the transition to NOCARRIER from another state.

      if ( state -> DemodState != DRD_STL_CAB_STATE_NOCARRIER )
      {
        state -> DemodState = DRD_STL_CAB_STATE_NOCARRIER;
      }
    }
    else
    {
      if ( state -> check_period )
      {
        state -> check_period = 0;

        state -> Period = state -> last_cab_on_time + state -> last_cab_off_time;

        if ( state -> Period > state -> CabTimes.STL.DRD_STL_55_MinPer && state -> Period < state -> CabTimes.STL.DRD_STL_55_MaxPer )
        {
          state -> DemodState = DRD_STL_CAB_STATE_55;
        }
        else if ( state -> Period > state -> CabTimes.STL.DRD_STL_45_MinPer && state -> Period < state -> CabTimes.STL.DRD_STL_45_MaxPer )
        {
          state -> DemodState = DRD_STL_CAB_STATE_45;
        }
        else if ( state -> Period > state -> CabTimes.STL.DRD_STL_STREET_MinPer && state -> Period < state -> CabTimes.STL.DRD_STL_STREET_MaxPer)
        {
          state -> DemodState = DRD_STL_CAB_STATE_STREET;
        }
        else if ( state -> Period > state -> CabTimes.STL.DRD_STL_35_MinPer && state -> Period < state -> CabTimes.STL.DRD_STL_35_MaxPer )
        {
          state -> DemodState = DRD_STL_CAB_STATE_35;
        }
        else if ( state -> Period > state -> CabTimes.STL.DRD_STL_25_MinPer && state -> Period < state -> CabTimes.STL.DRD_STL_25_MaxPer )
        {
          state -> DemodState = DRD_STL_CAB_STATE_25;
        }
        else if ( state -> Period > state ->CabTimes.STL.DRD_STL_YARD_MinPer && state -> Period < state -> CabTimes.STL.DRD_STL_YARD_MaxPer )
        {
          state -> DemodState = DRD_STL_CAB_STATE_YARD;
        }
        else if ( state -> Period > state -> CabTimes.STL.DRD_STL_15_MinPer && state -> Period < state -> CabTimes.STL.DRD_STL_15_MaxPer )
        {
          state -> DemodState = DRD_STL_CAB_STATE_15;
        }
        else if ( state -> Period > state -> CabTimes.STL.DRD_STL_5_MinPer && state -> Period < state -> CabTimes.STL.DRD_STL_5_MaxPer )
        {
          state -> DemodState = DRD_STL_CAB_STATE_5;
        }
        else 
        {
          state -> DemodState = DRD_STL_CAB_STATE_UNKNOWN;
          state -> ScanForPeaks = 0;         // stop scanning for peaks until a known modulation is present
        }
      }
    }
  }

  state -> PeakAvg = sqrtf( state -> PeakSquaredAvg );
} // end DRD_STL_CABDecoder()

static void TraceState( int location, int state )
{
  if ( traceCount < 32 )
  {
    traceArray[traceCount++] = (location << 8) | state;
  }
  else
  {
    traceCount = 0;

    traceArray[traceCount++] = (location << 8) | state;
  }

  return;
}

/*******************************************************************************
* Function: DRD_RET_CabDecoder
*
* Summary:  Processes the demod data to extract RET speed codes.
*
*******************************************************************************/

void DRD_RET_CABDecoder( struct _drd_state_ *state, int16_t *data, uint16_t size )
{
  uint16_t i, j;
  int Voltage1CountOut, Voltage2CountOut, Magnitude1CountOut, Magnitude2CountOut;
  float32_t Voltage1MagSquared, Voltage2MagSquared, FilteredVoltage1MagSquared, FilteredVoltage2MagSquared;
  
  if ( state -> Mode == MODE_MEAS )
  {
    state -> PresFreqLock = 0;
  }
  else
  {
    state -> PresFreqLock = 1;
  }

  for( j = 0;j < size;j++ )
  {
    SineSamples[j] = (float32_t) data[j];
  }

  arm_fir_decimate_f32( &RETInstance, SineSamples, FirstInstanceSineOutputs, size );

  SingleLineDFTFloat( &Voltage1MatchedFilter, FirstInstanceSineOutputs, Voltage1RealOutputs, Voltage1ImagOutputs, size/8, &Voltage1CountOut );

  SingleLineDFTFloat( &Voltage2MatchedFilter, FirstInstanceSineOutputs, Voltage2RealOutputs, Voltage2ImagOutputs, size/8, &Voltage2CountOut );

  if ( (Voltage1CountOut >= (size/8)) && (Voltage2CountOut >= (size/8)) )
  {
    for( i = 0;i < (size/8);i++ )
    {
      Voltage1MagSquared = Voltage1RealOutputs[i]*Voltage1RealOutputs[i] + Voltage1ImagOutputs[i]*Voltage1ImagOutputs[i];

      Voltage2MagSquared = Voltage2RealOutputs[i]*Voltage2RealOutputs[i] + Voltage2ImagOutputs[i]*Voltage2ImagOutputs[i];

      state -> PeakDemodAvg = Voltage1MagSquared*RET_CAB_CONSTANT;

      state -> PeakSquaredAvg = .5*state -> PeakSquaredAvg + .5*state -> PeakDemodAvg;

      state -> SampCount += 1;

      if ( Voltage1MagSquared >= state -> DemodThreshold )
      {
        state -> PresFreqIdx = (state -> PresFreqIdx + scanFreqIdx) % state -> NumFreqs;

        scanFreqIdx = 0;

        if ( state -> DemodState == DRD_RET_CAB_STATE_NOCARRIER )
        {
          TraceState( 1, DRD_RET_CAB_STATE_UNKNOWN );

          state -> DemodState = DRD_RET_CAB_STATE_UNKNOWN;
	}

        if ( (Voltage2MagSquared >= state -> DemodThreshold) && (Voltage2MagSquared >= .2*Voltage1MagSquared) )
        {
          int retIndex = 0;

#ifdef _CHANGE_
	  if ( !scanExtraCount && !scanFreq1Idx )
	  {
            state -> DemodState = DRD_RET_CAB_STATE_UNKNOWN;
	  }
#endif

          if ( state -> DemodState == DRD_RET_CAB_STATE_UNKNOWN )
          {
            state -> PresFreq1Idx = (state -> PresFreq1Idx + scanFreq1Idx) % state -> NumFreqs;

            scanFreq1Idx = 0;
          
            if ( state -> PresFreqIdx < state -> PresFreq1Idx )
            {
              retIndex = (state -> NumFreqs - 1)*state -> PresFreqIdx - state -> PresFreqIdx*(state -> PresFreqIdx + 1)/2 + state -> PresFreq1Idx - state -> PresFreqIdx - 1;

              TraceState( 2, RETCodes[retIndex] );

              state -> DemodState = RETCodes[retIndex];
            }
            else
            {
              retIndex = (state -> NumFreqs - 1)*state -> PresFreq1Idx - state -> PresFreq1Idx*(state -> PresFreq1Idx + 1)/2 + state -> PresFreqIdx - state -> PresFreq1Idx - 1;

              TraceState( 3, RETCodes[retIndex] );

              state -> DemodState = RETCodes[retIndex];
            }
          }
          else
          {
            if ( scanFreq1Idx && (state -> PresFreqIdx != state -> PresFreq1Idx) )
            {
              scanExtraCount += 1;

              TraceState( 4, DRD_RET_CAB_STATE_MULTIPLE );

              state -> DemodState = DRD_RET_CAB_STATE_MULTIPLE;
            }
            else
            {
              if ( !scanExtraCount )
              {
                state -> PresFreq1Idx = (state -> PresFreq1Idx + scanFreq1Idx) % state -> NumFreqs;

                scanFreq1Idx = 0;
          
                if ( state -> PresFreqIdx < state -> PresFreq1Idx )
                {
                  retIndex = (state -> NumFreqs - 1)*state -> PresFreqIdx - state -> PresFreqIdx*(state -> PresFreqIdx + 1)/2 + state -> PresFreq1Idx - state -> PresFreqIdx - 1;

                  TraceState( 5, RETCodes[retIndex] );

                  state -> DemodState = RETCodes[retIndex];
                }
                else
                {
                  retIndex = (state -> NumFreqs - 1)*state -> PresFreq1Idx - state -> PresFreq1Idx*(state -> PresFreq1Idx + 1)/2 + state -> PresFreqIdx - state -> PresFreq1Idx - 1;

                  TraceState( 6, RETCodes[retIndex] );

                  state -> DemodState = RETCodes[retIndex];
                }
              }
              else
              {
                scanExtraCount = 0;
              }
            }
          }

          scanFreq1Idx += 1;

          if ( scanFreq1Idx >= state -> NumFreqs )
          {
            scanFreq1Idx = 0;
          }

          if ( ((state -> PresFreq1Idx + scanFreq1Idx) % state -> NumFreqs) == state -> PresFreqIdx )
          {
            scanFreq1Idx += 1;

            if ( scanFreq1Idx >= state -> NumFreqs )
            {
              scanFreq1Idx = 0;
            }
          }

          frequency = state -> aDispFreqTab[((state -> PresFreq1Idx + scanFreq1Idx) % state -> NumFreqs)];

          InitializeDFT( &Voltage2MatchedFilter, frequency, SAMPLE_FREQUENCY/8.0 );

	  break;
        }
        else
        {
          if ( !scanFreq1Idx )
          {
            TraceState( 7, DRD_RET_CAB_STATE_UNKNOWN );

            state -> DemodState = DRD_RET_CAB_STATE_UNKNOWN;
          }

          scanFreq1Idx += 1;

          if ( scanFreq1Idx >= state -> NumFreqs )
          {
            scanFreq1Idx = 0;
          }

          if ( ((state -> PresFreq1Idx + scanFreq1Idx) % state -> NumFreqs) == state -> PresFreqIdx )
          {
            scanFreq1Idx += 1;

            if ( scanFreq1Idx >= state -> NumFreqs )
            {
              scanFreq1Idx = 0;
            }
          }

          frequency = state -> aDispFreqTab[((state -> PresFreq1Idx + scanFreq1Idx) % state -> NumFreqs)];

          InitializeDFT( &Voltage2MatchedFilter, frequency, SAMPLE_FREQUENCY/8.0 );

	  break;
        }
      }
      else
      {
        TraceState( 8, DRD_RET_CAB_STATE_UNKNOWN );

        state -> DemodState = DRD_RET_CAB_STATE_UNKNOWN;

        if ( state -> PresFreqLock )
        {
          scanFreq1Idx = 0;

          scanFreqIdx += 1;

          if ( scanFreqIdx >= state -> NumFreqs )
          {
            scanFreqIdx = 0;
          }

          if ( ((state -> PresFreqIdx + scanFreqIdx) % state -> NumFreqs) == state -> PresFreq1Idx )
          {
            scanFreqIdx += 1;

            if ( scanFreqIdx >= state -> NumFreqs )
            {
              scanFreqIdx = 0;
            }
          }

          frequency = state -> aDispFreqTab[((state -> PresFreqIdx + scanFreqIdx) % state -> NumFreqs)];

          InitializeDFT( &Voltage1MatchedFilter, frequency, SAMPLE_FREQUENCY/8.0 );
        }

	break;
      }
    }
  }

  state -> PeakAvg = sqrtf( state -> PeakSquaredAvg );
} // DRD_RET__Decoder()
