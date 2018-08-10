#include "sensorhub.h"
#include "drdstate.h"

int SensorDetect(void);

extern PERS_TAB_T DRDPersTable;

void PowerDown( void )
{
  Chip_GPIO_SetPinState( LPC_GPIO, 0,  7, true );
  
  while(1);
}

////////////////////////////////////////////////////////////////////////////////
// Function: ReadShuntType
//
// Summary:  Reads the ADC channel 2 to determine what shunt type is present
//           in dual shunt DRD types.
//
// Returns:
// 0 = no shunt present
// 1 = shunt for sig type 1
// 2 = shunt for sig type 2
//
// NOTE: You cannot do this while timer 0 interrupt is running!!!
////////////////////////////////////////////////////////////////////////////////

int ReadShuntType(void)
{
    int sensDet = -1;
    float fval32 = GetADCVoltage( ADC_SHUNTMONV );

    // Determine if the ADC value matches a known sensor detection scheme
    if ( fval32 <= .0032f )
    {
        sensDet = SENSDET_SHORT;
    }
    else if ( ( fval32 >= .0132f ) && ( fval32 <= .0162f ) )
    {
        sensDet = SENSDET_200OHM;
    }
    else if ( ( fval32 >= .0319f ) && ( fval32 <= .0390f ) )
    {
        sensDet = SENSDET_500OHM;
    }

    return sensDet;
}

////////////////////////////////////////////////////////////////////////////////
// Function: CheckSensorMatch
//
// Summary:  Reads the ADC channel 2 to determine what shunt type is present
//           in dual shunt DRD types.
//
// Returns:
// 0 = no sensor present
// 1 = sensor detection matches that for present gSigType
//
// NOTE: You cannot do this while timer 0 interrupt is running!!!
////////////////////////////////////////////////////////////////////////////////

int CheckSensorMatch( struct _drd_state_ *state )
{
  int sensdet = SENSTYPE_NONE;
  float fval32;

  if ( SensorDetect() == 0 )
  {
    return SENSTYPE_NONE;
  }
    
  fval32 = GetADCVoltage( ADC_SHUNTMONV );

  // Determine if the ADC value matches a known sensor detection scheme
  if ( fval32 <= 32.0f )
  {
    sensdet = SENSDET_SHORT;
  }
  else if ( ( fval32 >= 132.0f ) && ( fval32 <= 162.0f ) )
  {
    sensdet = SENSDET_200OHM;
  }
  else if ( ( fval32 >= 319.0f ) && ( fval32 <= 390.0f ) )
  {
    sensdet = SENSDET_500OHM;
  }

  // See if the analog sensor detection matches that for this gSigType
  if ( DRDPersTable.SigType1 == state -> SigType )
  {
    if ( sensdet & DRDPersTable.SensorType1 )
    {
      return 1;
    }
  }
  else if ( DRDPersTable.SigType2 == state -> SigType )
  {
    if ( sensdet & DRDPersTable.SensorType2 )
    {
      return 1;
    }
  }
  else if ( DRDPersTable.SigType3 == state -> SigType )
  {
    if ( sensdet & DRDPersTable.SensorType3 )
    {
      return 1;
    }
  }

  return SENSTYPE_NONE;
} // end CheckSensorMatch()

int SensorDetect(void)
{
    float fval32 = GetADCVoltage( ADC_SHUNTMONV );

    if ( fval32 < 1.25 )
    {
        return 1;
    }
    
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// The backlight is PWMed partly in HW and partly in SW.
// It can be set fro 0 to 100% in steps of 10%.
// The PWM rate is 150 hz.
// DMA Timer 2 output is used in toggle mode to drive the backlight.
// At every interrupt the backlight state will toggle and the counter
// reference (reload) value must be loaded with the correct value
// to achieve the desired duty cycle.
// At duty cycles of 0 and 100 the timer is not used and the output is hard set
// to either 0 or 1.
////////////////////////////////////////////////////////////////////////////////

void Backlight_SetDuty(int Duty)
{
  uint32_t      totcounts;

  if ( Duty <= 0 )
  {
    Chip_TIMER_SetMatch( LPC_TIMER3, 0, 640000 );
  }
  else if ( Duty >= 100 )
  {
    Chip_TIMER_SetMatch( LPC_TIMER3, 0, 0 );
  }
  else
  {
    totcounts = 639999;     // counts for 150 hz

    if ( Duty <= 10 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*90/100 );
    }
    else if ( Duty <= 20 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*80/100 );
    }
    else if ( Duty <= 30 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*70/100 );
    }
    else if ( Duty <= 40 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*60/100 );
    }
    else if ( Duty <= 50 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*50/100 );
    }
    else if ( Duty <= 60 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*40/100 );
    }
    else if ( Duty <= 70 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*30/100 );
    }
    else if ( Duty <= 80 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*20/100 );
    }
    else if ( Duty <= 100 )
    {
      Chip_TIMER_SetMatch( LPC_TIMER3, 0, totcounts*10/100 );
    }
  }
} // end Backlight_SetDuty()

////////////////////////////////////////////////////////////////////////////////
// DMA Timer 1 output drives the beeper at 2048 Hz in toggle mode.
// PIT 0 is used to time the duration.  It interrupts when the beep
// should end.
// +PTIMER2    DT1OUT     DACK1      -          O     BUZZER
////////////////////////////////////////////////////////////////////////////////

void Beeper(int onoff)
{
  if ( onoff == 0 )
  {
    Chip_TIMER_Disable( LPC_TIMER2 );
  }
  else
  {
    Chip_TIMER_Enable( LPC_TIMER2 );
  }
}

void HeartbeatLED( uint32_t OnOff )
{
  if ( OnOff )
  {
    Chip_GPIO_SetPinState( LPC_GPIO, 0, 2, true );
  }
  else
  {
    Chip_GPIO_SetPinState( LPC_GPIO, 0, 2, false );
  }
}
