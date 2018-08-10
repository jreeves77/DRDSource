#include "projdefs.h"

const PERS_TAB_T DRDPersTable = 
{
  DRD_PERSONALITY_DRD11,              // Personality ID
  "DRD-11",                           // Title
  "LO BAT   ",                        // Message to display for low battery
  "Arms",                             // Amps units for sig type 1
  "",                                 // Amps units for sig type 2
  "",                                 // Amps units for sig type 3
  &CPU_Timer0_ISR_1,                  // DMA Timer 0 ISR for sig type 1 amps mode
  &CPU_Timer0_ISR_1,                  // DMA Timer 0 ISR for sig type 2 amps mode
  &CPU_Timer0_ISR_1,                  // DMA Timer 0 ISR for sig type 3 amps mode
  &CPU_Timer0_ISR_2,                  // DMA Timer 0 ISR for sig type 1 code mode
  &CPU_Timer0_ISR_2,                  // DMA Timer 0 ISR for sig type 2 code mode
  &CPU_Timer0_ISR_2,                  // DMA Timer 0 ISR for sig type 3 code mode
  500,                                // msec delay when changing freq during freq cal for sig type 1
  0,                                  // msec delay when changing freq during freq cal for sig type 2
  0,                                  // msec delay when changing freq during freq cal for sig type 3
  1,                                  // step size for cal freq tuning in tenths of hz for sig type 1
  0,                                  // step size for cal freq tuning in tenths of hz for sig type 2
  0,                                  // step size for cal freq tuning in tenths of hz for sig type 3
  MODE_MEAS,                          // Initial Mode
  SIGTYPE_DRD11_CAB,                  // Initial Signalling Type
  0,                                  // Initial index into freq table
  SIGTYPE_DRD11_CAB,                  // Signalling Type 1
  SIGTYPE_NONE,                       // Signalling Type 2
  SIGTYPE_NONE,                       // Signalling Type 3
  SENSTYPE_RAILSENS | SENSDET_200OHM, // Sensor type for Sig mode 1 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  0,                                  // Sensor type for Sig mode 2 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  0,                                  // Sensor type for Sig mode 3 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  0,                                  // HW gain setting (0 or 1) for sig type 1
  0,                                  // HW gain setting (0 or 1) for sig type 2
  0,                                  // HW gain setting (0 or 1) for sig type 3
  0,                                  // anti-alias filter selection for sig type 1
  0,                                  // anti-alias filter selection for sig type 2
  0,                                  // anti-alias filter selection for sig type 3
  1,                                  // number of freqs in sig type 1
  0,                                  // number of freqs in sig type 2
  0,                                  // number of freqs in sig type 3
  { // Display Freqs for Sig Type 1
    100, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
  },
  { // Display Freqs for Sig Type 2
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
  },
  { // Display Freqs for Sig Type 3
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
  }
};
