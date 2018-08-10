#include "projdefs.h"

PERS_TAB_T DRDPersTable = 
{
  DRD_PERSONALITY_BART,               // Personality ID
  "CIRCA",                            // Title
  "LO BAT   ",                        // Message to display for low battery
  "mApp",                             // MEAS units for sig type 1
  "mVrms",                            // MEAS units for sig type 2
  "",                                 // MEAS units for sig type 3
  250,                                // msec delay when changing freq during freq cal for sig type 1
  250,                                // msec delay when changing freq during freq cal for sig type 2
  0,                                  // msec delay when changing freq during freq cal for sig type 3
  10,                                 // step size for cal freq tuning in tenths of hz for sig type 1
  10,                                 // step size for cal freq tuning in tenths of hz for sig type 2
  10,                                 // step size for cal freq tuning in tenths of hz for sig type 3
  MODE_MEAS,                          // Initial Mode
  SIGTYPE_BARTSC,                     // Initial Signalling Type
  4,                                  // Initial index into freq table
  SIGTYPE_BARTSC,                     // Signalling Type 1
  SIGTYPE_BARTMV,                     // Signalling Type 2
  SIGTYPE_NONE,                       // Signalling Type 3
  SENSTYPE_RAILSENS | SENSDET_SHORT,  // Sensor type for Sig mode 1 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  SENSTYPE_DIRECT   | SENSDET_500OHM, // Sensor type for Sig mode 2 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  0,                                  // Sensor type for Sig mode 3 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  0,                                  // HW gain setting (0 or 1) for sig type 1
  1,                                  // HW gain setting (0 or 1) for sig type 2
  0,                                  // HW gain setting (0 or 1) for sig type 3
  1,                                  // anti-alias filter selection for sig type 1
  1,                                  // anti-alias filter selection for sig type 2
  1,                                  // anti-alias filter selection for sig type 3
  12,                                 // number of freqs in sig type 1
  12,                                 // number of freqs in sig type 2
  0,                                  // number of freqs in sig type 3
  { // Display Freqs for Sig Type 1
    2952, 3528, 3888, 4392, 7776, 
    5184, 8763, 5842, 9936, 6624, 
    8400, 5600, 0, 0, 0,
    0, 0, 0, 0, 0
  },
  { // Display Freqs for Sig Type 2
    2952, 3528, 3888, 4392, 7776, 
    5184, 8763, 5842, 9936, 6624, 
    8400, 5600, 0, 0, 0,
    0, 0, 0, 0, 0
  },
  { // Display Freqs for Sig Type 3
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
  }
};
