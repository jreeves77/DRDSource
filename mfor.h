PERS_TAB_T DRDPersTable = 
{
  DRD_PERSONALITY_DRD_MFOR,           // Personality ID
  "DRD-MFOR",                         // Title
  "LO BAT ",                          // Message to display for low battery
  "AMP",                              // MEAS units for sig type 1
  "AMP",                              // MEAS units for sig type 2
  "",                                 // MEAS units for sig type 3
  250,                                // msec delay when changing freq during freq cal for sig type 1
  250,                                // msec delay when changing freq during freq cal for sig type 2
  0,                                  // msec delay when changing freq during freq cal for sig type 3
  10,                                 // step size for cal freq tuning in tenths of hz for sig type 1
  10,                                 // step size for cal freq tuning in tenths of hz for sig type 2
  0,                                  // step size for cal freq tuning in tenths of hz for sig type 3
  MODE_MEAS,                          // Initial Mode
  SIGTYPE_DRD_MFOR_CAB,              // Initial Signalling Type
  0,                                  // Initial index into freq table
  SIGTYPE_DRD_MFOR_CAB,               // Signalling Type 1
  SIGTYPE_DRD_MFOR_AFTC,              // Signalling Type 2
  SIGTYPE_NONE,                       // Signalling Type 3
  SENSTYPE_SHUNT | SENSDET_500OHM,    // Sensor type for Sig mode 1 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  SENSTYPE_SHUNT | SENSDET_500OHM,    // Sensor type for Sig mode 2 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  0,                                  // Sensor type for Sig mode 3 (bit encoded as type is bits 7:0 and detection as bits 10:8)
  1,                                  // HW gain setting (0 or 1) for sig type 1
  1,                                  // HW gain setting (0 or 1) for sig type 2
  0,                                  // HW gain setting (0 or 1) for sig type 3
  1,                                  // anti-alias filter selection for sig type 1
  1,                                  // anti-alias filter selection for sig type 2
  0,                                  // anti-alias filter selection for sig type 3
  1,                                  // number of freqs in sig type 1
  8,                                  // number of freqs in sig type 2
  0,                                  // number of freqs in sig type 3
  { // Display Freqs for Sig Type 1
    2340, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
  },
  { // Display Freqs for Sig Type 2
    2970, 3870, 4410, 3510, 4950,
    3330, 4230, 3690, 0, 0,
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
