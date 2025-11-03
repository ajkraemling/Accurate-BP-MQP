#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "beat_detection.h"

class Adafruit_MPRLS;

typedef struct
{
    int minSignal;
    int maxSignal;
    int baselineAverage;
    float atmPressure;
} CalibrationData;

// Perform calibration routine
void performCalibration(CalibrationData *calibData, BeatDetectionState *beatState,
                        Adafruit_MPRLS &mpr);

// Update adaptive thresholds based on current signal
void updateThresholds(CalibrationData *calibData, BeatDetectionState *beatState,
                      int currentSignal, unsigned long currentTime);

#endif