#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

class Adafruit_MPRLS;
struct BeatDetectionState; // forward declare only

struct CalibrationData
{
    int minSignal;
    int maxSignal;
    int baselineAverage;
    float atmPressure;
};

// Perform calibration routine
void performCalibration(CalibrationData *calibData, BeatDetectionState *beatState,
                        Adafruit_MPRLS *mpr);

// Update adaptive thresholds based on current signal
void updateThresholds(CalibrationData *calibData, BeatDetectionState *beatState,
                      int currentSignal, unsigned long currentTime);

// Recalibrate signal range (for when signal characteristics change dramatically)
void recalibrateSignalRange(CalibrationData *calibData, BeatDetectionState *beatState);

#endif
