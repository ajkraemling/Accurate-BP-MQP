#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

class Adafruit_MPRLS;
struct BeatDetectionState;

struct CalibrationData
{
    int minSignal;
    int maxSignal;
    int baselineAverage;
    float atmPressure;
};

// Perform calibration routine
void performCalibration(CalibrationData *calibData, BeatDetectionState *beatState,
                        Adafruit_MPRLS *mpr, hd44780_I2Cexp *lcd);

// Update adaptive thresholds based on current signal
void updateThresholds(CalibrationData *calibData, BeatDetectionState *beatState,
                      int currentSignal, unsigned long currentTime);

// Recalibrate signal range (for when signal characteristics change dramatically)
void recalibrateSignalRange(CalibrationData *calibData, BeatDetectionState *beatState);

#endif
