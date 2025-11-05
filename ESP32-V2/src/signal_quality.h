#ifndef SIGNAL_QUALITY_H
#define SIGNAL_QUALITY_H

#include <Arduino.h>
#include "calibration.h"
#include "beat_detection.h"
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

struct SignalQualityState
{
    bool qualityGood;
    unsigned long lastQualityCheck;
};

// Check signal quality
void checkQuality(hd44780_I2Cexp *lcd, SignalQualityState *state, CalibrationData *calibData,
                  BeatDetectionState *beatState, int currentSignal,
                  unsigned long currentTime);

#endif