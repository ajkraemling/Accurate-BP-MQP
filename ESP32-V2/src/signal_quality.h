#ifndef SIGNAL_QUALITY_H
#define SIGNAL_QUALITY_H

#include <Arduino.h>
#include "calibration.h"
#include "beat_detection.h"

typedef struct
{
    bool qualityGood;
    unsigned long lastQualityCheck;
} SignalQualityState;

// Check signal quality
void checkQuality(SignalQualityState *state, CalibrationData *calibData,
                  BeatDetectionState *beatState, int currentSignal,
                  unsigned long currentTime);

#endif