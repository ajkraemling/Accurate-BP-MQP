#include "signal_quality.h"
#include "config.h"
#include "lcd_display.h"

void checkQuality(hd44780_I2Cexp *lcd, SignalQualityState *state, CalibrationData *calibData,
                  BeatDetectionState *beatState, int currentSignal,
                  unsigned long currentTime)
{
    if (currentTime - state->lastQualityCheck < QUALITY_CHECK_INTERVAL_MS)
    {
        return;
    }

    // Check signal range
    int range = calibData->maxSignal - calibData->minSignal;
    bool rangeOK = (range >= MIN_SIGNAL_RANGE && range <= MAX_SIGNAL_RANGE);

    // Check heart rate
    int heartRate = getAverageHeartRate(beatState);
    bool heartRateOK = hasEnoughBeats(beatState) &&
                       (heartRate >= MIN_HEART_RATE_BPM && heartRate <= MAX_HEART_RATE_BPM);

    // Check signal stability
    bool stabilityOK = abs(currentSignal - calibData->baselineAverage) <
                       (range * STABILITY_THRESHOLD);

    state->qualityGood = rangeOK && heartRateOK && stabilityOK;

    // Update the LCD display to print
    printSignalWarnings(lcd, rangeOK, heartRateOK, stabilityOK);

    if (!state->qualityGood)
    {
        Serial.println("\n*** WARNING: Poor signal quality detected ***");
        if (!rangeOK)
            Serial.println("    - Signal range issue");
        if (!heartRateOK)
            Serial.println("    - Heart rate out of normal range or no beats detected");
        if (!stabilityOK)
            Serial.println("    - Signal unstable (finger may have moved)");
    }

    state->lastQualityCheck = currentTime;
}
