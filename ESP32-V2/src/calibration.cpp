#include "calibration.h"
#include "config.h"
#include "Adafruit_MPRLS.h"

void performCalibration(CalibrationData *calibData, BeatDetectionState *beatState,
                        Adafruit_MPRLS &mpr)
{
    Serial.println("\n=== CALIBRATION ===");
    Serial.print("  Place finger on sensor...\n   ");

    // Countdown
    for (int i = CALIBRATION_COUNTDOWN_SEC; i > 0; i--)
    {
        Serial.print(i);
        Serial.print("... ");
        delay(1000);
    }
    Serial.println("\nCalibrating...");

    // Initialize calibration values
    calibData->minSignal = 4095;
    calibData->maxSignal = 0;
    calibData->atmPressure = 0;
    long sumSignal = 0;
    int sampleCount = 0;

    unsigned long calibrationStart = millis();

    while (millis() - calibrationStart < CALIBRATION_DURATION_MS)
    {
        // Read PPG signal
        int signal = analogRead(PULSESENSOR_OUT);

        // Update min/max
        if (signal > calibData->maxSignal)
        {
            calibData->maxSignal = signal;
        }
        if (signal < calibData->minSignal && signal > 10)
        {
            calibData->minSignal = signal;
        }
        sumSignal += signal;

        // Accumulate pressure readings
        calibData->atmPressure += mpr.readPressure();
        sampleCount++;

        // Progress indicator
        if (sampleCount % 100 == 0)
        {
            Serial.print(".");
        }

        delay(20);
    }

    // Calculate baseline values
    int range = calibData->maxSignal - calibData->minSignal;
    calibData->baselineAverage = sumSignal / sampleCount;
    calibData->atmPressure = calibData->atmPressure / sampleCount;

    // Calculate initial thresholds
    calculateThresholds(beatState, calibData->minSignal, calibData->maxSignal);

    // Print calibration results
    Serial.println("\nCalibration complete!");
    Serial.println("   Signal Range: " + String(calibData->minSignal) + " - " +
                   String(calibData->maxSignal) + " (Range: " + String(range) + ")");
    Serial.println("   Average Signal: " + String(calibData->baselineAverage));
    Serial.println("   Upper Threshold: " + String(beatState->upperThreshold));
    Serial.println("   Lower Threshold: " + String(beatState->lowerThreshold));
    Serial.println("   Baseline Pressure: " + String(calibData->atmPressure, 1) + " hPa");

    // Signal quality warnings
    if (range < MIN_SIGNAL_RANGE)
    {
        Serial.println("\nWARNING: Weak pulse signal detected!");
        Serial.println("PPG sensor may be too loose or poorly positioned.");
    }
    else if (range > MAX_SIGNAL_RANGE)
    {
        Serial.println("\nWARNING: Signal may be saturated!");
        Serial.println("PPG sensor may be too tight or too much light may be getting in");
    }
}

void updateThresholds(CalibrationData *calibData, BeatDetectionState *beatState,
                      int currentSignal, unsigned long currentTime)
{
    if (currentTime - beatState->lastThresholdUpdate < THRESHOLD_UPDATE_INTERVAL_MS)
    {
        return;
    }

    bool thresholdsChanged = false;

    // Gradually update max if seeing high signals
    if (currentSignal > calibData->maxSignal - 100)
    {
        int oldMax = calibData->maxSignal;
        calibData->maxSignal = calibData->maxSignal * (1 - ADAPTIVE_ALPHA) +
                               currentSignal * ADAPTIVE_ALPHA;
        if (abs(oldMax - calibData->maxSignal) > 10)
        {
            thresholdsChanged = true;
        }
    }

    // Gradually update min if seeing low signals (but not noise)
    if (currentSignal < calibData->minSignal + 100 && currentSignal > 50)
    {
        int oldMin = calibData->minSignal;
        calibData->minSignal = calibData->minSignal * (1 - ADAPTIVE_ALPHA) +
                               currentSignal * ADAPTIVE_ALPHA;
        if (abs(oldMin - calibData->minSignal) > 10)
        {
            thresholdsChanged = true;
        }
    }

    // Recalculate thresholds if changed
    if (thresholdsChanged)
    {
        calculateThresholds(beatState, calibData->minSignal, calibData->maxSignal);

        if (DEBUG_MODE)
        {
            int range = calibData->maxSignal - calibData->minSignal;
            Serial.println("\n[Thresholds adapted: Upper=" + String(beatState->upperThreshold) +
                           " Lower=" + String(beatState->lowerThreshold) +
                           " Range=" + String(range) + "]");
        }
    }

    beatState->lastThresholdUpdate = currentTime;
}