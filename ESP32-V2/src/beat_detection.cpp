#include "beat_detection.h"
#include "calibration.h"
#include "config.h"
#include "lcd_display.h"

void initializeBeatDetection(BeatDetectionState *state)
{
    state->upperThreshold = 0;
    state->lowerThreshold = 0;
    state->beatDetected = false;
    state->lastBeatTime = 0;
    state->lastThresholdUpdate = 0;
    state->beatIndex = 0;
    state->normalUpperThreshold = 0;
    state->normalLowerThreshold = 0;
    state->bpMeasurementMode = false;

    for (int i = 0; i < 5; i++)
    {
        state->recentBeats[i] = 0;
    }
}

void calculateThresholds(BeatDetectionState *state, int minSignal, int maxSignal)
{
    int range = maxSignal - minSignal;
    state->upperThreshold = minSignal + (range * UPPER_THRESHOLD_RATIO);
    state->lowerThreshold = minSignal + (range * LOWER_THRESHOLD_RATIO);
}

bool detectHeartbeat(BeatDetectionState *state, int signal, unsigned long currentTime)
{
    bool heartbeatOccurred = false;

    // Detect rising edge (beat start)
    if (signal > state->upperThreshold &&
        !state->beatDetected &&
        (currentTime - state->lastBeatTime > BEAT_WINDOW_MS))
    {

        state->beatDetected = true;
        heartbeatOccurred = true;

        // Store beat interval (skip first beat)
        if (state->lastBeatTime > 0)
        {
            unsigned long interval = currentTime - state->lastBeatTime;
            state->recentBeats[state->beatIndex] = interval;
            state->beatIndex = (state->beatIndex + 1) % 5;
        }

        state->lastBeatTime = currentTime;
    }
    // Detect falling edge (beat end)
    else if (signal < state->lowerThreshold && state->beatDetected)
    {
        state->beatDetected = false;
    }

    return heartbeatOccurred;
}

int getAverageHeartRate(BeatDetectionState *state)
{
    int avgInterval = 0;
    int validBeats = 0;

    for (int i = 0; i < 5; i++)
    {
        if (state->recentBeats[i] > 0)
        {
            avgInterval += state->recentBeats[i];
            validBeats++;
        }
    }

    if (validBeats >= MIN_VALID_BEATS)
    {
        avgInterval /= validBeats;
        return 60000 / avgInterval;
    }

    return -1; // Invalid
}

bool hasEnoughBeats(BeatDetectionState *state)
{
    int validBeats = 0;
    for (int i = 0; i < 5; i++)
    {
        if (state->recentBeats[i] > 0)
            validBeats++;
    }
    return validBeats >= MIN_VALID_BEATS;
}

void setBPMeasurementMode(BeatDetectionState *state, bool enabled, int minSignal, int maxSignal)
{
    if (enabled && !state->bpMeasurementMode)
    {
        // Save normal thresholds
        state->normalUpperThreshold = state->upperThreshold;
        state->normalLowerThreshold = state->lowerThreshold;

        // Set much more sensitive thresholds for weak BP pulses
        int range = maxSignal - minSignal;
        state->upperThreshold = minSignal + (range * BP_MEASUREMENT_SENSITIVITY);
        state->lowerThreshold = minSignal + (range * (BP_MEASUREMENT_SENSITIVITY - 0.05));

        state->bpMeasurementMode = true;

        Serial.println("[BP] Switched to sensitive mode - Upper: " + String(state->upperThreshold) +
                       " Lower: " + String(state->lowerThreshold));
    }
    else if (!enabled && state->bpMeasurementMode)
    {
        // Restore normal thresholds
        state->upperThreshold = state->normalUpperThreshold;
        state->lowerThreshold = state->normalLowerThreshold;
        state->bpMeasurementMode = false;

        Serial.println("[BP] Switched to normal mode - Upper: " + String(state->upperThreshold) +
                       " Lower: " + String(state->lowerThreshold));
    }
}

void initializeBPMeasurement(BPMeasurementData *bpData)
{
    bpData->state = IDLE;
    bpData->oldState = IDLE;
    bpData->systolicPressure = 0;
    bpData->diastolicPressure = 0;
    bpData->maxPressureSeen = 0;
    bpData->systolicDetected = false;
    bpData->diastolicDetected = false;
    bpData->rangeRecalibrated = false;
    bpData->measurementStartTime = 0;
    bpData->baselineIndex = 0;
    bpData->baselineCount = 0;
    bpData->baselineSum = 0;
    bpData->baselineMin = 4095;
    bpData->baselineMax = 0;
    bpData->consecutiveAboveThreshold = 0;

    for (int i = 0; i < BP_BASELINE_WINDOW; i++)
    {
        bpData->baselineWindow[i] = 0;
    }
}

bool checkForPulseAboveBaseline(BPMeasurementData *bpData, int currentSignal)
{
    // Add current signal to rolling window
    if (bpData->baselineCount < BP_BASELINE_WINDOW)
    {
        // Still filling the window
        bpData->baselineWindow[bpData->baselineIndex] = currentSignal;
        bpData->baselineSum += currentSignal;
        bpData->baselineCount++;
    }
    else
    {
        // Window is full, update rolling average
        bpData->baselineSum -= bpData->baselineWindow[bpData->baselineIndex];
        bpData->baselineWindow[bpData->baselineIndex] = currentSignal;
        bpData->baselineSum += currentSignal;
    }

    bpData->baselineIndex = (bpData->baselineIndex + 1) % BP_BASELINE_WINDOW;

    // Calculate statistics from the window
    if (bpData->baselineCount >= BP_BASELINE_WINDOW)
    {
        // Calculate mean
        float baselineAvg = (float)bpData->baselineSum / BP_BASELINE_WINDOW;

        // Calculate standard deviation and calculate min/max for additional context
        float variance = 0;
        bpData->baselineMin = 4095;
        bpData->baselineMax = 0;
        for (int i = 0; i < BP_BASELINE_WINDOW; i++)
        {
            int reading = bpData->baselineWindow[i];
            float diff = reading - baselineAvg;
            variance += diff * diff;
            if (reading < bpData->baselineMin)
                bpData->baselineMin = reading;
            if (reading > bpData->baselineMax)
                bpData->baselineMax = reading;
        }
        variance /= BP_BASELINE_WINDOW;
        float stdDev = sqrt(variance);

        // Threshold based on statistical deviation
        float threshold = baselineAvg + (BP_THRESHOLD_MULTIPLIER * stdDev);

        // Require minimum absolute deviation to avoid false positives on perfectly flat signal
        if (stdDev < MIN_DEVIATION_FROM_FLAT)
            threshold = baselineAvg + MIN_DEVIATION_FROM_FLAT;

        // Check if current signal exceeds threshold
        if (currentSignal > threshold)
        {
            bpData->consecutiveAboveThreshold++;

            // Need 4 consecutive readings above threshold, in case of random noise
            if (bpData->consecutiveAboveThreshold >= 4)
            {
                Serial.println("[BP] Pulse detected! Signal: " + String(currentSignal) +
                               " > Threshold: " + String((int)threshold) +
                               " (Mean: " + String((int)baselineAvg) +
                               " + " + String(BP_THRESHOLD_MULTIPLIER) + "*StdDev: " + String((int)stdDev) + ")");
                Serial.println("[BP]   Baseline range: " + String(bpData->baselineMin) +
                               " - " + String(bpData->baselineMax));
                bpData->consecutiveAboveThreshold = 0; // Reset for next pulse
                return true;
            }
        }
        else
        {
            bpData->consecutiveAboveThreshold = 0;
        }
    }

    return false;
}

void updateBPMeasurement(BPMeasurementData *bpData, float currentPressure, int currentPPGSignal, hd44780_I2Cexp *lcd)
{
    // Track maximum pressure seen
    if (currentPressure > bpData->maxPressureSeen)
    {
        bpData->maxPressureSeen = currentPressure;
    }

    String lcdPrint;
    bool pulseDetected = false;
    switch (bpData->state)
    {
    case IDLE:
        // Start looking when pressure starts rising
        lcdPrint = "Waiting...";
        if (currentPressure > SYSTOLIC_MIN_PRESSURE)
        {
            bpData->state = INFLATING;
            Serial.println("\n[BP] Pressure detected, waiting for inflation...");
        }
        break;

    case INFLATING:
        lcdPrint = "Inflating cuff...\n\nPressure: " + String((int)currentPressure) + " mmHg";
        // Wait until pressure reaches measurement threshold
        if (currentPressure >= SYSTOLIC_START_PRESSURE)
        {
            bpData->state = MEASURE_SYSTOLIC;
            bpData->measurementStartTime = millis();
            Serial.println("\n[BP] Ready to measure - pressure at " + String((int)currentPressure) + " mmHg");
            Serial.println("[BP] Signal is now flatlined from cuff pressure");
            Serial.println("[BP] Waiting for pressure to drop and first heartbeat...");
            Serial.println("[BP] NOTE: Switching to high-sensitivity mode for weak pulses");
        }
        // Reset if pressure drops back down before reaching threshold
        else if (currentPressure < SYSTOLIC_MIN_PRESSURE &&
                 bpData->maxPressureSeen > SYSTOLIC_START_PRESSURE)
        {
            lcdPrint = "Deflated, resetting";
            Serial.println("\n[BP] Pressure dropped before measurement - resetting");
            initializeBPMeasurement(bpData);
        }
        break;

    case MEASURE_SYSTOLIC:
        // Continuously update baseline with flatline signal
        pulseDetected = checkForPulseAboveBaseline(bpData, currentPPGSignal);
        lcdPrint = "Deflating cuff...\n \nPressure: " + String((int)currentPressure) + " mmHg";
        // Wait for pressure to start dropping
        if (currentPressure < (bpData->maxPressureSeen - PRESSURE_DROP_THRESHOLD))
        {
            // Check if we detect a pulse above baseline
            if (pulseDetected)
            {
                bpData->systolicPressure = currentPressure;
                bpData->systolicDetected = true;
                bpData->state = MEASURE_DIASTOLIC;
                Serial.println("\n*** SYSTOLIC: " + String((int)bpData->systolicPressure) + " mmHg ***");
                Serial.println("[BP] First pulse detected above flatline baseline");
                lcdPrint = "SYSTOLIC: " + String((int)bpData->systolicPressure) + " mmHg";
                lcdPrintWithNewlines(lcd, lcdPrint.c_str());
                delay(1000);
            }
        }
        // Timeout if no drop detected
        else if (millis() - bpData->measurementStartTime > 30000)
        {
            Serial.println("\n[BP] Timeout waiting for pressure drop - resetting");
            initializeBPMeasurement(bpData);
        }
        break;

    case MEASURE_DIASTOLIC:
        // Continue recording heartbeats, this is where we would try to get diastolic
        // Skip this for now
        bpData->state = COMPLETE;
        // Timeout
        if (millis() - bpData->measurementStartTime > 60000)
        {
            Serial.println("\n[BP] Measurement timeout - resetting");
            initializeBPMeasurement(bpData);
        }
        break;

    case COMPLETE:
        lcdPrint = "Blood Pressure: \n" + String((int)bpData->systolicPressure) + "/" + String((int)bpData->diastolicPressure) + "mmHg";
        // Reset after a few seconds or when pressure drops to near zero
        if (currentPressure < 10)
        {
            Serial.println("[BP] Measurement complete, ready for next reading\n");
            initializeBPMeasurement(bpData);
        }
        break;
    }

    if (bpData->oldState != bpData->state || bpData->state == MEASURE_SYSTOLIC)
    {
        bpData->oldState = bpData->state;
        lcdPrintWithNewlines(lcd, lcdPrint.c_str());
    }
}

bool isReadyForMeasurement(BPMeasurementData *bpData)
{
    return bpData->state == MEASURE_SYSTOLIC || bpData->state == MEASURE_DIASTOLIC;
}

void resetBPMeasurement(BPMeasurementData *bpData)
{
    initializeBPMeasurement(bpData);
    Serial.println("[BP] Measurement reset\n");
}
