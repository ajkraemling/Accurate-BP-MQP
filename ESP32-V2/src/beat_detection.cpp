#include "beat_detection.h"
#include "calibration.h"
#include "config.h"

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
    bpData->state = BP_IDLE;
    bpData->systolicPressure = 0;
    bpData->diastolicPressure = 0;
    bpData->maxPressureSeen = 0;
    bpData->systolicDetected = false;
    bpData->diastolicDetected = false;
    bpData->rangeRecalibrated = false;
    bpData->measurementStartTime = 0;
}

void updateBPMeasurement(BPMeasurementData *bpData, float currentPressure, bool heartbeatOccurred)
{
    // Track maximum pressure seen
    if (currentPressure > bpData->maxPressureSeen)
    {
        bpData->maxPressureSeen = currentPressure;
    }

    switch (bpData->state)
    {
    case BP_IDLE:
        // Start looking when pressure starts rising
        if (currentPressure > SYSTOLIC_MIN_PRESSURE)
        {
            bpData->state = BP_WAITING_INFLATE;
            Serial.println("\n[BP] Pressure detected, waiting for inflation...");
        }
        break;

    case BP_WAITING_INFLATE:
        // Wait until pressure reaches measurement threshold
        if (currentPressure >= SYSTOLIC_START_PRESSURE)
        {
            bpData->state = BP_READY;
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
            Serial.println("\n[BP] Pressure dropped before measurement - resetting");
            initializeBPMeasurement(bpData);
        }
        break;

    case BP_READY:
        // Wait for pressure to start dropping, then recalibrate to flatline conditions
        if (currentPressure < (bpData->maxPressureSeen - PRESSURE_DROP_THRESHOLD))
        {
            // Recalibrate once when we start deflating
            if (!bpData->rangeRecalibrated)
            {
                extern CalibrationData calibData;
                extern BeatDetectionState beatState;
                recalibrateSignalRange(&calibData, &beatState);
                bpData->rangeRecalibrated = true;
            }

            // Now detect first heartbeat = systolic
            if (heartbeatOccurred)
            {
                bpData->systolicPressure = currentPressure;
                bpData->systolicDetected = true;
                bpData->state = BP_MEASURING;
                Serial.println("\n*** SYSTOLIC: " + String((int)bpData->systolicPressure) + " mmHg ***");
                Serial.println("[BP] Detected first weak pulse during deflation");
            }
        }
        // Timeout if no drop detected
        else if (millis() - bpData->measurementStartTime > 30000)
        {
            Serial.println("\n[BP] Timeout waiting for pressure drop - resetting");
            initializeBPMeasurement(bpData);
        }
        break;

    case BP_MEASURING:
        // Continue recording heartbeats, last one before pressure gets too low is diastolic
        if (heartbeatOccurred && currentPressure > DIASTOLIC_MIN_PRESSURE)
        {
            bpData->diastolicPressure = currentPressure;
            Serial.println("[BP] Heartbeat at " + String((int)currentPressure) + " mmHg (pulses getting stronger)");
        }

        // Complete when pressure drops below minimum
        if (currentPressure < DIASTOLIC_MIN_PRESSURE)
        {
            bpData->diastolicDetected = true;
            bpData->state = BP_COMPLETE;
            Serial.println("\n*** DIASTOLIC: " + String((int)bpData->diastolicPressure) + " mmHg ***");
            Serial.println("========================================");
            Serial.println("BLOOD PRESSURE: " + String((int)bpData->systolicPressure) + "/" +
                           String((int)bpData->diastolicPressure) + " mmHg");
            Serial.println("========================================\n");
        }

        // Timeout
        if (millis() - bpData->measurementStartTime > 60000)
        {
            Serial.println("\n[BP] Measurement timeout - resetting");
            initializeBPMeasurement(bpData);
        }
        break;

    case BP_COMPLETE:
        // Reset after a few seconds or when pressure drops to near zero
        if (currentPressure < 10)
        {
            Serial.println("[BP] Measurement complete, ready for next reading\n");
            initializeBPMeasurement(bpData);
        }
        break;
    }
}

bool isReadyForMeasurement(BPMeasurementData *bpData)
{
    return bpData->state == BP_READY || bpData->state == BP_MEASURING;
}

void resetBPMeasurement(BPMeasurementData *bpData)
{
    initializeBPMeasurement(bpData);
    Serial.println("[BP] Measurement reset\n");
}
