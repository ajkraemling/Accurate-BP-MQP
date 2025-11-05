#ifndef BEAT_DETECTION_H
#define BEAT_DETECTION_H

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "config.h"

struct CalibrationData;

struct BeatDetectionState
{
    int upperThreshold;
    int lowerThreshold;
    bool beatDetected;
    unsigned long lastBeatTime;
    unsigned long lastThresholdUpdate;
    int recentBeats[5];
    int beatIndex;
    // For BP measurement mode
    int normalUpperThreshold;
    int normalLowerThreshold;
    bool bpMeasurementMode;
};

// Blood pressure measurement states
enum BPMeasurementState
{
    IDLE,              // Not measuring
    INFLATING,         // Waiting for pressure to reach threshold
    MEASURE_SYSTOLIC,  // Above threshold, ready to detect systolic
    MEASURE_DIASTOLIC, // Found systolic, measuring diastolic
    COMPLETE           // Measurement complete
};

struct BPMeasurementData
{
    BPMeasurementState state;
    BPMeasurementState oldState;
    float systolicPressure;
    float diastolicPressure;
    float maxPressureSeen;
    bool systolicDetected;
    bool diastolicDetected;
    bool rangeRecalibrated;
    unsigned long measurementStartTime;
    // Rolling baseline for flatline detection
    int baselineWindow[BP_BASELINE_WINDOW];
    int baselineIndex;
    int baselineCount;
    long baselineSum;
    int baselineMin;
    int baselineMax;
    int consecutiveAboveThreshold;
};

// Initialize BP measurement data
void initializeBPMeasurement(BPMeasurementData *bpData);

// Update BP measurement state machine
void updateBPMeasurement(BPMeasurementData *bpData, float currentPressure, int currentPPGSignal, hd44780_I2Cexp *lcd);

// Update rolling baseline and check for pulse detection
bool checkForPulseAboveBaseline(BPMeasurementData *bpData, int currentSignal);

// Check if ready to start measurement
bool isReadyForMeasurement(BPMeasurementData *bpData);

// Reset measurement for new reading
void resetBPMeasurement(BPMeasurementData *bpData);

// Initialize beat detection state
void initializeBeatDetection(BeatDetectionState *state);

// Calculate thresholds based on signal range
void calculateThresholds(BeatDetectionState *state, int minSignal, int maxSignal);

// Detect heartbeat using threshold crossing with hysteresis
bool detectHeartbeat(BeatDetectionState *state, int signal, unsigned long currentTime);

// Get average heart rate from recent beats
int getAverageHeartRate(BeatDetectionState *state);

// Check if enough beats for quality assessment
bool hasEnoughBeats(BeatDetectionState *state);

// Enable/disable BP measurement mode (uses more sensitive thresholds)
void setBPMeasurementMode(BeatDetectionState *state, bool enabled, int minSignal, int maxSignal);

#endif