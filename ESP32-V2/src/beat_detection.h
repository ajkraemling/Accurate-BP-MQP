#ifndef BEAT_DETECTION_H
#define BEAT_DETECTION_H

#include <Arduino.h>

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
typedef enum
{
    BP_IDLE,            // Not measuring
    BP_WAITING_INFLATE, // Waiting for pressure to reach threshold
    BP_READY,           // Above threshold, ready to detect systolic
    BP_MEASURING,       // Found systolic, measuring diastolic
    BP_COMPLETE         // Measurement complete
} BPMeasurementState;

typedef struct
{
    BPMeasurementState state;
    float systolicPressure;
    float diastolicPressure;
    float maxPressureSeen;
    bool systolicDetected;
    bool diastolicDetected;
    bool rangeRecalibrated;
    unsigned long measurementStartTime;
} BPMeasurementData;

// Initialize BP measurement data
void initializeBPMeasurement(BPMeasurementData *bpData);

// Update BP measurement state machine
void updateBPMeasurement(BPMeasurementData *bpData, float currentPressure, bool heartbeatOccurred);

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