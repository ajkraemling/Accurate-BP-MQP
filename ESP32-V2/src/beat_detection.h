#ifndef BEAT_DETECTION_H
#define BEAT_DETECTION_H

#include <Arduino.h>

typedef struct
{
    int upperThreshold;
    int lowerThreshold;
    bool beatDetected;
    unsigned long lastBeatTime;
    unsigned long lastThresholdUpdate;
    int recentBeats[5];
    int beatIndex;
} BeatDetectionState;

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

#endif