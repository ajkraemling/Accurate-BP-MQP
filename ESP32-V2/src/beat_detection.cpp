#include "beat_detection.h"
#include "config.h"

void initializeBeatDetection(BeatDetectionState *state)
{
    state->upperThreshold = 0;
    state->lowerThreshold = 0;
    state->beatDetected = false;
    state->lastBeatTime = 0;
    state->lastThresholdUpdate = 0;
    state->beatIndex = 0;

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
