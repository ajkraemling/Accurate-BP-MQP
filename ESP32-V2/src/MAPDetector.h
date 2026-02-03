#ifndef MAP_DETECTOR_H
#define MAP_DETECTOR_H

#include "config.h"

// Beat record: stores one cardiac cycle measurement
struct BeatRecord {
    float cuffPressure;      // Cuff pressure at beat (mmHg)
    float amplitude;         // Oscillation amplitude (integrated energy)
    unsigned long timestamp; // Time of beat (ms)
};

// Forward declaration
struct PulseBuffer;

class MAPDetector {
public:
    static constexpr int MAX_BEATS = 100;

    MAPDetector();
    ~MAPDetector();

    // Reset detector state (call before new measurement)
    void reset();

    // ✅ UPDATED: Now takes oscillation amplitude from BPMonitor
    // pressure  = cuff pressure (mmHg)
    // osc       = band-pass filtered oscillometric signal
    // timestamp = time in ms
    void addSample(float pressure, float osc, unsigned long timestamp);

    // Attempt to calculate BP from accumulated beats
    bool detectMAP();

    // Get results (returns -1 if not yet calculated)
    float getMAP() const;
    float getSystolic() const;
    float getDiastolic() const;

    // Beat information
    int getBeatCount() const;
    void getBeatRecords(BeatRecord* out, int maxCount, int* actualCount) const;
    float getLatestAmplitude() const;
    float getLatestBeatPressure() const;

    // Tuning parameters
    void setSystolicRatio(float r);
    void setDiastolicRatio(float r);
    void setMinPeakAmplitude(float a);
    void setMaxReasonableAmplitude(float a);

    // Legacy interface (kept for compatibility, not used in new flow)
    float extractTrend(float pressure);
    float calcOscillation(float pressure, float trend);

private:
    // Beat storage
    BeatRecord beats[MAX_BEATS];
    int beatCount;

    // Detection state
    unsigned long lastPeakTime;
    float lastOscillation;
    float lastDerivative;
    bool initializationPhase;
    int filterSettleCount;
    bool inPulse;
    PulseBuffer* pulseBuffer;

    // Trend tracking
    float trendBuffer[TREND_WINDOW];
    int trendIdx;
    int trendCount;

    // Results
    float mapPressure;
    float systolicPressure;
    float diastolicPressure;

    // Tuning parameters
    float systolicRatio;
    float diastolicRatio;
    float minPeakAmplitude;
    float maxReasonableAmplitude;

    // Internal helpers
    void recordBeat(float pressure, float amplitude, unsigned long timestamp);
    float findMAPFromBeats(int startIdx, int endIdx) const;
    bool findSystolicDiastolic(float &sbp, float &dbp, int startIdx, int endIdx) const;
};

#endif
