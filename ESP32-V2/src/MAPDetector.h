#ifndef MAP_DETECTOR_H
#define MAP_DETECTOR_H

#include "config.h"

// Beat record: stores one cardiac cycle measurement
struct BeatRecord {
    float cuffPressure;      // Cuff pressure at beat (mmHg)
    float amplitude;         // Oscillation amplitude (mmHg)
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

    // Process new pressure sample
    // pressure: raw cuff pressure in mmHg
    // timestamp: time in milliseconds
    void addSample(float pressure, unsigned long timestamp);

    // Attempt to calculate BP from accumulated beats
    // Returns true if successful (requires sufficient beats)
    bool detectMAP();

    // Get results (returns -1 if not yet calculated)
    float getMAP() const;
    float getSystolic() const;
    float getDiastolic() const;

    // Get beat information
    int getBeatCount() const;
    void getBeatRecords(BeatRecord* out, int maxCount, int* actualCount) const;
    float getLatestAmplitude() const;
    float getLatestBeatPressure() const;

    // Tuning parameters
    void setSystolicRatio(float r);      // Typically 0.50-0.58
    void setDiastolicRatio(float r);     // Typically 0.80-0.90
    void setMinPeakAmplitude(float a);   // Minimum valid amplitude (mmHg)
    void setMaxReasonableAmplitude(float a); // Maximum valid amplitude (mmHg)

    // Legacy interface (for compatibility)
    float extractTrend(float pressure);
    float calcOscillation(float pressure, float trend);

private:
    // Beat storage
    BeatRecord beats[MAX_BEATS];
    int beatCount;

    // Detection state
    unsigned long lastPeakTime;
    float lastOscillation;
    float lastPressure;
    bool wasRising;
    bool filtersSettled;
    int filterSettleCount;
    bool initializationPhase;
    
    // Integration method state
    float lastDerivative;
    bool inPulse;
    PulseBuffer* pulseBuffer;

    // Legacy trend buffer (unused in current implementation)
    float trendBuffer[TREND_WINDOW];
    int trendIdx;
    int trendCount;

    // Peak detection state (unused in current implementation)
    bool inPotentialPeak;
    float peakCandidateValue;
    float peakCandidatePressure;

    // Results
    float mapPressure;
    float systolicPressure;
    float diastolicPressure;

    // Tuning parameters
    float systolicRatio;
    float diastolicRatio;
    float minPeakAmplitude;
    float maxReasonableAmplitude;

    // Internal methods
    void recordBeat(float pressure, float amplitude, unsigned long timestamp);
    float findMAPFromBeats(int startIdx, int endIdx) const;
    bool findSystolicDiastolic(float &sbp, float &dbp, int startIdx, int endIdx) const;
};

#endif