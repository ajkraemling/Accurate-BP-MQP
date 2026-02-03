#ifndef BP_MONITOR_H
#define BP_MONITOR_H

#include "SystolicDetector.h"
#include "filters.h"
#include "MAPDetector.h"

enum BPState
{
    IDLE,
    INFLATING,
    MEASURING,
    COMPLETE
};

struct BPMeasurement
{
    float pressure;           // Cuff pressure (mmHg)
    int ppgSignal;            // Filtered PPG signal
    int rawPPGSignal;         // Raw PPG (debug)
    unsigned long timestamp;  // Time (ms)
};

struct BPStatus
{
    BPState state;
    float currentPressure;
    float maxPressure;
    const char* statusMessage;
    const char* detailMessage;
};

struct BPResult
{
    float systolic;
    float confidence;
    float confidenceIntervalLow;
    float confidenceIntervalHigh;
    int agreementCount;
    int totalDetectors;
};

class BPMonitor
{
private:
    BPState state;
    float systolic;
    float maxPressure;
    unsigned long startTime;
    

    // ================= SYSTOLIC DETECTORS =================
    static const int MAX_DETECTORS = 200;
    static const int MAX_READINGS_PER_DETECTOR = 50;

    SystolicDetector *detectors[MAX_DETECTORS];
    int detectorCount;

    // ================= BASELINE HEART RATE =================
    static const int MAX_BASELINE_BEATS = 200;
    unsigned long baselineBeats[MAX_BASELINE_BEATS];
    int baselineBeatCount;
    HeartRateRange baselineHR;
    bool hrCalculated;
    unsigned long lastBPMMeasurement;

    // Pressure oscillation tracking (for HR during inflation)
    static const int PRESSURE_HISTORY_SIZE = 5;
    float pressureHistory[PRESSURE_HISTORY_SIZE];
    int pressureHistoryIdx;
    int pressureHistoryCount;
    float lastPressureDerivative;
    unsigned long lastPeakTime;

    // ================= PPG FILTER =================
    PPGBandpassFilter* externalFilter;   // Provided from main

    // ================= MAP DETECTOR =================
    MAPDetector mapDetector;

    // Internal helpers
    void calculateBaselineHeartRate(unsigned long currentTime);
    bool detectPressureOscillation(float currentPressure, unsigned long timestamp);

public:
    BPMonitor();

    void addDetector(SystolicDetector *detector);
    void reset();
    void update(const BPMeasurement& measurement);

    // Provide external bandpass filter (PPG)
    void setFilter(PPGBandpassFilter* filter);

    BPStatus getStatus() const;
    float getSystolic() const;
    BPState getState() const;
    int getDetectorCount() const;
    SystolicDetector* getDetector(int index) const;

    // Ensemble systolic result
    float getBestSystolic(float* outConfidence = nullptr) const;
    BPResult getEnsembleResult() const;

    // Baseline HR info
    HeartRateRange getBaselineHeartRate() const;
    float getBaselineBPM() const;
    const unsigned long* getBaselineBeats(int& outCount) const;

    // ================= MAP ACCESS =================
    float getMAP();
    MAPDetector* getMAPDetector();
};

#endif