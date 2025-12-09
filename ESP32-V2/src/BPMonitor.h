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
    float pressure;
    int ppgSignal;
    int rawPPGSignal;
    unsigned long timestamp;
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
    float systolic;              // Best systolic reading
    float confidence;            // Confidence score (0.0-1.0)
    float confidenceIntervalLow; // Lower bound of 95% CI
    float confidenceIntervalHigh;// Upper bound of 95% CI
    int agreementCount;          // Number of detectors in agreement
    int totalDetectors;          // Total detectors that detected something
};

class BPMonitor
{
private:
    BPState state;
    float systolic;
    float maxPressure;
    unsigned long startTime;

    static const int MAX_DETECTORS = 200;
    SystolicDetector *detectors[MAX_DETECTORS];
    int detectorCount;

    // Thresholds
    float startPressure;
    float minPressure;
    float pressureDropThreshold;
    unsigned long timeoutMs;
    
    // Baseline heart rate tracking
    static const int MAX_BASELINE_BEATS = 10;
    unsigned long baselineBeats[MAX_BASELINE_BEATS];
    int baselineBeatCount;
    HeartRateRange baselineHR;
    bool hrCalculated;
    
    // Pressure oscillation detection for baseline HR
    static const int PRESSURE_HISTORY_SIZE = 5;
    float pressureHistory[PRESSURE_HISTORY_SIZE];
    int pressureHistoryIdx;
    int pressureHistoryCount;
    float lastPressureDerivative;
    
    // Optional external filter to configure
    PPGBandpassFilter* externalFilter;

    MAPDetector mapDetector;
    
    // Calculate baseline HR from inflation pressure oscillations
    void calculateBaselineHeartRate();
    
    // Detect pressure oscillations (heartbeats)
    bool detectPressureOscillation(float currentPressure, unsigned long timestamp);

public:
    BPMonitor(float startPressure = 140.0f, 
              float minPressure = 80.0f,
              float pressureDropThreshold = 20.0f,
              unsigned long timeoutMs = 90000);
    
    void addDetector(SystolicDetector *detector);
    void reset();
    void update(const BPMeasurement& measurement);
    
    // Set external filter to be configured based on baseline HR
    void setFilter(PPGBandpassFilter* filter);
    
    BPStatus getStatus() const;
    float getSystolic() const;
    BPState getState() const;
    int getDetectorCount() const;
    SystolicDetector* getDetector(int index) const;
    
    // Get best reading across all detectors
    float getBestSystolic(float* outConfidence = nullptr) const;
    
    // Get ensemble result with confidence interval
    BPResult getEnsembleResult() const;
    
    // Get baseline heart rate info
    HeartRateRange getBaselineHeartRate() const;
    float getBaselineBPM() const;

    // MAP 
    bool hasValidMAPData() const;
    int getOscillationCount() const;
    float getMAP();
    MAPDetector* getMAPDetector();
};

#endif