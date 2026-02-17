#ifndef SYSTOLIC_DETECTOR_H
#define SYSTOLIC_DETECTOR_H
#include "EnvelopeSmoother.h"
#include "config.h"


struct DetectionRecord
{
    float pressure;           // Systolic pressure at this detection
    unsigned long timestamp;  // When detected
    float confidence;         // Confidence score (updated as more beats follow)
    int subsequentBeats;      // How many beats followed this one
};

// Heart rate range for validation
struct HeartRateRange
{
    unsigned long minInterval;  // Maximum HR (shortest interval)
    unsigned long maxInterval;  // Minimum HR (longest interval)
    bool isValid;
    
    HeartRateRange() : minInterval(MIN_BEAT_INTERVALS_MS), maxInterval(MAX_BEAT_INTERVALS_MS), isValid(false) {}
    
    void setFromBPM(float baselineBPM, float tolerance = 40.0f) {
        if (baselineBPM > 0) {
            float minBPM = baselineBPM - tolerance;
            float maxBPM = baselineBPM + tolerance;
            
            // Clamp to reasonable limits
            float clampMin = 60000.0f / MAX_BEAT_INTERVALS_MS;
            float clampMax = 60000.0f / MIN_BEAT_INTERVALS_MS;
            if (minBPM < clampMin) minBPM = clampMin;
            if (maxBPM > clampMax) maxBPM = clampMax;
            
            // Convert BPM to milliseconds: interval = 60000 / BPM
            maxInterval = (unsigned long)(60000.0f / minBPM);  // Slower HR = longer interval
            minInterval = (unsigned long)(60000.0f / maxBPM);  // Faster HR = shorter interval
            isValid = true;
        }
    }
};

class SystolicDetector
{
protected:
    const char* name;
    int lastSignal;
    bool lastPulseState;

    // Track ALL detections
    DetectionRecord detections[MAX_DETECTIONS];
    int detectionCount;

    // Track intervals between beats for consistency checking
    unsigned long lastBeatTime;
    static const int MAX_INTERVALS = 10;
    unsigned long recentIntervals[MAX_INTERVALS];
    int intervalCount;

    // Heart rate range for validation
    HeartRateRange hrRange;

    // Helper to record a new detection
    void recordDetection(float pressure, unsigned long timestamp);
    
    // Helper to update confidence scores based on new beat
    void updateConfidenceScores(unsigned long currentTimestamp);
    
    // Helper to calculate interval consistency
    float calculateIntervalConsistency(int detectionIndex);

public:
    SystolicDetector(const char* detectorName);
    virtual ~SystolicDetector() {}
    virtual bool detect(int ppgSignal, float pressureSignal, unsigned long timestamp, unsigned long recentBaselineBeat) = 0;
    virtual void reset() = 0;
    
    const char* getName() const;
    int getDetectionCount() const;
    
    // Set expected heart rate range
    void setHeartRateRange(const HeartRateRange& range);
    
    // Get best detection(s)
    DetectionRecord getBestDetection() const;
    int softmaxNormalize(DetectionRecord* output, int maxCount, float temperature) const;
    void getTopDetections(DetectionRecord* output, int maxCount, int* actualCount) const;
};

// Statistical baseline detection with configurable parameters
class BaselineDetector : public SystolicDetector
{
private:
    int windowSize;
    float thresholdMultiplier;
    int minDeviation;

    int *baseline;
    int baselineIdx;
    int baselineCount;
    long baselineSum;
    int consecutiveAbove;
    
    char nameBuffer[64];

public:
    BaselineDetector(int window, float threshold,
                     int minDev);
    ~BaselineDetector();
    bool detect(int ppgSignal, float pressureSignal, unsigned long timestamp, unsigned long recentBaselineBeat) override;
    void reset() override;
};

// Derivative-based detection (detects rising edge)
class DerivativeDetector : public SystolicDetector {
public:
    explicit DerivativeDetector(int derivThreshold);
    ~DerivativeDetector() override;

    bool detect(int ppgSignal, float pressureSignal, unsigned long timestamp, unsigned long recentBaselineBeat) override;
    void reset() override;

private:
    int threshold;

    int prevSample = 0;
    int prevDerivative = 0;
    bool hasPrev = false;

    char nameBuffer[64];
};

class EnvelopeSystolicDetector : public SystolicDetector
{
private:
    EnvelopeDetector envelopeDetector;

    int windowSize;

    float* envelopeHistory;
    float* pressureHistory;
    unsigned long* timeHistory;

    int historyIdx;
    int historyCount;

    bool detectionMade;
    char nameBuffer[64];

    // Analysis helpers
    bool isEnvelopeFlat(int lookback);
    bool isEnvelopeIncreasing(int lookback);
    float calculateSlope(int samples);
    float calculateIntercept(float slope, int idx);
    int getAdaptiveFlatWindow();
    int getAdaptiveRiseWindow();
    float getAdaptiveEnvelopeThreshold();

public:
    explicit EnvelopeSystolicDetector(int window);
    ~EnvelopeSystolicDetector() override;

    bool detect(int ppgSignal, float pressureSignal, unsigned long timestamp, unsigned long recentBaselineBeat) override;
    void reset() override;
};
#endif