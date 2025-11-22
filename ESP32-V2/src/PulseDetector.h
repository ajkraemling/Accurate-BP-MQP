#ifndef PULSE_DETECTOR_H
#define PULSE_DETECTOR_H

class PulseDetector
{
protected:
    const char* name;
    int lastSignal;
    bool lastPulseState;
    int systolic;

public:
    PulseDetector(const char* detectorName);
    virtual ~PulseDetector() {}
    virtual bool detect(int ppgSignal, float pressureSignal) = 0;
    virtual void reset() = 0;
    const char* getName() const;
    int getSystolic() const;
};

// Statistical baseline detection with configurable parameters
class BaselineDetector : public PulseDetector
{
private:
    int windowSize;
    float thresholdMultiplier;
    int minDeviation;
    int consecutiveRequired;

    int *baseline;
    int baselineIdx;
    int baselineCount;
    long baselineSum;
    int consecutiveAbove;
    
    char nameBuffer[64];

public:
    BaselineDetector(int window, float threshold,
                     int minDev, int consecutive);
    ~BaselineDetector();
    bool detect(int ppgSignal, float pressureSignal) override;
    void reset() override;
};

// Derivative-based detection (detects rising edge)
class DerivativeDetector : public PulseDetector
{
private:
    int windowSize;
    int threshold;
    int *signalBuffer;
    int bufferIdx;
    int bufferCount;
    unsigned long lastPulseTime;
    
    char nameBuffer[64];

public:
    DerivativeDetector(int window, int derivThreshold);
    ~DerivativeDetector();
    bool detect(int ppgSignal, float pressureSignal) override;
    void reset() override;
    
    void setCurrentTime(unsigned long currentTime);
};

// Voting ensemble that combines multiple detectors
class EnsembleDetector : public PulseDetector
{
private:
    static const int MAX_DETECTORS = 10;
    PulseDetector *detectors[MAX_DETECTORS];
    int detectorCount;
    int votesRequired;

public:
    EnsembleDetector(const char* name, int requiredVotes);
    void addDetector(PulseDetector *detector);
    bool detect(int ppgSignal, float pressureSignal) override;
    void reset() override;
};

#endif