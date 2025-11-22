#ifndef BP_MONITOR_H
#define BP_MONITOR_H

#include "PulseDetector.h"

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

class BPMonitor
{
private:
    BPState state;
    float systolic;
    float maxPressure;
    unsigned long startTime;

    static const int MAX_DETECTORS = 20;
    PulseDetector *detectors[MAX_DETECTORS];
    int detectorCount;

    // Thresholds (could be injected via constructor)
    float startPressure;
    float minPressure;
    float pressureDropThreshold;
    unsigned long timeoutMs;

public:
    BPMonitor(float startPressure = 180.0f, 
              float minPressure = 80.0f,
              float pressureDropThreshold = 10.0f,
              unsigned long timeoutMs = 90000);
    
    ~BPMonitor();
    
    void addDetector(PulseDetector *detector);
    void reset();
    void update(const BPMeasurement& measurement);
    
    BPStatus getStatus() const;
    float getSystolic() const;
    BPState getState() const;
    int getDetectorCount() const;
    PulseDetector* getDetector(int index) const;
};

#endif