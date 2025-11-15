#ifndef BP_MONITOR_H
#define BP_MONITOR_H

#include <Arduino.h>
#include "config.h"
#include "Display.h"
#include "PulseDetector.h"

enum BPState
{
    IDLE,
    INFLATING,
    MEASURING,
    COMPLETE
};

struct DetectionResult
{
    float pressure;
    int detectorIndex;
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

    // Store detection results for CSV output
    DetectionResult detectionResults[MAX_DETECTORS];
    int detectionCount;

public:
    BPMonitor();
    ~BPMonitor();
    void addDetector(PulseDetector *detector);
    void reset();
    void update(float pressure, int ppgSignal, Display &display);
    void printCSVRow(float pressure, int ppgSignal);
    void printCSVHeader() const;
    float getSystolic() const;
    BPState getState() const;
};

#endif