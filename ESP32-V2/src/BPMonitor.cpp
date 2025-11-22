#include "BPMonitor.h"
#include <string.h>

BPMonitor::BPMonitor(float startPressure, float minPressure, 
                     float pressureDropThreshold, unsigned long timeoutMs)
    : state(IDLE), systolic(0), maxPressure(0), startTime(0),
      detectorCount(0), startPressure(startPressure), minPressure(minPressure),
      pressureDropThreshold(pressureDropThreshold), timeoutMs(timeoutMs)
{
    for (int i = 0; i < MAX_DETECTORS; i++)
    {
        detectors[i] = nullptr;
    }
}

BPMonitor::~BPMonitor()
{
    // Detectors are managed externally
}

void BPMonitor::addDetector(PulseDetector *detector)
{
    if (detectorCount < MAX_DETECTORS)
    {
        detectors[detectorCount++] = detector;
    }
}

void BPMonitor::reset()
{
    state = IDLE;
    systolic = 0;
    maxPressure = 0;
    startTime = 0;
    
    for (int i = 0; i < detectorCount; i++)
    {
        detectors[i]->reset();
    }
}

void BPMonitor::update(const BPMeasurement& measurement)
{
    float pressure = measurement.pressure;
    int ppgSignal = measurement.ppgSignal;
    unsigned long currentTime = measurement.timestamp;
    
    // Track max pressure
    if (pressure > maxPressure)
    {
        maxPressure = pressure;
    }

    switch (state)
    {
    case IDLE:
        if (pressure > minPressure)
        {
            state = INFLATING;
        }
        break;

    case INFLATING:
        if (pressure >= startPressure)
        {
            state = MEASURING;
            startTime = currentTime;
        }
        else if (pressure < minPressure && maxPressure > startPressure)
        {
            reset();
        }
        break;

    case MEASURING:
        // Run detectors after sufficient pressure drop
        if (pressure < (maxPressure - pressureDropThreshold))
        {
            for (int i = 0; i < detectorCount; i++)
            {
                if (detectors[i]->getSystolic() == 0)
                {
                    detectors[i]->detect(ppgSignal, pressure);
                }
            }
        }

        // Check for timeout
        if (currentTime - startTime > timeoutMs)
        {
            state = COMPLETE;
        }

        // End when pressure drops low
        if (pressure < 10)
        {
            state = COMPLETE;
        }
        break;

    case COMPLETE:
        // Stay in complete state
        break;
    }
}

BPStatus BPMonitor::getStatus() const
{
    BPStatus status;
    status.state = state;
    status.currentPressure = maxPressure; // Current would need to be tracked separately
    status.maxPressure = maxPressure;
    
    switch (state)
    {
    case IDLE:
        status.statusMessage = "Waiting...";
        status.detailMessage = "";
        break;
    case INFLATING:
        status.statusMessage = "Inflating cuff...";
        status.detailMessage = "Pressure: ";
        break;
    case MEASURING:
        status.statusMessage = "Deflating cuff...";
        status.detailMessage = "Pressure: ";
        break;
    case COMPLETE:
        status.statusMessage = "Measurement";
        status.detailMessage = "Complete";
        break;
    }
    
    return status;
}

float BPMonitor::getSystolic() const
{
    return systolic;
}

BPState BPMonitor::getState() const
{
    return state;
}

int BPMonitor::getDetectorCount() const
{
    return detectorCount;
}

PulseDetector* BPMonitor::getDetector(int index) const
{
    if (index >= 0 && index < detectorCount)
    {
        return detectors[index];
    }
    return nullptr;
}