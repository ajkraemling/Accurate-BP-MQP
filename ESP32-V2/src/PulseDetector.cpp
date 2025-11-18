#include "PulseDetector.h"

// Base class implementation
PulseDetector::PulseDetector(const String &detectorName)
    : name(detectorName), lastSignal(0), lastPulseState(false), systolic(0) {}

String PulseDetector::getName() const
{
    return name;
}

int PulseDetector::getSystolic() const
{
    return systolic;
}

// BaselineDetector implementation
BaselineDetector::BaselineDetector(int window, float threshold, int minDev, int consecutive)
    : PulseDetector("BL_W" + to_string(window) + "_T" + to_string(threshold) + "_D" + to_string(minDev) + "_C" + to_string(consecutive)), windowSize(window), thresholdMultiplier(threshold),
      minDeviation(minDev), consecutiveRequired(consecutive),
      baselineIdx(0), baselineCount(0), baselineSum(0), consecutiveAbove(0)
{
    baseline = new int[windowSize];
    memset(baseline, 0, windowSize * sizeof(int));
}

BaselineDetector::~BaselineDetector()
{
    delete[] baseline;
}

bool BaselineDetector::detect(int ppgSignal, float pressureSignal)
{
    if (ppgSignal < 10)
        return false;

    // Update rolling window
    if (baselineCount < windowSize)
    {
        baseline[baselineIdx] = ppgSignal;
        baselineSum += ppgSignal;
        baselineCount++;
    }
    else
    {
        baselineSum -= baseline[baselineIdx];
        baseline[baselineIdx] = ppgSignal;
        baselineSum += ppgSignal;
    }
    baselineIdx = (baselineIdx + 1) % windowSize;

    if (baselineCount < windowSize)
        return false;

    // Calculate statistics
    float mean = (float)baselineSum / windowSize;
    float variance = 0;
    for (int i = 0; i < windowSize; i++)
    {
        float diff = baseline[i] - mean;
        variance += diff * diff;
    }
    float stdDev = sqrt(variance / windowSize);

    // Threshold
    float threshold = mean + (thresholdMultiplier * stdDev);
    if (stdDev < minDeviation)
    {
        threshold = mean + minDeviation;
    }

    // Check for pulse
    if (ppgSignal > threshold)
    {
        consecutiveAbove++;
        if (consecutiveAbove >= consecutiveRequired)
        {
            consecutiveAbove = 0;
            systolic = pressureSignal;
            return true;
        }
    }
    else
    {
        consecutiveAbove = 0;
    }

    return false;
}

void BaselineDetector::reset()
{
    memset(baseline, 0, windowSize * sizeof(int));
    baselineIdx = 0;
    baselineCount = 0;
    baselineSum = 0;
    consecutiveAbove = 0;
}

// DerivativeDetector implementation
DerivativeDetector::DerivativeDetector(int window, int derivThreshold)
    : PulseDetector("DRV_W" +to_string(window)+"_T" + to_string(derivThreshold)), windowSize(window), threshold(derivThreshold),
      bufferIdx(0), bufferCount(0), lastPulseTime(0)
{
    signalBuffer = new int[windowSize];
    memset(signalBuffer, 0, windowSize * sizeof(int));
}

DerivativeDetector::~DerivativeDetector()
{
    delete[] signalBuffer;
}

bool DerivativeDetector::detect(int ppgSignal, float pressureSignal)
{
    // Fill buffer
    signalBuffer[bufferIdx] = ppgSignal;
    bufferIdx = (bufferIdx + 1) % windowSize;
    if (bufferCount < windowSize)
    {
        bufferCount++;
        return false;
    }

    // Calculate derivative (current - oldest)
    int oldestIdx = bufferIdx;
    int derivative = ppgSignal - signalBuffer[oldestIdx];

    // Detect rising edge with minimum time between pulses
    unsigned long now = millis();
    if (derivative > threshold && (now - lastPulseTime) > 300)
    {
        lastPulseTime = now;
        systolic = pressureSignal;
        return true;
    }

    return false;
}

void DerivativeDetector::reset()
{
    memset(signalBuffer, 0, windowSize * sizeof(int));
    bufferIdx = 0;
    bufferCount = 0;
    lastPulseTime = 0;
}

// EnsembleDetector implementation
EnsembleDetector::EnsembleDetector(const String &name, int requiredVotes)
    : PulseDetector(name), detectorCount(0), votesRequired(requiredVotes)
{
    for (int i = 0; i < MAX_DETECTORS; i++)
    {
        detectors[i] = nullptr;
    }
}

void EnsembleDetector::addDetector(PulseDetector *detector)
{
    if (detectorCount < MAX_DETECTORS)
    {
        detectors[detectorCount++] = detector;
    }
}

bool EnsembleDetector::detect(int ppgSignal, float pressureSignal)
{
    int votes = 0;
    for (int i = 0; i < detectorCount; i++)
    {
        if (detectors[i]->detect(ppgSignal, pressureSignal))
        {
            votes++;
        }
    }
    if (votes >= votesRequired)
    {
        systolic = pressureSignal;
        return true;
    }
    else
        return false;
}

void EnsembleDetector::reset()
{
    for (int i = 0; i < detectorCount; i++)
    {
        detectors[i]->reset();
    }
}