#include "SystolicDetector.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// Base class implementation
SystolicDetector::SystolicDetector(const char* detectorName)
    : name(detectorName), lastSignal(0), lastPulseState(false),
      detectionCount(0), lastBeatTime(0), intervalCount(0)
{
    memset(detections, 0, sizeof(detections));
    memset(recentIntervals, 0, sizeof(recentIntervals));
}

const char* SystolicDetector::getName() const
{
    return name;
}

int SystolicDetector::getDetectionCount() const
{
    return detectionCount;
}

void SystolicDetector::setHeartRateRange(const HeartRateRange& range)
{
    hrRange = range;
}

void SystolicDetector::recordDetection(float pressure, unsigned long timestamp)
{
    if (detectionCount < MAX_DETECTIONS)
    {
        detections[detectionCount].pressure = pressure;
        detections[detectionCount].timestamp = timestamp;
        detections[detectionCount].confidence = 0.1f;  // Initial low confidence
        detections[detectionCount].subsequentBeats = 0;
        detectionCount++;
    }
    
    // Track interval
    if (lastBeatTime > 0 && timestamp > lastBeatTime)
    {
        unsigned long interval = timestamp - lastBeatTime;
        
        // Store in recent intervals (rolling buffer)
        if (intervalCount < MAX_INTERVALS)
        {
            recentIntervals[intervalCount++] = interval;
        }
        else
        {
            // Shift left and add new
            for (int i = 0; i < MAX_INTERVALS - 1; i++)
            {
                recentIntervals[i] = recentIntervals[i + 1];
            }
            recentIntervals[MAX_INTERVALS - 1] = interval;
        }
    }
    
    lastBeatTime = timestamp;
    
    // Update confidence for all previous detections
    updateConfidenceScores(timestamp);
}

void SystolicDetector::updateConfidenceScores(unsigned long currentTimestamp)
{
    // Go through each previous detection and update its confidence
    // based on how many subsequent beats followed at consistent intervals
    
    for (int i = 0; i < detectionCount - 1; i++)  // -1 because current is already added
    {
        DetectionRecord& det = detections[i];
        
        // Count CONSECUTIVE beats that came after this detection
        // A beat only counts if it's within expected range (40-180 BPM = 333-1500ms)
        int beatsAfter = 0;
        unsigned long expectedNextBeat = det.timestamp;
        
        for (int j = i + 1; j < detectionCount; j++)
        {
            unsigned long interval = detections[j].timestamp - expectedNextBeat;
            
            // Check if this beat is within reasonable BPM range
            if (interval >= 333 && interval <= 1500)  // 40-180 BPM
            {
                beatsAfter++;
                expectedNextBeat = detections[j].timestamp;
            }
            else
            {
                // Beat was skipped or too irregular, stop counting
                break;
            }
        }
        det.subsequentBeats = beatsAfter;
        
        // Calculate confidence based on:
        // 1. How early this detection was (lower pressure index = earlier = better)
        // 2. How many beats followed it
        // 3. How consistent those intervals are
        
        float earlyBonus = 1.0f + (0.1f * (detectionCount - i - 1) / (float)detectionCount);
        if (i == 0) earlyBonus = 1.75f;  // Big bonus for first detection
        else if (i == 1) earlyBonus = 1.35f;
        else if (i == 2) earlyBonus = 1.15f;
        
        // Subsequent beats bonus
        float beatBonus = 0.2f + (beatsAfter * 0.15f);
        if (beatBonus > 1.0f) beatBonus = 1.0f;
        
        // Consistency bonus
        float consistencyBonus = calculateIntervalConsistency(i);
        
        // Combine factors
        det.confidence = beatBonus * consistencyBonus * earlyBonus;
        
        // Cap at 1.0
        // if (det.confidence > 1.0f) det.confidence = 1.0f;
    }
    
    // Current detection starts with low confidence until beats follow
    if (detectionCount > 0)
    {
        detections[detectionCount - 1].confidence = 0.1f;
    }
}

float SystolicDetector::calculateIntervalConsistency(int detectionIndex)
{
    // Look at CONSECUTIVE intervals AFTER this detection
    // Stop if a beat is skipped (interval out of range)
    if (detectionIndex >= detectionCount - 1)
    {
        return 0.5f;  // No intervals after this one yet
    }
    
    // Collect consecutive intervals from this detection forward
    int intervalCount = 0;
    unsigned long intervals[MAX_DETECTIONS];
    
    for (int i = detectionIndex; i < detectionCount - 1; i++)
    {
        unsigned long interval = detections[i + 1].timestamp - detections[i].timestamp;
        
        // Only count if within expected heart rate range
        bool inRange = false;
        if (hrRange.isValid) {
            inRange = (interval >= hrRange.minInterval && interval <= hrRange.maxInterval);
        } else {
            // Fallback to default (40-180 BPM = 333-1500ms)
            inRange = (interval >= 333 && interval <= 1500);
        }
        
        if (inRange) {
            intervals[intervalCount++] = interval;
        } else {
            break;  // Skip detected, stop counting
        }
    }
    
    if (intervalCount < 2)
    {
        return 0.7f;  // Need at least 2 consecutive intervals
    }
    
    // Calculate mean
    float mean = 0;
    for (int i = 0; i < intervalCount; i++)
    {
        mean += intervals[i];
    }
    mean /= intervalCount;
    
    // Calculate coefficient of variation
    float variance = 0;
    for (int i = 0; i < intervalCount; i++)
    {
        float diff = intervals[i] - mean;
        variance += diff * diff;
    }
    float stdDev = sqrt(variance / intervalCount);
    float cv = stdDev / mean;
    
    // Convert CV to consistency score
    float consistencyScore;
    if (cv < 0.1f)           // < 10% variation
        consistencyScore = 1.3f;
    else if (cv < 0.15f)     // < 15% variation
        consistencyScore = 1.2f;
    else if (cv < 0.2f)      // < 20% variation
        consistencyScore = 1.0f;
    else if (cv < 0.3f)      // < 30% variation
        consistencyScore = 0.8f;
    else                     // > 30% variation
        consistencyScore = 0.5f;
    
    return consistencyScore;
}

DetectionRecord SystolicDetector::getBestDetection() const
{
    DetectionRecord best;
    best.pressure = 0;
    best.timestamp = 0;
    best.confidence = 0;
    best.subsequentBeats = 0;
    
    for (int i = 0; i < detectionCount; i++)
    {
        if (detections[i].confidence > best.confidence)
        {
            best = detections[i];
        }
    }
    
    return best;
}

void SystolicDetector::getTopDetections(DetectionRecord* output, int maxCount, int* actualCount) const
{
    // Simple bubble sort to get top N
    DetectionRecord sorted[MAX_DETECTIONS];
    memcpy(sorted, detections, detectionCount * sizeof(DetectionRecord));
    
    // Sort by confidence (descending)
    for (int i = 0; i < detectionCount - 1; i++)
    {
        for (int j = 0; j < detectionCount - i - 1; j++)
        {
            if (sorted[j].confidence < sorted[j + 1].confidence)
            {
                DetectionRecord temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    // Copy top N
    int count = detectionCount < maxCount ? detectionCount : maxCount;
    memcpy(output, sorted, count * sizeof(DetectionRecord));
    *actualCount = count;
}

// BaselineDetector implementation
BaselineDetector::BaselineDetector(int window, float threshold, int minDev)
    : SystolicDetector(nullptr), windowSize(window), thresholdMultiplier(threshold),
      minDeviation(minDev), 
      baselineIdx(0), baselineCount(0), baselineSum(0), consecutiveAbove(0)
{
    snprintf(nameBuffer, sizeof(nameBuffer), "BL_W%d_T%.1f_D%d",
             window, threshold, minDev);
    name = nameBuffer;
    
    baseline = new int[windowSize];
    memset(baseline, 0, windowSize * sizeof(int));
}

BaselineDetector::~BaselineDetector()
{
    delete[] baseline;
}

bool BaselineDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp)
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

    // Detect rising edge (not just staying above threshold)
    bool currentlyAbove = ppgSignal > threshold;
    
    if (currentlyAbove)
    {
        // Minimum interval enforcement (300ms = ~200 BPM max)
        if (lastBeatTime == 0 || (timestamp - lastBeatTime) > 300)
        {
            recordDetection(pressureSignal, timestamp);
            return true;
        }
    }
    else
    {
        consecutiveAbove = 0;  // Reset when signal drops below threshold
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
    
    detectionCount = 0;
    lastBeatTime = 0;
    intervalCount = 0;
    memset(detections, 0, sizeof(detections));
    memset(recentIntervals, 0, sizeof(recentIntervals));
}

// DerivativeDetector implementation
DerivativeDetector::DerivativeDetector(int window, int derivThreshold)
    : SystolicDetector(nullptr), windowSize(window), threshold(derivThreshold),
      bufferIdx(0), bufferCount(0)
{
    snprintf(nameBuffer, sizeof(nameBuffer), "DRV_W%d_T%d", window, derivThreshold);
    name = nameBuffer;
    
    signalBuffer = new int[windowSize];
    memset(signalBuffer, 0, windowSize * sizeof(int));
}

DerivativeDetector::~DerivativeDetector()
{
    delete[] signalBuffer;
}

bool DerivativeDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp)
{
    
    signalBuffer[bufferIdx] = ppgSignal;
    bufferIdx = (bufferIdx + 1) % windowSize;
    if (bufferCount < windowSize)
    {
        bufferCount++;
        return false;
    }

    int oldestIdx = bufferIdx;
    int derivative = ppgSignal - signalBuffer[oldestIdx];

    int avgSignal = 0;
    for (int i = 0; i < windowSize; i++)
    {
        avgSignal += signalBuffer[i];
    }
    avgSignal /= windowSize;
    
    if (avgSignal < 5)
    {
        return false;  // Signal too weak/flat
    }

    // Minimum time between pulses using heart rate range
    unsigned long minInterval = hrRange.isValid ? hrRange.minInterval : 300;
    if (derivative > threshold && 
        (lastBeatTime == 0 || (timestamp - lastBeatTime) > minInterval))
    {
        recordDetection(pressureSignal, timestamp);
        return true;
    }

    return false;
}

void DerivativeDetector::reset()
{
    memset(signalBuffer, 0, windowSize * sizeof(int));
    bufferIdx = 0;
    bufferCount = 0;
    
    detectionCount = 0;
    lastBeatTime = 0;
    intervalCount = 0;
    memset(detections, 0, sizeof(detections));
    memset(recentIntervals, 0, sizeof(recentIntervals));
}

// EnsembleDetector implementation
EnsembleDetector::EnsembleDetector(const char* name, int requiredVotes)
    : SystolicDetector(name), detectorCount(0), votesRequired(requiredVotes)
{
    for (int i = 0; i < MAX_DETECTORS; i++)
    {
        detectors[i] = nullptr;
    }
}

void EnsembleDetector::addDetector(SystolicDetector *detector)
{
    if (detectorCount < MAX_DETECTORS)
    {
        detectors[detectorCount++] = detector;
    }
}

bool EnsembleDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp)
{
    int votes = 0;
    
    for (int i = 0; i < detectorCount; i++)
    {
        if (detectors[i]->detect(ppgSignal, pressureSignal, timestamp))
        {
            votes++;
        }
    }
    
    if (votes >= votesRequired)
    {
        recordDetection(pressureSignal, timestamp);
        return true;
    }
    return false;
}

void EnsembleDetector::reset()
{
    detectionCount = 0;
    lastBeatTime = 0;
    intervalCount = 0;
    memset(detections, 0, sizeof(detections));
    memset(recentIntervals, 0, sizeof(recentIntervals));
    
    for (int i = 0; i < detectorCount; i++)
    {
        detectors[i]->reset();
    }
}