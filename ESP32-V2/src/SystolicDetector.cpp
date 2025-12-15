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
    for (int i = 0; i < detectionCount - 1; i++)
    {
        DetectionRecord& det = detections[i];
        
        // Count CONSECUTIVE beats that came after this detection
        int beatsAfter = 0;
        unsigned long expectedNextBeat = det.timestamp;
        
        for (int j = i + 1; j < detectionCount; j++)
        {
            unsigned long interval = detections[j].timestamp - expectedNextBeat;
            
            if (interval >= MIN_BEAT_INTERVALS_MS && interval <= MAX_BEAT_INTERVALS_MS)
            {
                beatsAfter++;
                expectedNextBeat = detections[j].timestamp;
            }
            else
            {
                break;
            }
        }
        det.subsequentBeats = beatsAfter;
        
        if (beatsAfter == 0) {
            det.confidence = 0.01f;
            continue;  // Skip to next detection
        }
        
        float earlyBonus = 1.0f;
        if (beatsAfter > 0) {  // Only give early bonus if beats actually followed
            if (i == 0) earlyBonus = 1.5f;
            else if (i == 1) earlyBonus = 1.25f;
            else if (i == 2) earlyBonus = 1.1f;
            else earlyBonus = 1.0f + (0.05f * (detectionCount - i - 1) / (float)detectionCount);
        }
        
        // Subsequent beats bonus - this is the PRIMARY score
        float beatBonus = 0.1f + (beatsAfter * 0.15f);
        if (beatBonus > 1.0f) beatBonus = 1.0f;
        
        // Consistency bonus
        float consistencyBonus = calculateIntervalConsistency(i);
        
        // Combine: beat count is most important, then consistency, then position
        det.confidence = beatBonus * consistencyBonus * earlyBonus;
    }
    
    // Current detection starts with very low confidence
    if (detectionCount > 0)
    {
        detections[detectionCount - 1].confidence = 0.05f;
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
            // Fallback to default (40-180 BPM = 300-1500ms)
            inRange = (interval >= MIN_BEAT_INTERVALS_MS && interval <= MAX_BEAT_INTERVALS_MS);
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
        if (lastBeatTime == 0 || (timestamp - lastBeatTime) > MIN_BEAT_INTERVALS_MS)
        {
            recordDetection(pressureSignal, timestamp);
            return true;
        }
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

DerivativeDetector::DerivativeDetector(int derivThreshold)
    : SystolicDetector(nullptr),
      threshold(derivThreshold),
      prevSample(0),
      prevDerivative(0),
      hasPrev(false)
{
    snprintf(nameBuffer, sizeof(nameBuffer), "DRV_T%d", derivThreshold);
    name = nameBuffer;
}

DerivativeDetector::~DerivativeDetector() = default;

bool DerivativeDetector::detect(int ppgSignal,
                                float pressureSignal,
                                unsigned long timestamp)
{
    if (!hasPrev) {
        prevSample = ppgSignal;
        prevDerivative = 0;
        hasPrev = true;
        return false;
    }

    int derivative = ppgSignal - prevSample;

    unsigned long minInterval =
        hrRange.isValid ? hrRange.minInterval : MIN_BEAT_INTERVALS_MS;

    bool timeOK =
        (lastBeatTime == 0) ||
        (timestamp - lastBeatTime >= minInterval);

    bool isRisingEdge =
        (prevDerivative > 0) &&
        (derivative <= 0) &&
        (prevDerivative >= threshold) &&
        timeOK;

    if (isRisingEdge) {
        recordDetection(pressureSignal, timestamp); // this sets lastBeatTime
    }

    prevDerivative = derivative;
    prevSample = ppgSignal;

    return isRisingEdge;
}

void DerivativeDetector::reset()
{
    detectionCount = 0;
    lastBeatTime = 0;
    intervalCount = 0;
    memset(detections, 0, sizeof(detections));
    memset(recentIntervals, 0, sizeof(recentIntervals));

    prevSample = 0;
    prevDerivative = 0;
    hasPrev = false;
}

EnvelopeSystolicDetector::EnvelopeSystolicDetector(int window)
    : SystolicDetector(nullptr),
      windowSize(window),
      historyIdx(0),
      historyCount(0),
      detectionMade(false)
{
    envelopeHistory = new float[windowSize];
    pressureHistory = new float[windowSize];
    timeHistory = new unsigned long[windowSize];

    snprintf(nameBuffer, sizeof(nameBuffer), "Env_W%d", windowSize);
    name = nameBuffer;

    reset();
}

EnvelopeSystolicDetector::~EnvelopeSystolicDetector()
{
    delete[] envelopeHistory;
    delete[] pressureHistory;
    delete[] timeHistory;
}

bool EnvelopeSystolicDetector::isEnvelopeFlat(int lookback)
{
    if (historyCount < lookback) return false;

    float mean = 0.0f;
    for (int i = 0; i < lookback; i++) {
        int idx = (historyIdx - 1 - i + windowSize) % windowSize;
        mean += envelopeHistory[idx];
    }
    mean /= lookback;

    float var = 0.0f;
    for (int i = 0; i < lookback; i++) {
        int idx = (historyIdx - 1 - i + windowSize) % windowSize;
        float d = envelopeHistory[idx] - mean;
        var += d * d;
    }
    var /= lookback;

    constexpr float VAR_THRESH = 2.0f;
    constexpr float AMP_THRESH = 10.0f;

    return (var < VAR_THRESH && mean < AMP_THRESH);
}

bool EnvelopeSystolicDetector::isEnvelopeIncreasing(int lookback)
{
    if (historyCount < lookback) return false;

    int rises = 0;
    for (int i = 1; i < lookback; i++) {
        int curr = (historyIdx - i + windowSize) % windowSize;
        int prev = (historyIdx - i - 1 + windowSize) % windowSize;
        if (envelopeHistory[curr] > envelopeHistory[prev])
            rises++;
    }

    return rises >= (lookback * 2) / 3;
}

float EnvelopeSystolicDetector::calculateSlope(int samples)
{
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

    for (int i = 0; i < samples; i++) {
        int idx = (historyIdx - samples + i + windowSize) % windowSize;
        float x = pressureHistory[idx];
        float y = envelopeHistory[idx];

        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    float denom = samples * sumX2 - sumX * sumX;
    if (fabs(denom) < 1e-3f) return 0.0f;

    return (samples * sumXY - sumX * sumY) / denom;
}


float EnvelopeSystolicDetector::calculateIntercept(float slope, int idx) {
    // y = slope * x + intercept
    // intercept = y - slope * x
    float y = envelopeHistory[idx];
    float x = pressureHistory[idx];
    return y - slope * x;
}

bool EnvelopeSystolicDetector::detect(int ppgSignal,
                                      float pressureSignal,
                                      unsigned long timestamp)
{
    float env = envelopeDetector.update((float)ppgSignal);

    envelopeHistory[historyIdx] = env;
    pressureHistory[historyIdx] = pressureSignal;
    timeHistory[historyIdx] = timestamp;

    historyIdx = (historyIdx + 1) % windowSize;
    if (historyCount < windowSize)
        historyCount++;

    if (historyCount < windowSize || detectionMade)
        return false;

    constexpr int FLAT_WIN = 10;
    constexpr int RISE_WIN = 8;
    constexpr float ENV_PRESENT = 5.0f;
    constexpr float MIN_SLOPE = 0.1f;
    constexpr float MAX_DELTA = 40.0f;

    bool flat = isEnvelopeFlat(FLAT_WIN);
    bool rising = isEnvelopeIncreasing(RISE_WIN);

    int currIdx = (historyIdx - 1 + windowSize) % windowSize;

    if (!flat || !rising || envelopeHistory[currIdx] < ENV_PRESENT)
        return false;

    float slope = calculateSlope(RISE_WIN);

    float systolic = pressureSignal;   // default fallback

    if (slope > MIN_SLOPE) {
        float intercept = envelopeHistory[currIdx]
                        - slope * pressureHistory[currIdx];

        float est = -intercept / slope;

        if (est > pressureSignal && est < pressureSignal + MAX_DELTA)
            systolic = est;
    }

    recordDetection(systolic, timestamp);
    detectionMade = true;
    return true;
}



void EnvelopeSystolicDetector::reset()
{
    envelopeDetector.reset();

    historyIdx = 0;
    historyCount = 0;
    detectionMade = false;

    memset(envelopeHistory, 0, windowSize * sizeof(float));
    memset(pressureHistory, 0, windowSize * sizeof(float));
    memset(timeHistory, 0, windowSize * sizeof(unsigned long));

    lastSignal = 0;
    lastPulseState = false;
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