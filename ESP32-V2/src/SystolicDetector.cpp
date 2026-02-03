#include "SystolicDetector.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// ===== BASE CLASS IMPLEMENTATION =====

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
    // Resolve interval limits once
    unsigned long minInterval = MIN_BEAT_INTERVALS_MS;
    unsigned long maxInterval = MAX_BEAT_INTERVALS_MS;

    if (hrRange.isValid)
    {
        minInterval = hrRange.minInterval;
        maxInterval = hrRange.maxInterval;
    }

    for (int i = 0; i < detectionCount - 1; i++)
    {
        DetectionRecord& det = detections[i];

        int beatsAfter = 0;
        unsigned long expectedNextBeat = det.timestamp;

        for (int j = i + 1; j < detectionCount; j++)
        {
            unsigned long interval = detections[j].timestamp - expectedNextBeat;

            // Skip early false detections
            if (interval < minInterval)
            {
                continue;
            }

            // Too late → rhythm broken
            if (interval > maxInterval)
            {
                break;
            }

            // Valid beat
            beatsAfter++;
            expectedNextBeat = detections[j].timestamp;
        }

        det.subsequentBeats = beatsAfter;

        // No supporting beats → almost zero confidence
        if (beatsAfter == 0)
        {
            det.confidence = 0.01f;
            continue;
        }

        // Primary score: number of subsequent beats
        float beatBonus = 0.1f + (beatsAfter * 0.08f);  // slower growth
        if (beatBonus > 1.2f) beatBonus = 1.2f;

        // Rhythm consistency
        float consistencyBonus = calculateIntervalConsistency(i);
        if (consistencyBonus > 1.0f) consistencyBonus = 1.0f;

        // Mild early bonus (tie-breaker only)
        float positionFactor = (float)(detectionCount - i - 1) / (float)detectionCount;
        float earlyBonus = 1.0f; //+ (0.1f * positionFactor);  // max ~1.1

        det.confidence = beatBonus * consistencyBonus * earlyBonus;
    }

    // Most recent detection has no future context yet
    if (detectionCount > 0)
    {
        detections[detectionCount - 1].confidence = 0.05f;
    }
}

float SystolicDetector::calculateIntervalConsistency(int detectionIndex)
{
    if (detectionIndex >= detectionCount - 1)
    {
        return 0.7f;  // No future intervals yet
    }

    // Resolve interval limits
    unsigned long minInterval = MIN_BEAT_INTERVALS_MS;
    unsigned long maxInterval = MAX_BEAT_INTERVALS_MS;

    if (hrRange.isValid)
    {
        minInterval = hrRange.minInterval;
        maxInterval = hrRange.maxInterval;
    }

    unsigned long intervals[MAX_DETECTIONS];
    int intervalCount = 0;

    unsigned long lastValidTimestamp = detections[detectionIndex].timestamp;

    for (int i = detectionIndex + 1; i < detectionCount; i++)
    {
        unsigned long interval = detections[i].timestamp - lastValidTimestamp;

        // Skip early false detections
        if (interval < minInterval)
        {
            continue;
        }

        // Too late → rhythm broken
        if (interval > maxInterval)
        {
            break;
        }

        // Valid interval
        intervals[intervalCount++] = interval;
        lastValidTimestamp = detections[i].timestamp;

        if (intervalCount >= MAX_DETECTIONS)
            break;
    }

    // Not enough data to judge consistency
    if (intervalCount < 2)
    {
        return 0.7f;
    }

    // Mean interval
    float mean = 0.0f;
    for (int i = 0; i < intervalCount; i++)
    {
        mean += intervals[i];
    }
    mean /= intervalCount;

    // Standard deviation
    float variance = 0.0f;
    for (int i = 0; i < intervalCount; i++)
    {
        float diff = intervals[i] - mean;
        variance += diff * diff;
    }

    float stdDev = sqrtf(variance / intervalCount);
    float cv = stdDev / mean;

    // Convert CV to consistency score
    if (cv < 0.10f) return 1.3f;
    if (cv < 0.15f) return 1.2f;
    if (cv < 0.20f) return 1.0f;
    if (cv < 0.30f) return 0.8f;
    return 0.5f;
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

int SystolicDetector::softmaxNormalize(DetectionRecord* output, int maxCount, float temperature) const
{
    if (detectionCount == 0 || maxCount == 0)
        return 0;

    // Determine how many to copy
    int count = detectionCount < maxCount ? detectionCount : maxCount;

    // compute unnormalized softmax weights
    float weights[MAX_DETECTIONS];  // temp array
    float sumExp = 0.0f;
    for (int i = 0; i < count; i++)
    {
        weights[i] = powf(detections[i].confidence / temperature, 2);
        sumExp += weights[i];
    }

    // normalize and write output
    for (int i = 0; i < count; i++)
    {
        output[i] = detections[i];
        output[i].confidence = weights[i] / sumExp;
    }

    return count;
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

float SystolicDetector::getSystolic() const
{
    DetectionRecord best = getBestDetection();
    return best.pressure;
}

float SystolicDetector::getConfidence() const
{
    DetectionRecord best = getBestDetection();
    return best.confidence;
}

// ===== BASELINE DETECTOR IMPLEMENTATION =====

BaselineDetector::BaselineDetector(int window, float threshold, int minDev)
    : SystolicDetector("BaselineDetector"),
      windowSize(window),
      thresholdMultiplier(threshold),
      minDeviation(minDev),
      baseline(nullptr),
      baselineIdx(0),
      baselineCount(0),
      baselineSum(0),
      consecutiveAbove(0)
{
    baseline = new int[windowSize];
    memset(baseline, 0, windowSize * sizeof(int));
    snprintf(nameBuffer, sizeof(nameBuffer), "Baseline(w=%d,t=%.1f,d=%d)", 
             window, threshold, minDev);
    name = nameBuffer;
}

BaselineDetector::~BaselineDetector()
{
    delete[] baseline;
}

void BaselineDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp)
{
    // Update baseline buffer
    if (baselineCount < windowSize)
    {
        baseline[baselineCount] = ppgSignal;
        baselineSum += ppgSignal;
        baselineCount++;
    }
    else
    {
        baselineSum -= baseline[baselineIdx];
        baseline[baselineIdx] = ppgSignal;
        baselineSum += ppgSignal;
        baselineIdx = (baselineIdx + 1) % windowSize;
    }

    if (baselineCount < windowSize)
    {
        lastSignal = ppgSignal;
        return;
    }

    int avgBaseline = baselineSum / windowSize;
    int deviation = ppgSignal - avgBaseline;
    int threshold = (int)(thresholdMultiplier * avgBaseline);
    if (threshold < minDeviation)
        threshold = minDeviation;

    bool isPulse = (deviation > threshold);

    if (isPulse && !lastPulseState)
    {
        // Rising edge
        consecutiveAbove++;
        if (consecutiveAbove >= 2)
        {
            recordDetection(pressureSignal, timestamp);
        }
    }
    else if (!isPulse)
    {
        consecutiveAbove = 0;
    }

    lastPulseState = isPulse;
    lastSignal = ppgSignal;
}

void BaselineDetector::reset()
{
    baselineIdx = 0;
    baselineCount = 0;
    baselineSum = 0;
    consecutiveAbove = 0;
    lastSignal = 0;
    lastPulseState = false;
    detectionCount = 0;
    lastBeatTime = 0;
    intervalCount = 0;
    memset(baseline, 0, windowSize * sizeof(int));
    memset(recentIntervals, 0, sizeof(recentIntervals));
}

// ===== DERIVATIVE DETECTOR IMPLEMENTATION =====

DerivativeDetector::DerivativeDetector(int derivThreshold)
    : SystolicDetector("DerivativeDetector"),
      threshold(derivThreshold),
      prevSample(0),
      prevDerivative(0),
      hasPrev(false)
{
    snprintf(nameBuffer, sizeof(nameBuffer), "Derivative(t=%d)", derivThreshold);
    name = nameBuffer;
}

DerivativeDetector::~DerivativeDetector()
{
}

void DerivativeDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp)
{
    if (!hasPrev)
    {
        prevSample = ppgSignal;
        hasPrev = true;
        return;
    }

    int derivative = ppgSignal - prevSample;

    // Detect zero crossing from negative to positive (rising edge)
    if (prevDerivative <= 0 && derivative > threshold)
    {
        recordDetection(pressureSignal, timestamp);
    }

    prevSample = ppgSignal;
    prevDerivative = derivative;
}

void DerivativeDetector::reset()
{
    prevSample = 0;
    prevDerivative = 0;
    hasPrev = false;
    detectionCount = 0;
    lastBeatTime = 0;
    intervalCount = 0;
    lastSignal = 0;
    lastPulseState = false;
    memset(recentIntervals, 0, sizeof(recentIntervals));
}

// ===== ENVELOPE DETECTOR IMPLEMENTATION =====

EnvelopeSystolicDetector::EnvelopeSystolicDetector(int window)
    : SystolicDetector("EnvelopeDetector"),
      envelopeDetector(),  // Default constructor
      windowSize(window),
      historyIdx(0),
      historyCount(0),
      detectionMade(false)
{
    envelopeHistory = new float[windowSize];
    pressureHistory = new float[windowSize];
    timeHistory = new unsigned long[windowSize];
    
    memset(envelopeHistory, 0, windowSize * sizeof(float));
    memset(pressureHistory, 0, windowSize * sizeof(float));
    memset(timeHistory, 0, windowSize * sizeof(unsigned long));
    
    snprintf(nameBuffer, sizeof(nameBuffer), "Envelope(w=%d)", window);
    name = nameBuffer;
}

EnvelopeSystolicDetector::~EnvelopeSystolicDetector()
{
    delete[] envelopeHistory;
    delete[] pressureHistory;
    delete[] timeHistory;
}

void EnvelopeSystolicDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp)
{
    float envelope = envelopeDetector.update(ppgSignal);
    
    // Store in circular buffer
    envelopeHistory[historyIdx] = envelope;
    pressureHistory[historyIdx] = pressureSignal;
    timeHistory[historyIdx] = timestamp;
    
    historyIdx = (historyIdx + 1) % windowSize;
    if (historyCount < windowSize)
        historyCount++;
    
    if (historyCount < windowSize)
        return;
    
    // Adaptive parameters
    int flatWindow = getAdaptiveFlatWindow();
    int riseWindow = getAdaptiveRiseWindow();
    float envThreshold = getAdaptiveEnvelopeThreshold();
    
    // Check for flat envelope followed by rise
    bool wasFlat = isEnvelopeFlat(flatWindow);
    bool isRising = isEnvelopeIncreasing(riseWindow);
    
    if (wasFlat && isRising && !detectionMade)
    {
        // Find pressure at start of rise
        int startIdx = (historyIdx - riseWindow + windowSize) % windowSize;
        float detectionPressure = pressureHistory[startIdx];
        unsigned long detectionTime = timeHistory[startIdx];
        
        recordDetection(detectionPressure, detectionTime);
        detectionMade = true;
    }
    else if (!isRising)
    {
        detectionMade = false;
    }
}

void EnvelopeSystolicDetector::reset()
{
    envelopeDetector.reset();
    historyIdx = 0;
    historyCount = 0;
    detectionMade = false;
    detectionCount = 0;
    lastBeatTime = 0;
    intervalCount = 0;
    lastSignal = 0;
    lastPulseState = false;
    
    memset(envelopeHistory, 0, windowSize * sizeof(float));
    memset(pressureHistory, 0, windowSize * sizeof(float));
    memset(timeHistory, 0, windowSize * sizeof(unsigned long));
    memset(recentIntervals, 0, sizeof(recentIntervals));
}

bool EnvelopeSystolicDetector::isEnvelopeFlat(int lookback)
{
    if (historyCount < lookback)
        return false;
    
    float slope = calculateSlope(lookback);
    return (fabs(slope) < 0.1f);  // Threshold for flatness
}

bool EnvelopeSystolicDetector::isEnvelopeIncreasing(int lookback)
{
    if (historyCount < lookback)
        return false;
    
    float slope = calculateSlope(lookback);
    return (slope > 0.2f);  // Threshold for increase
}

float EnvelopeSystolicDetector::calculateSlope(int samples)
{
    if (samples > historyCount)
        samples = historyCount;
    
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    
    for (int i = 0; i < samples; i++)
    {
        int idx = (historyIdx - samples + i + windowSize) % windowSize;
        float x = i;
        float y = envelopeHistory[idx];
        
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }
    
    float n = samples;
    float slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    
    return slope;
}

float EnvelopeSystolicDetector::calculateIntercept(float slope, int idx)
{
    return envelopeHistory[idx] - slope * idx;
}

int EnvelopeSystolicDetector::getAdaptiveFlatWindow()
{
    return windowSize / 3;  // Use 1/3 of window
}

int EnvelopeSystolicDetector::getAdaptiveRiseWindow()
{
    return windowSize / 4;  // Use 1/4 of window
}

float EnvelopeSystolicDetector::getAdaptiveEnvelopeThreshold()
{
    // Calculate mean envelope over entire window
    float sum = 0;
    for (int i = 0; i < historyCount; i++)
    {
        sum += envelopeHistory[i];
    }
    return sum / historyCount * 0.1f;  // 10% of mean
}