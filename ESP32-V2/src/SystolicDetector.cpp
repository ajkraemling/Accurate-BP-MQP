#include "SystolicDetector.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <Arduino.h>  // For Serial debug output

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

// Returns the number of normalized detections written to output
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

void BaselineDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp)
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
    {
        lastSignal = ppgSignal;
        return;
    }

    // Calculate statistics
    float mean = (float)baselineSum / windowSize;
    float variance = 0;
    for (int i = 0; i < windowSize; i++)
    {
        float diff = baseline[i] - mean;
        variance += diff * diff;
    }
    float stdDev = sqrt(variance / windowSize);

    // Threshold: mean + (multiplier × stdDev) OR mean + minDeviation, whichever is larger
    float threshold = mean + (thresholdMultiplier * stdDev);
    if (stdDev < minDeviation || threshold < mean + minDeviation)
    {
        threshold = mean + minDeviation;
    }

    // Debug output periodically (to avoid spam)
    static unsigned long lastDebug = 0;
    
    if (millis() - lastDebug > 5000) {  // Every 5 seconds
        lastDebug = millis();
        Serial.print("[Det w");
        Serial.print(windowSize);
        Serial.print(" d");
        Serial.print(minDeviation);
        Serial.print("] sig:");
        Serial.print(ppgSignal);
        Serial.print(" mean:");
        Serial.print((int)mean);
        Serial.print(" dev:");
        Serial.print((int)(ppgSignal - mean));
        Serial.print(" thr:");
        Serial.println((int)threshold);
    }

    // Detect rising edge (transition from below to above threshold)
    bool currentlyAbove = ppgSignal > threshold;
    bool wasBelow = !lastPulseState;
    
    // Rising edge detected
    if (currentlyAbove && wasBelow)
    {
        // Minimum interval enforcement (prevent multiple detections per pulse)
        if (lastBeatTime == 0 || (timestamp - lastBeatTime) > MIN_BEAT_INTERVALS_MS)
        {
            recordDetection(pressureSignal, timestamp);
        }
    }
    
    // Update state for next iteration
    lastPulseState = currentlyAbove;
    lastSignal = ppgSignal;
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
    lastSignal = 0;
    lastPulseState = false;
    memset(detections, 0, sizeof(detections));
    memset(recentIntervals, 0, sizeof(recentIntervals));
}

// ===== DERIVATIVE DETECTOR IMPLEMENTATION =====

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

void DerivativeDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp)
{
    if (!hasPrev) {
        prevSample = ppgSignal;
        prevDerivative = 0;
        hasPrev = true;
        return;
    }

    int derivative = ppgSignal - prevSample;

    unsigned long minInterval =
        hrRange.isValid ? hrRange.minInterval : MIN_BEAT_INTERVALS_MS;

    bool timeOK =
        (lastBeatTime == 0) ||
        (timestamp - lastBeatTime >= minInterval);

    // Detect peak: derivative was positive, now zero or negative, and exceeded threshold
    bool isRisingEdge =
        (prevDerivative > 0) &&
        (derivative <= 0) &&
        (prevDerivative >= threshold) &&
        timeOK;

    if (isRisingEdge) {
        recordDetection(pressureSignal, timestamp);
    }

    prevDerivative = derivative;
    prevSample = ppgSignal;
}

void DerivativeDetector::reset()
{
    detectionCount = 0;
    lastBeatTime = 0;
    intervalCount = 0;
    lastSignal = 0;
    lastPulseState = false;
    memset(detections, 0, sizeof(detections));
    memset(recentIntervals, 0, sizeof(recentIntervals));

    prevSample = 0;
    prevDerivative = 0;
    hasPrev = false;
}

// ===== ENVELOPE DETECTOR IMPLEMENTATION =====

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
    float y = envelopeHistory[idx];
    float x = pressureHistory[idx];
    return y - slope * x;
}

int EnvelopeSystolicDetector::getAdaptiveFlatWindow() {
    if (!hrRange.isValid) return 10;
    
    unsigned long typicalInterval = (hrRange.minInterval + hrRange.maxInterval) / 2;
    float samplesPerBeat = typicalInterval / SAMPLE_RATE_MS;
    int adaptiveWindow = (int)(samplesPerBeat * 1.5f);
    
    if (adaptiveWindow < 5) adaptiveWindow = 5;
    if (adaptiveWindow > 25) adaptiveWindow = 25;
    
    return adaptiveWindow;
}

int EnvelopeSystolicDetector::getAdaptiveRiseWindow() {
    if (!hrRange.isValid) return 8;
    
    unsigned long typicalInterval = (hrRange.minInterval + hrRange.maxInterval) / 2;
    float samplesPerBeat = typicalInterval / SAMPLE_RATE_MS;
    int adaptiveWindow = (int)(samplesPerBeat * 1.0f);
    
    if (adaptiveWindow < 5) adaptiveWindow = 5;
    if (adaptiveWindow > 20) adaptiveWindow = 20;
    
    return adaptiveWindow;
}

float EnvelopeSystolicDetector::getAdaptiveEnvelopeThreshold() {
    if (historyCount < 10) return 5.0f;
    
    float maxRecent = 0.0f;
    int loopMax = 20;
    if (loopMax > historyCount) loopMax = historyCount;
    for (int i = 0; i < loopMax ; i++) {
        int idx = (historyIdx - 1 - i + windowSize) % windowSize;
        if (envelopeHistory[idx] > maxRecent) {
            maxRecent = envelopeHistory[idx];
        }
    }
    
    float adaptiveThresh = maxRecent * 0.15f;
    
    if (adaptiveThresh < 2.0f) adaptiveThresh = 2.0f;
    if (adaptiveThresh > 20.0f) adaptiveThresh = 20.0f;
    
    return adaptiveThresh;
}

void EnvelopeSystolicDetector::detect(int ppgSignal, float pressureSignal, unsigned long timestamp) {
    float env = envelopeDetector.update((float)ppgSignal);
    
    envelopeHistory[historyIdx] = env;
    pressureHistory[historyIdx] = pressureSignal;
    timeHistory[historyIdx] = timestamp;
    
    historyIdx = (historyIdx + 1) % windowSize;
    if (historyCount < windowSize) historyCount++;
    
    if (historyCount < windowSize || detectionMade) return;
    
    int flatWin = getAdaptiveFlatWindow();
    int riseWin = getAdaptiveRiseWindow();
    float envThreshold = getAdaptiveEnvelopeThreshold();
    
    bool flat = isEnvelopeFlat(flatWin);
    bool rising = isEnvelopeIncreasing(riseWin);
    
    int currIdx = (historyIdx - 1 + windowSize) % windowSize;
    
    if (!flat || !rising || envelopeHistory[currIdx] < envThreshold) {
        return;
    }
    
    constexpr float MIN_SLOPE = 0.1f;
    constexpr float MAX_DELTA = 40.0f;
    
    float slope = calculateSlope(riseWin);
    float systolic = pressureSignal;
    
    if (slope > MIN_SLOPE) {
        float intercept = envelopeHistory[currIdx] - slope * pressureHistory[currIdx];
        float est = -intercept / slope;
        if (est > pressureSignal && est < pressureSignal + MAX_DELTA) {
            systolic = est;
        }
    }
    
    recordDetection(systolic, timestamp);
    detectionMade = true;
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