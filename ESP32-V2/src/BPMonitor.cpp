#include "BPMonitor.h"
#include <string.h>
#include <stdio.h>
#include <cmath>
#include <iostream>

BPMonitor::BPMonitor()
    : state(IDLE), systolic(0), maxPressure(0), startTime(0),
      detectorCount(0), baselineBeatCount(0), hrCalculated(false),
      pressureHistoryIdx(0), pressureHistoryCount(0), lastPressureDerivative(0)
{
    for (int i = 0; i < MAX_DETECTORS; i++)
    {
        detectors[i] = nullptr;
    }
    memset(baselineBeats, 0, sizeof(baselineBeats));
    memset(pressureHistory, 0, sizeof(pressureHistory));
}

void BPMonitor::addDetector(SystolicDetector *detector)
{
    if (detectorCount < MAX_DETECTORS)
    {
        detectors[detectorCount++] = detector;
    }
}

void BPMonitor::setFilter(PPGBandpassFilter* filter) { externalFilter = filter; }

void BPMonitor::reset()
{
    state = IDLE;
    systolic = 0;
    maxPressure = 0;
    startTime = 0;
    baselineBeatCount = 0;
    hrCalculated = false;
    pressureHistoryIdx = 0;
    pressureHistoryCount = 0;
    lastPressureDerivative = 0;
    memset(baselineBeats, 0, sizeof(baselineBeats));
    memset(pressureHistory, 0, sizeof(pressureHistory));
    
    mapDetector.reset();

    for (int i = 0; i < detectorCount; i++)
    {
        detectors[i]->reset();
    }
}

float BPMonitor::getMAP() { return mapDetector.getMAP(); }
MAPDetector* BPMonitor::getMAPDetector() { return &mapDetector; }

bool BPMonitor::detectPressureOscillation(float currentPressure, unsigned long timestamp)
{
    // Add current pressure to history
    pressureHistory[pressureHistoryIdx] = currentPressure;
    pressureHistoryIdx = (pressureHistoryIdx + 1) % PRESSURE_HISTORY_SIZE;
    if (pressureHistoryCount < PRESSURE_HISTORY_SIZE)
    {
        pressureHistoryCount++;
    }
    
    // Need full buffer to detect oscillations
    if (pressureHistoryCount < PRESSURE_HISTORY_SIZE)
    {
        return false;
    }
    
    // Remove linear trend by looking at local deviations
    // Calculate mean of the window
    float mean = 0;
    for (int i = 0; i < PRESSURE_HISTORY_SIZE; i++)
    {
        mean += pressureHistory[i];
    }
    mean /= PRESSURE_HISTORY_SIZE;
    
    // Get detrended value (deviation from mean)
    int currentIdx = (pressureHistoryIdx - 1 + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
    float currentDetrended = pressureHistory[currentIdx] - mean;
    
    // Get previous detrended value
    int prevIdx = (currentIdx - 1 + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
    float prevDetrended = pressureHistory[prevIdx] - mean;
    
    // Detect peak: was positive, now negative (zero crossing from above)
    bool isPeak = (prevDetrended > 0.1f && currentDetrended < 0);
    
    if (isPeak)
    {
        // Record beat with minimum interval (300-1500ms = 40-200 BPM)
        if (baselineBeatCount == 0)
        {
            // First beat
            if (baselineBeatCount < MAX_BASELINE_BEATS)
            {
                baselineBeats[baselineBeatCount++] = timestamp;
                return true;
            }
        }
        else
        {
            unsigned long interval = timestamp - baselineBeats[baselineBeatCount-1];
            
            // Only accept beats in physiological range (40-200 BPM = 300-1500ms)
            if (interval >= MIN_BEAT_INTERVALS_MS && interval <= MAX_BEAT_INTERVALS_MS)
            {
                if (baselineBeatCount < MAX_BASELINE_BEATS)
                {
                    baselineBeats[baselineBeatCount++] = timestamp;
                    return true;
                }
            }
        }
    }
    
    return false;
}

void BPMonitor::calculateBaselineHeartRate(unsigned long currentTime)
{
    // Calculate average interval
    unsigned long totalInterval = 0;
    for (int i = 1; i < baselineBeatCount; i++)
    {
        totalInterval += baselineBeats[i] - baselineBeats[i-1];
    }
    
    float avgInterval = (float)totalInterval / (baselineBeatCount - 1);
    float avgBPM = 60000.0f / avgInterval;
    
    // Account for delay
    const float DELAY_MS = 75.0f;
    
    // Set range for all detectors (+/-40 BPM tolerance)
    baselineHR.setFromBPM(avgBPM, 40.0f);
    
    // Adjust minimum interval to account for delay
    // (pressure peak happens first, PPG follows ~75ms later)
    if (baselineHR.minInterval > DELAY_MS) 
        baselineHR.minInterval -= (unsigned long)DELAY_MS;
    
    for (int i = 0; i < detectorCount; i++)
    {
        detectors[i]->setHeartRateRange(baselineHR);
    }
    
    // Configure external filter
    if (externalFilter != nullptr && (currentTime - lastBPMMeasurement) > 4000) {
        lastBPMMeasurement = currentTime;
        externalFilter->setHeartRateRange(avgBPM, 40.0f);
    }
    
    hrCalculated = true;
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
        if (pressure > BP_MIN_IDLE_PRESSURE)
        {
            state = INFLATING;

            mapDetector.reset();
            hrCalculated = false;
            baselineBeatCount = 0;
            maxPressure = pressure;
        }
        break;

    case INFLATING:
        if (pressure >= BP_START_PRESSURE)
        {
            if (pressure < (maxPressure - PRESSURE_DROP_THRESHOLD)) {
                state = MEASURING;
                startTime = currentTime;
            }
        }
        break;

    case MEASURING:
        if (detectPressureOscillation(pressure, currentTime))
        {
            if (baselineBeatCount >= 2)
            {
                calculateBaselineHeartRate(currentTime);
            }
        }

        mapDetector.addSample(pressure, currentTime);

        for (int i = 0; i < detectorCount; i++)
        {
            detectors[i]->detect(ppgSignal, pressure, currentTime);
        }

        if (pressure < BP_MIN_IDLE_PRESSURE)
        {
            state = COMPLETE;
        }

        break;


    case COMPLETE:
        mapDetector.detectMAP();
        // Show results, maybe loop back to IDLE?
        break;
    }
}

BPStatus BPMonitor::getStatus() const
{
    BPStatus status;
    status.state = state;
    status.currentPressure = maxPressure;
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

float BPMonitor::getSystolic() const { return systolic; }
BPState BPMonitor::getState() const { return state; }
int BPMonitor::getDetectorCount() const { return detectorCount; }

SystolicDetector* BPMonitor::getDetector(int index) const
{
    if (index >= 0 && index < detectorCount)
    {
        return detectors[index];
    }
    return nullptr;
}

// Get best reading based on confidence across all detectors
float BPMonitor::getBestSystolic(float* outConfidence) const
{
    DetectionRecord bestOverall;
    bestOverall.pressure = 0;
    bestOverall.confidence = 0;
    
    for (int i = 0; i < detectorCount; i++)
    {
        DetectionRecord best = detectors[i]->getBestDetection();
        
        if (best.confidence > bestOverall.confidence)
        {
            bestOverall = best;
        }
    }
    
    if (outConfidence)
    {
        *outConfidence = bestOverall.confidence;
    }
    
    return bestOverall.pressure;
}

HeartRateRange BPMonitor::getBaselineHeartRate() const { return baselineHR; }

float BPMonitor::getBaselineBPM() const
{
    if (!hrCalculated || baselineBeatCount < 2) return 0.0f;
    
    unsigned long totalInterval = 0;
    for (int i = 1; i < baselineBeatCount; i++)
    {
        totalInterval += baselineBeats[i] - baselineBeats[i-1];
    }
    
    float avgInterval = (float)totalInterval / (baselineBeatCount - 1);
    return 60000.0f / avgInterval;
}

BPResult BPMonitor::getEnsembleResult() const
{
    BPResult result;
    result.systolic = 0;
    result.confidence = 0;
    result.confidenceIntervalLow = 0;
    result.confidenceIntervalHigh = 0;
    result.agreementCount = 0;
    result.totalDetectors = 0;
    
    // Collect all valid detections
    struct DetectorReading {
        float pressure;
        float confidence;
    };
    
    DetectorReading readings[MAX_DETECTORS];
    int readingCount = 0;
    
    for (int i = 0; i < detectorCount; i++)
    {
        DetectionRecord best = detectors[i]->getBestDetection();
        if (best.pressure > 0 && best.confidence > 0.3f)  // Minimum confidence threshold
        {
            readings[readingCount].pressure = best.pressure;
            readings[readingCount].confidence = best.confidence;
            readingCount++;
        }
    }
    
    result.totalDetectors = readingCount;
    
    if (readingCount == 0)
    {
        return result;
    }
    
    // Calculate confidence-weighted mean
    float weightedSum = 0;
    float totalWeight = 0;
    
    for (int i = 0; i < readingCount; i++)
    {
        float weight = readings[i].confidence * readings[i].confidence;  // Square for emphasis
        weightedSum += readings[i].pressure * weight;
        totalWeight += weight;
    }
    
    result.systolic = weightedSum / totalWeight;
    
    // Calculate weighted standard deviation
    float weightedVariance = 0;
    for (int i = 0; i < readingCount; i++)
    {
        float weight = readings[i].confidence * readings[i].confidence;
        float diff = readings[i].pressure - result.systolic;
        weightedVariance += weight * diff * diff;
    }
    float weightedStdDev = sqrt(weightedVariance / totalWeight);
    
    // Count detectors within ±1 std dev (this is our "agreement")
    int agreementCount = 0;
    float agreementWeightSum = 0;
    
    for (int i = 0; i < readingCount; i++)
    {
        float diff = readings[i].pressure - result.systolic;
        if (diff >= -weightedStdDev && diff <= weightedStdDev)
        {
            agreementCount++;
            agreementWeightSum += readings[i].confidence;
        }
    }
    
    result.agreementCount = agreementCount;
    
    // Calculate ensemble confidence based on:
    // 1. Agreement ratio (what fraction agree within 1 std dev)
    // 2. Average confidence of agreeing detectors
    // 3. Sample size (more detectors = more confidence)
    
    float agreementRatio = (float)agreementCount / readingCount;
    float avgAgreementConfidence = agreementWeightSum / agreementCount;
    float sampleSizeFactor = 1.0f - exp(-readingCount / 20.0f);  // Saturates at ~60 detectors
    
    result.confidence = agreementRatio * avgAgreementConfidence * sampleSizeFactor;
    
    // Cap at 1.0
    if (result.confidence > 1.0f) result.confidence = 1.0f;
    
    // Calculate 95% confidence interval (±1.96 standard errors)
    // Standard error = std dev / sqrt(effective N)
    float effectiveN = totalWeight;  // Use sum of squared weights as effective sample size
    float standardError = weightedStdDev / sqrt(effectiveN);
    
    // 95% CI: ±1.96 SE, but widen if low confidence
    float ciMultiplier = 1.96f * (2.0f - result.confidence);  // Wider CI for low confidence
    
    result.confidenceIntervalLow = result.systolic - (ciMultiplier * standardError);
    result.confidenceIntervalHigh = result.systolic + (ciMultiplier * standardError);
    
    // Ensure CI is at least ±2 mmHg (measurement precision limit)
    float minHalfWidth = 2.0f;
    float currentHalfWidth = (result.confidenceIntervalHigh - result.confidenceIntervalLow) / 2.0f;
    if (currentHalfWidth < minHalfWidth)
    {
        result.confidenceIntervalLow = result.systolic - minHalfWidth;
        result.confidenceIntervalHigh = result.systolic + minHalfWidth;
    }
    
    return result;
}