#include "BPMonitor.h"
#include <string.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <algorithm>

BPMonitor::BPMonitor()
    : state(IDLE), systolic(0), maxPressure(0), startTime(0),
      detectorCount(0), baselineBeatCount(0), hrCalculated(false),
      pressureHistoryIdx(0), pressureHistoryCount(0), lastPressureDerivative(0), lastPeakTime(0)
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
    lastPeakTime = 0;
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
    
    if (pressureHistoryCount < PRESSURE_HISTORY_SIZE)
    {
        return false;
    }
    
    // Get indices
    int currentIdx = (pressureHistoryIdx - 1 + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
    int prevIdx = (currentIdx - 1 + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
    int prevPrevIdx = (prevIdx - 1 + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
    
    float prev = pressureHistory[prevIdx];
    float prevPrev = pressureHistory[prevPrevIdx];
    
    bool isPeak = (prev > prevPrev) && (prev > currentPressure);
    
    if (isPeak)
    {
        // CRITICAL FIX: Check interval from LAST DETECTED PEAK, not last accepted beat
        if (baselineBeatCount == 0)
        {
            // First beat - always accept
            baselineBeats[baselineBeatCount++] = timestamp;
            lastPeakTime = timestamp;  // Track this separately
            return true;
        }
        else
        {
            // Measure from the LAST PEAK (accepted or not)
            unsigned long interval = timestamp - lastPeakTime;
            lastPeakTime = timestamp;  // Update for next comparison
            
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
    
    // Calculate average interval, skipping invalid ones
    unsigned long totalInterval = 0;
    int validIntervalCount = 0;
    
    for (int i = 1; i < baselineBeatCount; i++)
    {
        unsigned long interval = baselineBeats[i] - baselineBeats[i-1];
        
        // Skip obviously wrong intervals
        if (interval >= MIN_BEAT_INTERVALS_MS && interval <= MAX_BEAT_INTERVALS_MS)
        {
            totalInterval += interval;
            validIntervalCount++;
        }
    }
    
    // Need at least one valid interval to calculate
    if (validIntervalCount == 0)
    {
        return; // Can't calculate baseline with no valid intervals
    }
    
    float avgInterval = (float)totalInterval / validIntervalCount;
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

const unsigned long* BPMonitor::getBaselineBeats(int& outCount) const
{
    outCount = baselineBeatCount;
    return baselineBeats;
}


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
    BPResult result{};
    result.systolic = 0;
    result.confidence = 0;
    result.confidenceIntervalLow = 0;
    result.confidenceIntervalHigh = 0;
    result.agreementCount = 0;
    result.totalDetectors = 0;

    // Temporary structure for all hypotheses from all detectors
    struct DetectorReading {
        float pressure;
        float weight;  // softmax-normalized confidence
    };

    DetectorReading readings[MAX_DETECTORS * MAX_READINGS_PER_DETECTOR];
    int readingCount = 0;

    // --- 1. Collect all normalized hypotheses from detectors ---
    for (int i = 0; i < detectorCount; i++)
    {
        if (readingCount >= MAX_DETECTORS * MAX_READINGS_PER_DETECTOR) break;
        
        DetectionRecord top[MAX_READINGS_PER_DETECTOR];
        int actualCount = detectors[i]->softmaxNormalize(top, MAX_READINGS_PER_DETECTOR, 0.1f); // T=0.1 for sharpening

        for (int j = 0; j < actualCount; j++)
        {
            if (top[j].pressure <= 40 || top[j].pressure >= 185) continue;

            readings[readingCount].pressure = top[j].pressure;
            readings[readingCount].weight = top[j].confidence;  // already normalized
            readingCount++;
        }
    }

    result.totalDetectors = detectorCount;

    if (readingCount == 0)
        return result;  // no valid readings

    // --- 2. Compute weighted mean (systolic) ---
    float weightedSum = 0;
    float totalWeight = 0;
    for (int i = 0; i < readingCount; i++)
    {
        weightedSum += readings[i].pressure * readings[i].weight;
        totalWeight += readings[i].weight;
    }
    result.systolic = weightedSum / totalWeight;

    // --- 3. Compute weighted standard deviation ---
    float weightedVariance = 0;
    for (int i = 0; i < readingCount; i++)
    {
        float diff = readings[i].pressure - result.systolic;
        weightedVariance += readings[i].weight * diff * diff;
    }
    float weightedStdDev = sqrt(weightedVariance / totalWeight);

    // --- 4. Count agreement (within ±1 std dev) ---
    int agreementCount = 0;
    float agreementWeightSum = 0;
    for (int i = 0; i < readingCount; i++)
    {
        float diff = readings[i].pressure - result.systolic;
        if (diff >= -weightedStdDev && diff <= weightedStdDev)
        {
            agreementCount++;
            agreementWeightSum += readings[i].weight;
        }
    }
    result.agreementCount = agreementCount;

    // --- 5. Compute ensemble confidence ---
    float agreementRatio = (float)agreementCount / readingCount;
    float avgAgreementWeight = agreementWeightSum / agreementCount;

    // Sample size factor saturates around 60 detectors
    float sampleSizeFactor = 1.0f - expf(-detectorCount / 20.0f);

    result.confidence = agreementRatio * avgAgreementWeight * sampleSizeFactor;
    if (result.confidence > 1.0f) result.confidence = 1.0f;

    // --- 6. Compute 95% confidence interval ---
    float effectiveN = totalWeight;  // sum of softmax weights
    float standardError = weightedStdDev / sqrtf(effectiveN);

    float ciMultiplier = 1.96f * (2.0f - result.confidence);  // widen CI for low confidence
    result.confidenceIntervalLow = result.systolic - (ciMultiplier * standardError);
    result.confidenceIntervalHigh = result.systolic + (ciMultiplier * standardError);

    // --- 7. Enforce minimum ±2 mmHg CI ---
    float halfWidth = (result.confidenceIntervalHigh - result.confidenceIntervalLow) / 2.0f;
    if (halfWidth < 2.0f)
    {
        result.confidenceIntervalLow = result.systolic - 2.0f;
        result.confidenceIntervalHigh = result.systolic + 2.0f;
    }

    return result;
} 