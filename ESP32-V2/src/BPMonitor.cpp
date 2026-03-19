#include "BPMonitor.h"
#include <string.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <algorithm>

BPMonitor::BPMonitor()
    : state(IDLE), systolic(0), maxPressure(0), startTime(0),
      detectorCount(0), baselineBeatCount(0), hrCalculated(false), lastBeatDetectedPressure(999),
      pressureHistoryIdx(0), pressureHistoryCount(0), lastPressureDerivative(0), lastPeakTime(0), startInflating(false)
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

void BPMonitor::setMotorController(MotorController* motorController) { motor = motorController; }

void BPMonitor::setMAPDetector(MAPDetector* detector) { mapDetector = detector; }

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
    lastBeatDetectedPressure = 999;
    startInflating = false;
    memset(baselineBeats, 0, sizeof(baselineBeats));
    memset(pressureHistory, 0, sizeof(pressureHistory));
    
    mapDetector->reset();

    for (int i = 0; i < detectorCount; i++)
    {
        detectors[i]->reset();
    }
}

float BPMonitor::getMAP() { return mapDetector->getMAP(); }
MotorController* BPMonitor::getMotorController() { return motor; }
MAPDetector* BPMonitor::getMAPDetector() { return mapDetector; }

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
    if (pressure > maxPressure) maxPressure = pressure;

    switch (state)
    {
    case IDLE:
    {
        if (pressure > BP_MIN_IDLE_PRESSURE || startInflating)
        {
            state = INFLATING;
            mapDetector->reset();
            hrCalculated = false;
            baselineBeatCount = 0;
            maxPressure = pressure;
            motor->startInflation(); 
        }
        break;
    }

    case INFLATING:
    { 
        // Use a simple beat detection algorithm to detect when last beat was detected
        if (ppgSignal > 100) lastBeatDetectedPressure = pressure;

        if (pressure < (maxPressure - PRESSURE_DROP_THRESHOLD) // For omron, if we notice a pressure drop start measuring
            || pressure > 180 // For our motor, based on highest pressure it should go
            || (pressure - lastBeatDetectedPressure) > 180 // For our motor, based on how high it should go after last detection. This may interfere with Omron Testing
    ){
            state = MEASURING;
            startTime = currentTime;
            motor->startDeflation();
        }
        break;
    }

    case MEASURING:
    {
        if (detectPressureOscillation(pressure, currentTime))
        {
            if (baselineBeatCount >= 2) 
            {
                calculateBaselineHeartRate(currentTime);
            }
        }

        mapDetector->addSample(pressure, currentTime);

        unsigned long recentBeat = baselineBeats[baselineBeatCount-1];
        for (int i = 0; i < detectorCount; i++)
        {
            detectors[i]->detect(ppgSignal, pressure, currentTime, recentBeat);
        }

        // Controlled deflation - faster after 80 mmHg
        if (ppgSignal > 1000 || pressure < 70) {
            motor->openSolenoid();
        }

        if (pressure < BP_MIN_IDLE_PRESSURE)
        {
            state = COMPLETE;

        }

        break;
    }


    case COMPLETE:
    {
        mapDetector->detectMAP();
        // Show results, maybe loop back to IDLE?
        break;
    }
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

void BPMonitor::startInflation() {startInflating = true;};

void BPMonitor::holdPressure() {
    // Add hold pressure code here :)
    return;
};

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
    struct DetectorReading {
        float pressure;
        float weight;
    };

    static DetectorReading readings[MAX_DETECTORS * MAX_READINGS_PER_DETECTOR];
    static int detectorStartIndices[MAX_DETECTORS];

    BPResult result{};
    result.systolic = 0.0f;
    result.confidence = 0.0f;
    result.confidenceIntervalLow = 0.0f;
    result.confidenceIntervalHigh = 0.0f;
    result.agreementCount = 0;
    result.totalDetectors = detectorCount;


    if (detectorCount <= 0)
    {
        return result;
    }

    const int MAX_TOTAL_READINGS = MAX_DETECTORS * MAX_READINGS_PER_DETECTOR;
    int readingCount = 0;

    // --- 1. Collect hypotheses ---
    for (int i = 0; i < detectorCount; i++)
    {
        if (readingCount >= MAX_TOTAL_READINGS)
        {
            
            break;
        }

        detectorStartIndices[i] = readingCount;

        if (detectors[i] == nullptr)
        {

            continue;
        }

        DetectionRecord top[MAX_READINGS_PER_DETECTOR];
        int actualCount = detectors[i]->softmaxNormalize(
            top,
            MAX_READINGS_PER_DETECTOR,
            0.1f
        );


        for (int j = 0; j < actualCount; j++)
        {
            if (readingCount >= MAX_TOTAL_READINGS)
            {
                break;
            }

            float p = top[j].pressure;
            float w = top[j].confidence;

            if (p <= 0.0f || p >= 185.0f)
            {

                continue;
            }

            if (!isfinite(p) || !isfinite(w))
            {

                continue;
            }

            readings[readingCount].pressure = p;
            readings[readingCount].weight = w;
            readingCount++;
        }
    }



    if (readingCount <= 0)
    {

        return result;
    }

    // --- 2. Normalize per-detector contribution ---
    for (int i = 0; i < detectorCount; i++)
    {
        int startIdx = detectorStartIndices[i];
        int endIdx = (i + 1 < detectorCount) ? detectorStartIndices[i + 1] : readingCount;

        if (startIdx >= readingCount)
            continue;

        float detectorWeightSum = 0.0f;

        for (int j = startIdx; j < endIdx; j++)
            detectorWeightSum += readings[j].weight;


        if (detectorWeightSum > 0.000001f)
        {
            for (int j = startIdx; j < endIdx; j++)
                readings[j].weight /= detectorWeightSum;
        }
    }

    // --- 3. Weighted mean ---
    float weightedSum = 0.0f;
    float totalWeight = 0.0f;

    for (int i = 0; i < readingCount; i++)
    {
        weightedSum += readings[i].pressure * readings[i].weight;
        totalWeight += readings[i].weight;
    }


    if (totalWeight <= 0.000001f)
    {

        return result;
    }

    result.systolic = weightedSum / totalWeight;

    if (!isfinite(result.systolic))
    {

        return result;
    }

    // --- 4. Weighted standard deviation ---
    float weightedVariance = 0.0f;

    for (int i = 0; i < readingCount; i++)
    {
        float diff = readings[i].pressure - result.systolic;
        weightedVariance += readings[i].weight * diff * diff;
    }

    float weightedStdDev = 0.0f;

    if (totalWeight > 0.000001f)
        weightedStdDev = sqrtf(weightedVariance / totalWeight);

    if (!isfinite(weightedStdDev))
        weightedStdDev = 0.0f;



    // --- 5. Agreement within ±1 std dev ---
    int agreementCount = 0;
    float agreementWeightSum = 0.0f;

    for (int i = 0; i < readingCount; i++)
    {
        float diff = readings[i].pressure - result.systolic;

        if (fabs(diff) <= weightedStdDev)
        {
            agreementCount++;
            agreementWeightSum += readings[i].weight;
        }
    }

    result.agreementCount = agreementCount;

    float agreementRatio = 0.0f;
    if (readingCount > 0)
        agreementRatio = (float)agreementCount / (float)readingCount;

    float avgAgreementWeight = 0.0f;
    if (agreementCount > 0)
        avgAgreementWeight = agreementWeightSum / agreementCount;

    float sampleSizeFactor = 1.0f - expf(-(float)detectorCount / 20.0f);

    result.confidence = agreementRatio * avgAgreementWeight * sampleSizeFactor;


    if (!isfinite(result.confidence) || result.confidence < 0.0f)
        result.confidence = 0.0f;

    if (result.confidence > 1.0f)
        result.confidence = 1.0f;

    // --- 6. Confidence interval ---
    float effectiveN = totalWeight;
    float standardError = 0.0f;

    if (effectiveN > 0.000001f)
        standardError = weightedStdDev / sqrtf(effectiveN);

    float ciMultiplier = 1.96f * (2.0f - result.confidence);

    result.confidenceIntervalLow  = result.systolic - (ciMultiplier * standardError);
    result.confidenceIntervalHigh = result.systolic + (ciMultiplier * standardError);



    if (!isfinite(result.confidenceIntervalLow) ||
        !isfinite(result.confidenceIntervalHigh))
    {

        result.confidenceIntervalLow  = result.systolic - 2.0f;
        result.confidenceIntervalHigh = result.systolic + 2.0f;
    }

    // --- 7. Enforce minimum ±2 mmHg CI ---
    float halfWidth =
        (result.confidenceIntervalHigh - result.confidenceIntervalLow) / 2.0f;

    if (halfWidth < 2.0f)
    {
        result.confidenceIntervalLow  = result.systolic - 2.0f;
        result.confidenceIntervalHigh = result.systolic + 2.0f;
    }

    return result;
}