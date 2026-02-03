#include "BPMonitor.h"

BPMonitor::BPMonitor()
{
    reset();
}

void BPMonitor::reset()
{
    state = IDLE;
    systolic = -1;
    maxPressure = 0;
    startTime = 0;
    detectorCount = 0;
    externalFilter = nullptr;

    // Reset baseline HR tracking
    baselineBeatCount = 0;
    hrCalculated = false;
    lastBPMMeasurement = 0;
    baselineHR.isValid = false;
    baselineHR.minInterval = 0;
    baselineHR.maxInterval = 0;

    mapDetector.reset();

    // Reset circular pressure history buffer
    pressureHistoryIdx = 0;
    pressureHistoryCount = 0;
    lastPressureDerivative = 0;
    lastPeakTime = 0;
    
    for (int i = 0; i < PRESSURE_HISTORY_SIZE; i++)
        pressureHistory[i] = 0;
    
    for (int i = 0; i < MAX_BASELINE_BEATS; i++)
        baselineBeats[i] = 0;
}

void BPMonitor::addDetector(SystolicDetector *detector)
{
    if (detectorCount < MAX_DETECTORS)
        detectors[detectorCount++] = detector;
}

void BPMonitor::setFilter(PPGBandpassFilter* filter)
{
    externalFilter = filter;
}

int BPMonitor::getDetectorCount() const
{
    return detectorCount;
}

SystolicDetector* BPMonitor::getDetector(int index) const
{
    if (index < 0 || index >= detectorCount)
        return nullptr;
    return detectors[index];
}

void BPMonitor::update(const BPMeasurement& m)
{
    // Track max pressure reached
    if (m.pressure > maxPressure)
        maxPressure = m.pressure;

    // Store pressure in circular history buffer
    pressureHistory[pressureHistoryIdx] = m.pressure;
    pressureHistoryIdx = (pressureHistoryIdx + 1) % PRESSURE_HISTORY_SIZE;
    if (pressureHistoryCount < PRESSURE_HISTORY_SIZE)
        pressureHistoryCount++;

    // Oscillation signal for MAP detection
    static float lastPressure = 0;
    float osc = m.pressure - lastPressure;
    lastPressure = m.pressure;

    mapDetector.addSample(m.pressure, osc, m.timestamp);

    switch (state)
    {
        case IDLE:
            if (m.pressure > 30)
            {
                state = INFLATING;
                startTime = m.timestamp;
            }
            break;

        case INFLATING:
            // Detect baseline heart rate from pressure oscillations
            if (detectPressureOscillation(m.pressure, m.timestamp))
            {
                if (baselineBeatCount < MAX_BASELINE_BEATS)
                {
                    baselineBeats[baselineBeatCount++] = m.timestamp;
                }
            }
            
            // Once pressure stops increasing, begin measuring
            if (m.pressure >= maxPressure - 1)
            {
                state = MEASURING;
                calculateBaselineHeartRate(m.timestamp);
                
                // Set heart rate range for all detectors
                for (int i = 0; i < detectorCount; i++)
                {
                    detectors[i]->setHeartRateRange(baselineHR);
                }
            }
            break;

        case MEASURING:
            for (int i = 0; i < detectorCount; i++)
            {
                detectors[i]->detect(m.ppgSignal, m.pressure, m.timestamp);
            }

            // When cuff deflates enough, finalize results
            if (m.pressure < 40)
            {
                state = COMPLETE;
                mapDetector.detectMAP();
                systolic = mapDetector.getSystolic();
            }
            break;

        case COMPLETE:
            break;
    }
}

BPStatus BPMonitor::getStatus() const
{
    BPStatus s;
    s.state = state;

    // Most recent pressure from history buffer
    if (pressureHistoryCount > 0)
    {
        int lastIdx = (pressureHistoryIdx - 1 + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
        s.currentPressure = pressureHistory[lastIdx];
    }
    else
    {
        s.currentPressure = 0;
    }

    s.maxPressure = maxPressure;

    switch (state)
    {
        case IDLE:
            s.statusMessage = "Idle";
            s.detailMessage = "Waiting to start";
            break;
        case INFLATING:
            s.statusMessage = "Inflating";
            s.detailMessage = "Pump running";
            break;
        case MEASURING:
            s.statusMessage = "Measuring";
            s.detailMessage = "Hold still";
            break;
        case COMPLETE:
            s.statusMessage = "Complete";
            s.detailMessage = "Reading ready";
            break;
    }

    return s;
}

float BPMonitor::getSystolic() const
{
    return systolic;
}

BPState BPMonitor::getState() const
{
    return state;
}

float BPMonitor::getMAP()
{
    return mapDetector.getMAP();
}

MAPDetector* BPMonitor::getMAPDetector()
{
    return &mapDetector;
}

BPResult BPMonitor::getEnsembleResult() const
{
    BPResult r;
    r.systolic = -1;
    r.confidence = 0;
    r.confidenceIntervalLow = 0;
    r.confidenceIntervalHigh = 0;
    r.agreementCount = 0;
    r.totalDetectors = detectorCount;

    int validCount = 0;
    float sum = 0;

    for (int i = 0; i < detectorCount; i++)
    {
        float val = detectors[i]->getSystolic();
        if (val > 0)
        {
            sum += val;
            validCount++;
        }
    }

    if (validCount > 0)
    {
        r.systolic = sum / validCount;
        r.confidence = (float)validCount / detectorCount;
        r.agreementCount = validCount;
    }

    return r;
}

float BPMonitor::getBestSystolic(float* outConfidence) const
{
    BPResult result = getEnsembleResult();
    if (outConfidence != nullptr)
    {
        *outConfidence = result.confidence;
    }
    return result.systolic;
}

HeartRateRange BPMonitor::getBaselineHeartRate() const
{
    return baselineHR;
}

float BPMonitor::getBaselineBPM() const
{
    if (!baselineHR.isValid || baselineHR.minInterval == 0)
        return 0;
    
    // Calculate average BPM from average interval
    unsigned long avgInterval = (baselineHR.minInterval + baselineHR.maxInterval) / 2;
    return 60000.0 / avgInterval;
}

const unsigned long* BPMonitor::getBaselineBeats(int& outCount) const
{
    outCount = baselineBeatCount;
    return baselineBeats;
}

void BPMonitor::calculateBaselineHeartRate(unsigned long currentTime)
{
    if (baselineBeatCount < 2)
    {
        hrCalculated = false;
        baselineHR.isValid = false;
        return;
    }

    // Calculate intervals between beats
    unsigned long minInterval = 999999;
    unsigned long maxInterval = 0;
    unsigned long sumIntervals = 0;
    int intervalCount = 0;

    for (int i = 1; i < baselineBeatCount; i++)
    {
        unsigned long interval = baselineBeats[i] - baselineBeats[i-1];
        if (interval > 0)
        {
            sumIntervals += interval;
            intervalCount++;
            if (interval < minInterval) minInterval = interval;
            if (interval > maxInterval) maxInterval = interval;
        }
    }

    if (intervalCount > 0)
    {
        baselineHR.minInterval = minInterval;
        baselineHR.maxInterval = maxInterval;
        baselineHR.isValid = true;
        hrCalculated = true;
    }
    else
    {
        baselineHR.isValid = false;
        hrCalculated = false;
    }
}

bool BPMonitor::detectPressureOscillation(float currentPressure, unsigned long timestamp)
{
    // Need at least 2 samples to calculate derivative
    if (pressureHistoryCount < 2)
        return false;

    // Get previous pressure
    int prevIdx = (pressureHistoryIdx - 2 + PRESSURE_HISTORY_SIZE) % PRESSURE_HISTORY_SIZE;
    float prevPressure = pressureHistory[prevIdx];
    
    // Calculate derivative (rate of change)
    float derivative = currentPressure - prevPressure;
    
    // Detect peak (derivative changes from positive to negative)
    bool peakDetected = false;
    if (lastPressureDerivative > 0 && derivative < 0)
    {
        // Check minimum time between peaks (avoid false detections)
        if (lastPeakTime == 0 || (timestamp - lastPeakTime) > 300)  // At least 300ms
        {
            peakDetected = true;
            lastPeakTime = timestamp;
        }
    }
    
    lastPressureDerivative = derivative;
    return peakDetected;
}