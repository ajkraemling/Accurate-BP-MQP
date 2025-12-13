#include "MAPDetector.h"
#include "filters.h"
#include <string.h>
#include <math.h>
#include <algorithm>
#include <config.h>

// Band-pass filter for oscillometric component of cuff pressure
static PPGBandpassFilter g_oscFilter(1000.0f / SAMPLE_RATE_MS);

// Cuff pressure trend tracker (slow-moving average for DC component)
class CuffTrendTracker {
private:
    static const int WINDOW_SIZE = 50;  // ~1 second at 50Hz
    float buffer[WINDOW_SIZE];
    int idx;
    int count;
    bool initialized;
    
public:
    CuffTrendTracker() : idx(0), count(0), initialized(false) {
        memset(buffer, 0, sizeof(buffer));
    }
    
    void reset() {
        idx = 0;
        count = 0;
        initialized = false;
        memset(buffer, 0, sizeof(buffer));
    }
    
    void initialize(float initialValue) {
        for (int i = 0; i < WINDOW_SIZE; i++) {
            buffer[i] = initialValue;
        }
        idx = 0;
        count = WINDOW_SIZE;
        initialized = true;
    }
    
    float update(float value) {
        if (!initialized) {
            initialize(value);
            return value;
        }
        
        buffer[idx] = value;
        idx = (idx + 1) % WINDOW_SIZE;
        if (count < WINDOW_SIZE) count++;
        
        float sum = 0.0f;
        for (int i = 0; i < count; ++i) {
            sum += buffer[i];
        }
        return sum / (float)count;
    }
    
    bool isReady() const {
        return count >= WINDOW_SIZE;
    }
};

static CuffTrendTracker g_cuffTrend;

// Pulse integration buffer - stores samples between pulse starts
struct PulseBuffer {
    static const int MAX_SAMPLES = 100;  // Max samples per pulse (~2 seconds at 50Hz)
    float oscillations[MAX_SAMPLES];
    float pressures[MAX_SAMPLES];
    int count;
    
    PulseBuffer() : count(0) {}
    
    void reset() {
        count = 0;
    }
    
    void addSample(float osc, float pressure) {
        if (count < MAX_SAMPLES) {
            oscillations[count] = osc;
            pressures[count] = pressure;
            count++;
        }
    }
    
    // Integrate area above linear baseline connecting first and last samples
    float integrateEnergy() const {
        if (count < 3) return 0.0f;  // Need at least 3 points
        
        float startOsc = oscillations[0];
        float endOsc = oscillations[count - 1];
        
        // Calculate area above linear baseline
        float area = 0.0f;
        for (int i = 0; i < count; i++) {
            // Linear interpolation of baseline at this sample
            float t = (float)i / (float)(count - 1);
            float baseline = startOsc + t * (endOsc - startOsc);
            
            // Area contribution (trapezoidal rule)
            float heightAboveBaseline = oscillations[i] - baseline;
            if (heightAboveBaseline > 0.0f) {
                area += heightAboveBaseline;
            }
        }
        
        return area;
    }
    
    // Get average cuff pressure during this pulse
    float averagePressure() const {
        if (count == 0) return 0.0f;
        
        float sum = 0.0f;
        for (int i = 0; i < count; i++) {
            sum += pressures[i];
        }
        return sum / (float)count;
    }
};

MAPDetector::MAPDetector()
    : trendIdx(0),
      trendCount(0),
      lastPressure(0.0f),
      lastOscillation(0.0f),
      lastPeakTime(0),
      inPotentialPeak(false),
      peakCandidateValue(0.0f),
      peakCandidatePressure(0.0f),
      beatCount(0),
      mapPressure(-1.0f),
      systolicPressure(-1.0f),
      diastolicPressure(-1.0f),
      systolicRatio(0.55f),
      diastolicRatio(0.85f),
      minPeakAmplitude(0.5f),      // Lower threshold for integration method
      maxReasonableAmplitude(50.0f), // Higher ceiling (we're integrating, not peak detecting)
      wasRising(false),
      filtersSettled(false),
      filterSettleCount(0),
      initializationPhase(true),
      lastDerivative(0.0f),
      inPulse(false),
      pulseBuffer(nullptr)
{
    memset(trendBuffer, 0, sizeof(trendBuffer));
    memset(beats, 0, sizeof(beats));
    pulseBuffer = new PulseBuffer();
}

MAPDetector::~MAPDetector()
{
    if (pulseBuffer) {
        delete pulseBuffer;
        pulseBuffer = nullptr;
    }
}

void MAPDetector::reset()
{
    trendIdx = 0;
    trendCount = 0;
    lastPressure = 0.0f;
    lastOscillation = 0.0f;
    lastPeakTime = 0;
    inPotentialPeak = false;
    peakCandidateValue = 0.0f;
    peakCandidatePressure = 0.0f;
    beatCount = 0;
    mapPressure = -1.0f;
    systolicPressure = -1.0f;
    diastolicPressure = -1.0f;
    wasRising = false;
    filtersSettled = false;
    filterSettleCount = 0;
    initializationPhase = true;
    lastDerivative = 0.0f;
    inPulse = false;

    memset(trendBuffer, 0, sizeof(trendBuffer));
    memset(beats, 0, sizeof(beats));

    if (pulseBuffer) {
        pulseBuffer->reset();
    }

    g_oscFilter.reset();
    g_cuffTrend.reset();
}

void MAPDetector::setSystolicRatio(float r) { systolicRatio = r; }
void MAPDetector::setDiastolicRatio(float r) { diastolicRatio = r; }
void MAPDetector::setMinPeakAmplitude(float a) { minPeakAmplitude = a; }
void MAPDetector::setMaxReasonableAmplitude(float a) { maxReasonableAmplitude = a; }

void MAPDetector::addSample(float pressure, unsigned long timestamp)
{
    // === INITIALIZATION PHASE ===
    if (initializationPhase) {
        filterSettleCount++;
        
        // Feed filters but don't process
        g_oscFilter.filter(pressure);
        g_cuffTrend.update(pressure);
        
        // Wait for 100 samples (~2 seconds at 50Hz)
        if (filterSettleCount >= 100) {
            initializationPhase = false;
            filtersSettled = true;
        }
        
        lastOscillation = 0.0f;
        lastDerivative = 0.0f;
        return;
    }
    
    // === NORMAL PROCESSING ===
    
    // Track cuff pressure trend
    float cuffTrend = g_cuffTrend.update(pressure);
    
    // Band-pass filter to extract oscillations
    float osc = g_oscFilter.filter(pressure);
    
    // Calculate derivative (rate of change) to detect pulse starts
    float derivative = osc - lastOscillation;
    
    // Threshold for detecting pulse start: strong positive derivative
    // This indicates the rapid upstroke of a cardiac pulse
    const float DERIVATIVE_THRESHOLD = 0.15f;  // Tunable based on your signal
    
    // Refractory period check
    bool refractoryOK = (lastPeakTime == 0) || 
                        ((timestamp - lastPeakTime) >= MIN_BEAT_INTERVALS_MS);
    
    // Detect pulse start: derivative crosses threshold (rising edge detection)
    bool pulseStartDetected = false;
    if (derivative > DERIVATIVE_THRESHOLD && 
        lastDerivative <= DERIVATIVE_THRESHOLD && 
        refractoryOK) {
        pulseStartDetected = true;
    }
    
    // === STATE MACHINE ===
    
    if (pulseStartDetected) {
        // Process previous pulse if we collected data
        if (inPulse && pulseBuffer->count >= 3) {
            // Integrate energy above baseline
            float energy = pulseBuffer->integrateEnergy();
            float avgPressure = pulseBuffer->averagePressure();
            
            // Validate and record
            if (energy >= minPeakAmplitude && energy <= maxReasonableAmplitude) {
                recordBeat(avgPressure, energy, timestamp);
                lastPeakTime = timestamp;
            }
        }
        
        // Start new pulse
        pulseBuffer->reset();
        inPulse = true;
    }
    
    // Accumulate samples during pulse
    if (inPulse) {
        pulseBuffer->addSample(osc, cuffTrend);
    }
    
    // Update state
    lastOscillation = osc;
    lastDerivative = derivative;
    lastPressure = cuffTrend;
}

// Legacy functions kept for interface compatibility
float MAPDetector::extractTrend(float pressure)
{
    trendBuffer[trendIdx] = pressure;
    trendIdx = (trendIdx + 1) % TREND_WINDOW;
    if (trendCount < TREND_WINDOW) trendCount++;

    float sum = 0.0f;
    for (int i = 0; i < trendCount; ++i) sum += trendBuffer[i];
    return sum / (float)trendCount;
}

float MAPDetector::calcOscillation(float pressure, float trend)
{
    (void)trend;
    return pressure;
}

void MAPDetector::recordBeat(float pressure, float amplitude, unsigned long timestamp)
{
    if (beatCount >= MAX_BEATS) return;

    // Validate amplitude is within acceptable range
    if (amplitude < minPeakAmplitude || amplitude > maxReasonableAmplitude)
        return;

    beats[beatCount].cuffPressure = pressure;
    beats[beatCount].amplitude = amplitude;
    beats[beatCount].timestamp = timestamp;
    beatCount++;
}

int MAPDetector::getBeatCount() const { return beatCount; }

void MAPDetector::getBeatRecords(BeatRecord* out, int maxCount, int* actualCount) const
{
    int c = (beatCount < maxCount) ? beatCount : maxCount;
    if (c > 0 && out != nullptr)
    {
        memcpy(out, beats, c * sizeof(BeatRecord));
    }
    if (actualCount) *actualCount = c;
}

bool MAPDetector::detectMAP()
{
    if (beatCount < 8)
        return false;

    int validStart = 0;
    int validEnd = beatCount;

    // Find valid range
    for (int i = 0; i < beatCount; ++i)
    {
        if (beats[i].cuffPressure <= BP_START_PRESSURE &&
            beats[i].amplitude >= minPeakAmplitude &&
            beats[i].amplitude <= maxReasonableAmplitude)
        {
            validStart = i;
            break;
        }
    }

    for (int i = beatCount - 1; i >= validStart; --i)
    {
        if (beats[i].cuffPressure >= BP_MIN_IDLE_PRESSURE &&
            beats[i].amplitude >= minPeakAmplitude &&
            beats[i].amplitude <= maxReasonableAmplitude)
        {
            validEnd = i + 1;
            break;
        }
    }

    if ((validEnd - validStart) < 8)
        return false;

    // Find MAP
    mapPressure = findMAPFromBeats(validStart, validEnd);
    if (mapPressure < 40.0f || mapPressure > 180.0f)
        return false;

    // Find SBP/DBP
    float sbp = -1.0f, dbp = -1.0f;
    if (findSystolicDiastolic(sbp, dbp, validStart, validEnd))
    {
        // Physiological validation
        if (sbp <= mapPressure || dbp >= mapPressure)
            return false;
        
        float pulsePressure = sbp - dbp;
        if (pulsePressure < 20.0f || pulsePressure > 150.0f)
            return false;
        
        systolicPressure  = sbp;
        diastolicPressure = dbp;
        return true;
    }

    return false;
}

float MAPDetector::findMAPFromBeats(int startIdx, int endIdx) const
{
    if (startIdx >= endIdx) return -1.0f;

    int   maxIdx = startIdx;
    float maxAmp = beats[startIdx].amplitude;

    for (int i = startIdx + 1; i < endIdx; ++i)
    {
        if (beats[i].amplitude > maxAmp)
        {
            maxAmp = beats[i].amplitude;
            maxIdx = i;
        }
    }

    if (maxAmp < minPeakAmplitude * 1.5f)
        return -1.0f;

    return beats[maxIdx].cuffPressure;
}

bool MAPDetector::findSystolicDiastolic(float &sbp, float &dbp,
                                        int startIdx, int endIdx) const
{
    if (startIdx >= endIdx) return false;

    // Find MAP (max amplitude) in valid range
    int   maxIdx = startIdx;
    float maxAmp = beats[startIdx].amplitude;

    for (int i = startIdx + 1; i < endIdx; ++i)
    {
        if (beats[i].amplitude > maxAmp)
        {
            maxAmp = beats[i].amplitude;
            maxIdx = i;
        }
    }

    if (maxAmp <= minPeakAmplitude)
        return false;

    const float sysThresh = systolicRatio  * maxAmp;
    const float diaThresh = diastolicRatio * maxAmp;

    // --- Systolic: search left from MAP (higher pressure side) ---
    bool  foundSys    = false;
    float sysPressure = -1.0f;

    for (int i = maxIdx - 1; i >= startIdx; --i)
    {
        if (beats[i].amplitude <= sysThresh)
        {
            float a1 = beats[i].amplitude;
            float p1 = beats[i].cuffPressure;
            float a2 = beats[i+1].amplitude;
            float p2 = beats[i+1].cuffPressure;

            if (fabsf(a2 - a1) > 1e-6f)
            {
                float t = (sysThresh - a1) / (a2 - a1);
                sysPressure = p1 + t * (p2 - p1);
            }
            else
            {
                sysPressure = beats[i].cuffPressure;
            }

            foundSys = true;
            break;
        }
    }

    if (!foundSys)
    {
        sysPressure = beats[startIdx].cuffPressure;
        foundSys = true;
    }

    // --- Diastolic: search right from MAP (lower pressure side) ---
    bool  foundDia    = false;
    float diaPressure = -1.0f;

    for (int i = maxIdx + 1; i < endIdx; ++i)
    {
        if (beats[i].amplitude <= diaThresh)
        {
            float a1 = beats[i-1].amplitude;
            float p1 = beats[i-1].cuffPressure;
            float a2 = beats[i].amplitude;
            float p2 = beats[i].cuffPressure;

            float denom = (a1 - a2);
            if (fabsf(denom) > 1e-6f)
            {
                float u = (diaThresh - a2) / denom;
                diaPressure = p2 + u * (p1 - p2);
            }
            else
            {
                diaPressure = beats[i].cuffPressure;
            }

            foundDia = true;
            break;
        }
    }

    if (!foundDia)
    {
        diaPressure = beats[endIdx - 1].cuffPressure;
        foundDia = true;
    }

    if (!foundSys || !foundDia)
        return false;

    sbp = sysPressure;
    dbp = diaPressure;
    return true;
}

float MAPDetector::getLatestAmplitude() const
{
    if (beatCount == 0) return 0.0f;
    return beats[beatCount - 1].amplitude;
}

float MAPDetector::getLatestBeatPressure() const
{
    if (beatCount == 0) return 0.0f;
    return beats[beatCount - 1].cuffPressure;
}

float MAPDetector::getMAP() const        { return mapPressure; }
float MAPDetector::getSystolic() const   { return systolicPressure; }
float MAPDetector::getDiastolic() const  { return diastolicPressure; }