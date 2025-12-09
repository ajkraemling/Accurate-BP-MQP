#include "MAPDetector.h"
#include <string.h>
#include <math.h>

MAPDetector::MAPDetector()
    : oscillationCount(0), maxOscillationAmplitude(0), mapPressure(0),
      trendIdx(0), trendCount(0), lastPressure(0), lastBeatTime(0),
      pressureAtLastBeat(0)
{
    memset(oscillations, 0, sizeof(oscillations));
    memset(trendBuffer, 0, sizeof(trendBuffer));
}

void MAPDetector::recordBeat(float currentPressure, unsigned long timestamp)
{
    // This is called when BPMonitor::detectPressureOscillation returns true
    
    // Skip first beat (no interval to measure)
    if (lastBeatTime == 0)
    {
        lastBeatTime = timestamp;
        pressureAtLastBeat = currentPressure;
        return;
    }
    
    // Calculate beat interval (represents oscillation amplitude indirectly)
    unsigned long interval = timestamp - lastBeatTime;
    
    // Calculate pressure drop since last beat (shows deflation rate)
    float pressureDrop = pressureAtLastBeat - currentPressure;
    
    // Extract baseline trend
    float trend = extractTrend(currentPressure);
    
    // Use beat interval as a proxy for pulse amplitude
    // Shorter intervals (faster heart rate) often correlate with stronger pulses
    // But primarily we're just marking that a pulse occurred at this pressure
    int amplitude = 100;  // Fixed amplitude since we're just marking pulse occurrence
    
    // Record this pulse event
    recordOscillation(amplitude, trend, timestamp);
    
    // Update for next beat
    lastBeatTime = timestamp;
    pressureAtLastBeat = currentPressure;
}

void MAPDetector::addReading(float pressure)
{
    if (oscillationCount >= MAX_OSCILLATIONS)
    {
        return;  // Buffer full
    }
    
    // Extract baseline trend
    float trend = extractTrend(pressure);
    
    // Calculate oscillation (difference from trend)
    float oscillation = calculateOscillation(pressure, trend);
    
    // Store as amplitude (absolute value)
    int amplitude = (int)(fabs(oscillation) * 10);  // Scale up for better resolution
    
    // Record this oscillation point
    recordOscillation(amplitude, trend, 0);
    
    lastPressure = pressure;
}

float MAPDetector::extractTrend(float pressure)
{
    // Add to circular buffer
    trendBuffer[trendIdx] = pressure;
    trendIdx = (trendIdx + 1) % TREND_WINDOW;
    if (trendCount < TREND_WINDOW)
    {
        trendCount++;
    }
    
    // Calculate moving average (baseline trend)
    float sum = 0;
    for (int i = 0; i < trendCount; i++)
    {
        sum += trendBuffer[i];
    }
    
    return sum / trendCount;
}

float MAPDetector::calculateOscillation(float rawPressure, float trendPressure)
{
    return rawPressure - trendPressure;
}

void MAPDetector::recordOscillation(float amplitude, float cuffPressure, unsigned long timestamp)
{
    if (oscillationCount >= MAX_OSCILLATIONS)
    {
        return;
    }
    
    // Record the oscillation
    oscillations[oscillationCount].amplitude = (int)amplitude;
    oscillations[oscillationCount].cuffPressure = cuffPressure;
    oscillations[oscillationCount].timestamp = timestamp;
    oscillationCount++;
}

float MAPDetector::detectMAP()
{
    return findMAPFromOscillations();
}

float MAPDetector::findMAPFromOscillations()
{
    if (oscillationCount < 10)
    {
        return -1;  // Insufficient data
    }
    
    // Build envelope by counting beats per pressure range
    const int NUM_BINS = 20;
    int bins[NUM_BINS];
    float binPressures[NUM_BINS];
    memset(bins, 0, sizeof(bins));
    memset(binPressures, 0, sizeof(binPressures));
    
    // Find pressure range
    float minPressure = oscillations[0].cuffPressure;
    float maxPressure = oscillations[0].cuffPressure;
    for (int i = 1; i < oscillationCount; i++)
    {
        if (oscillations[i].cuffPressure < minPressure) minPressure = oscillations[i].cuffPressure;
        if (oscillations[i].cuffPressure > maxPressure) maxPressure = oscillations[i].cuffPressure;
    }
    
    float binSize = (maxPressure - minPressure) / NUM_BINS;
    if (binSize < 0.1f) binSize = 0.1f;
    
    // Count oscillations in each pressure bin
    for (int i = 0; i < oscillationCount; i++)
    {
        int binIdx = (int)((oscillations[i].cuffPressure - minPressure) / binSize);
        if (binIdx >= NUM_BINS) binIdx = NUM_BINS - 1;
        if (binIdx < 0) binIdx = 0;
        
        bins[binIdx]++;
        binPressures[binIdx] += oscillations[i].cuffPressure;
    }
    
    // Average pressure per bin
    for (int i = 0; i < NUM_BINS; i++)
    {
        if (bins[i] > 0)
        {
            binPressures[i] /= bins[i];
        }
        else
        {
            binPressures[i] = minPressure + (i + 0.5f) * binSize;
        }
    }
    
    // Find bin with maximum beat count (most pulses detected)
    int maxBin = 0;
    int maxCount = bins[0];
    for (int i = 1; i < NUM_BINS; i++)
    {
        if (bins[i] > maxCount)
        {
            maxCount = bins[i];
            maxBin = i;
        }
    }
    
    if (maxCount < 2)
    {
        return -1;  // Not enough pulses detected
    }
    
    // MAP is at the pressure where we detected the most pulses
    mapPressure = binPressures[maxBin];
    maxOscillationAmplitude = maxCount;
    
    return mapPressure;
}

float MAPDetector::getMAP() const
{
    return mapPressure;
}

int MAPDetector::getMaxOscillationAmplitude() const
{
    return maxOscillationAmplitude;
}

void MAPDetector::getOscillationData(OscillationRecord* output, int maxCount, int* actualCount) const
{
    int count = oscillationCount < maxCount ? oscillationCount : maxCount;
    memcpy(output, oscillations, count * sizeof(OscillationRecord));
    *actualCount = count;
}

bool MAPDetector::hasValidData() const
{
    // Need at least 10 beats detected
    return (oscillationCount >= 10);
}

void MAPDetector::reset()
{
    oscillationCount = 0;
    maxOscillationAmplitude = 0;
    mapPressure = 0;
    trendIdx = 0;
    trendCount = 0;
    lastPressure = 0;
    lastBeatTime = 0;
    pressureAtLastBeat = 0;
    memset(oscillations, 0, sizeof(oscillations));
    memset(trendBuffer, 0, sizeof(trendBuffer));
}