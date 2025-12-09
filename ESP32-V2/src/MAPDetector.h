#ifndef MAP_DETECTOR_H
#define MAP_DETECTOR_H

struct OscillationRecord
{
    int amplitude;
    float cuffPressure;
    unsigned long timestamp;
};

class MAPDetector
{
private:
    static const int MAX_OSCILLATIONS = 100;
    static const int TREND_WINDOW = 20;
    
    OscillationRecord oscillations[MAX_OSCILLATIONS];
    int oscillationCount;
    int maxOscillationAmplitude;
    float mapPressure;
    
    float trendBuffer[TREND_WINDOW];
    int trendIdx;
    int trendCount;
    
    float lastPressure;
    unsigned long lastBeatTime;
    float pressureAtLastBeat;
    
public:
    MAPDetector();
    
    // Called when BPMonitor detects a heartbeat
    void recordBeat(float currentPressure, unsigned long timestamp);
    
    // Alternative: add continuous readings (your original approach)
    void addReading(float pressure);
    
    // Process all recordings and calculate MAP
    float detectMAP();
    
    // Get the detected MAP value
    float getMAP() const;
    
    // Get maximum oscillation amplitude
    int getMaxOscillationAmplitude() const;
    
    // Check if we have enough valid data
    bool hasValidData() const;
    
    // Get oscillation data for analysis
    void getOscillationData(OscillationRecord* output, int maxCount, int* actualCount) const;
    
    // Get statistics
    int getOscillationCount() const { return oscillationCount; }
    
    // Reset for new measurement
    void reset();
    
private:
    // Extract baseline trend from raw signal
    float extractTrend(float pressure);
    
    // Calculate oscillation amplitude from raw and trend
    float calculateOscillation(float rawPressure, float trendPressure);
    
    // Record an oscillation point
    void recordOscillation(float amplitude, float cuffPressure, unsigned long timestamp);
    
    // Find MAP from oscillation envelope
    float findMAPFromOscillations();
};

#endif