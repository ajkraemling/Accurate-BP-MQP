// BandpassFilter.h
// Second-order IIR bandpass filter for oscillometric signal extraction
// Designed for 0.5-5 Hz passband at 50 Hz sample rate
#pragma once
#include <math.h>

class BandpassFilter {
public:
    BandpassFilter() { reset(); }
    
    void reset() {
        x1 = x2 = 0.0f;
        y1 = y2 = 0.0f;
    }
    
    // Butterworth bandpass: 0.5-5 Hz @ 50 Hz sample rate
    // Isolates cardiac pulsations (0.67-3.3 Hz typical heart rate range)
    float update(float x0) {
        // Coefficients for 2nd order Butterworth BP filter
        // Passband: 0.5-5 Hz, Fs=50 Hz
        const float b0 = 0.06811f;
        const float b1 = 0.0f;
        const float b2 = -0.06811f;
        const float a1 = -1.71128f;
        const float a2 = 0.86378f;
        
        // Direct Form II implementation
        float y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        
        // Shift states
        x2 = x1;
        x1 = x0;
        y2 = y1;
        y1 = y0;
        
        return y0;
    }

private:
    float x1, x2; // input history
    float y1, y2; // output history
};

// Envelope detector with smoothing
class EnvelopeDetector {
public:
    static constexpr int WINDOW = 10; // 200ms @ 50Hz - smooths 2-3 beats
    
    EnvelopeDetector() { reset(); }
    
    void reset() {
        idx = count = 0;
        for (int i = 0; i < WINDOW; i++) buf[i] = 0.0f;
    }
    
    float update(float oscillation) {
        // Rectify (absolute value)
        float rectified = fabsf(oscillation);
        
        // Moving average smoothing
        buf[idx] = rectified;
        idx = (idx + 1) % WINDOW;
        if (count < WINDOW) count++;
        
        float sum = 0.0f;
        for (int i = 0; i < count; i++) sum += buf[i];
        
        return sum / count;
    }

private:
    float buf[WINDOW];
    int idx, count;
};

// Exponential moving average for slow trend (cuff pressure baseline)
class TrendFilter {
public:
    TrendFilter() : alpha(0.02f), trend(0.0f), initialized(false) {}
    
    void reset() {
        trend = 0.0f;
        initialized = false;
    }
    
    // Alpha = 0.02 gives ~50 sample time constant (1 second @ 50Hz)
    float update(float x) {
        if (!initialized) {
            trend = x;
            initialized = true;
        } else {
            trend = alpha * x + (1.0f - alpha) * trend;
        }
        return trend;
    }
    
    void setAlpha(float a) { alpha = a; }

private:
    float alpha;
    float trend;
    bool initialized;
};