#ifndef FILTERS_H
#define FILTERS_H
class BiquadFilter {
private:
    float b0, b1, b2;  // Numerator coefficients
    float a1, a2;      // Denominator coefficients (a0 = 1)
    float x1, x2;      // Previous inputs
    float y1, y2;      // Previous outputs
    
public:
    // Constructor with coefficients
    BiquadFilter(float b0_, float b1_, float b2_, float a1_, float a2_);
    
    // Filter a single sample
    float filter(float x);
    
    // Reset filter state
    void reset();
};

class PPGBandpassFilter {
private:
    BiquadFilter hpf;  // Highpass filter (removes DC drift)
    BiquadFilter lpf;  // Lowpass filter (removes high-freq noise)
    
public:
    // Constructor - defaults to 50 Hz sample rate
    // 0.5 Hz highpass, 5 Hz lowpass (0.5-5 Hz passband)
    // IF SAMPLE RATE CHANGES, THIS MUST CHANGE
    // IE IF DELAY CHANGES, CHANGE THIS
    PPGBandpassFilter();
    
    // Filter a single PPG sample
    float filter(float input);
    
    // Reset both filters
    void reset();
};

#endif