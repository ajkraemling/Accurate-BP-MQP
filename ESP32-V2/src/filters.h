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
    
    BiquadFilter();
    
    void setCoefficients(float b0_, float b1_, float b2_, float a1_, float a2_);

    // Filter a single sample
    float filter(float x);
    
    // Reset filter state
    void reset();
};

class PPGBandpassFilter {
private:
    BiquadFilter hpf;  // Highpass filter (removes DC drift)
    BiquadFilter lpf;  // Lowpass filter (removes high-freq noise)
    
    float sampleRate;
    float hpfCutoff;
    float lpfCutoff;
    
    // Calculate and set biquad coefficients for given cutoff
    void updateFilterCoefficients();

public:
    PPGBandpassFilter(float sampleRate = 50.0f);
    
    // Configure filter based on expected heart rate
    void setHeartRateRange(float baselineBPM, float tolerance = 60.0f);
    
    // Filter a single PPG sample
    float filter(float input);
    
    // Reset both filters
    void reset();
    
    // Get current cutoff frequencies (for debugging)
    float getHPFCutoff() const { return hpfCutoff; }
    float getLPFCutoff() const { return lpfCutoff; }
};

#endif