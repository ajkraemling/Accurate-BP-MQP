#include "filters.h"

BiquadFilter::BiquadFilter(float b0_, float b1_, float b2_, float a1_, float a2_)
    : b0(b0_), b1(b1_), b2(b2_), a1(a1_), a2(a2_),
      x1(0), x2(0), y1(0), y2(0) {}

float BiquadFilter::filter(float x) {
    // Direct Form II Transposed
    float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
    
    // Shift states
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    
    return y;
}

void BiquadFilter::reset() {
    x1 = x2 = 0;
    y1 = y2 = 0;
}

// PPGBandpassFilter implementation
// To determine coefficients: https://www.earlevel.com/main/2021/09/02/biquad-calculator-v3/
// Use proper sample rate. For lowpass, use 5 Fc (Hz), for highpass, use 0.5 Fc (Hz), and use Q = 0.7071 (Butterworth bandpass is 1/sqrt(2))
// The three zeroes are the first three inputs, the two poles after 1.0 are the last two inputs. 
PPGBandpassFilter::PPGBandpassFilter()
    : hpf(0.9695, -0.9695, 0, -0.9391, 0),  // 0.5 Hz HPF @ 50 Hz // USING FIRST PASS HERE, REDUCE LAG
      lpf(0.0675, 0.1349, 0.0675, -1.1430, 0.4128)    // 5 Hz LPF @ 50 Hz
    // : hpf(0.9565, -1.9131, 0.9565, -1.9111, 0.9150),  // 0.5 Hz HPF @ 50 Hz
    //   lpf(0.0675, 0.1349, 0.0675, -1.1430, 0.4128)    // 5 Hz LPF @ 50 Hz
    // COMMON OTHER SAMPLE RATES FOR SECOND PASS:
    // 20 Hz (50ms delay):
    //   hpf(0.89486, -1.78972, 0.89486, -1.77683, 0.80080),  // 0.5 Hz HPF @ 20 Hz
    //   lpf(0.29289, 0.58578, 0.29289, -1.30070, 0.17157)    // 5 Hz LPF @ 20 Hz
    // 100 Hz (10ms delay):
    //   hpf(0.97803, -1.95606, 0.97803, -1.95558, 0.95654),  // 0.5 Hz HPF @ 100 Hz
    //   lpf(0.02008, 0.04017, 0.02008, -1.56102, 0.64134)    // 5 Hz LPF @ 100 Hz
{}

float PPGBandpassFilter::filter(float input) {
    // Cascade: HPF first, then LPF
    float hp_out = hpf.filter(input);
    return lpf.filter(hp_out);
}

void PPGBandpassFilter::reset() {
    hpf.reset();
    lpf.reset();
}