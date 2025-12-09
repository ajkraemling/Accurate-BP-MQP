#include "filters.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

BiquadFilter::BiquadFilter(float b0_, float b1_, float b2_, float a1_, float a2_)
    : b0(b0_), b1(b1_), b2(b2_), a1(a1_), a2(a2_),
      x1(0), x2(0), y1(0), y2(0) {}

BiquadFilter::BiquadFilter()
    : b0(1), b1(0), b2(0), a1(0), a2(0),
      x1(0), x2(0), y1(0), y2(0) {}

void BiquadFilter::setCoefficients(float b0_, float b1_, float b2_, float a1_, float a2_) {
    b0 = b0_;
    b1 = b1_;
    b2 = b2_;
    a1 = a1_;
    a2 = a2_;
}

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

// // PPGBandpassFilter implementation
// // To determine coefficients: https://www.earlevel.com/main/2021/09/02/biquad-calculator-v3/
// // Use proper sample rate. For lowpass, use 5 Fc (Hz), for highpass, use 0.5 Fc (Hz), and use Q = 0.7071 (Butterworth bandpass is 1/sqrt(2))
// // The three zeroes are the first three inputs, the two poles after 1.0 are the last two inputs. 
// PPGBandpassFilter::PPGBandpassFilter()
//     : hpf(0.9695, -0.9695, 0, -0.9391, 0),  // 0.5 Hz HPF @ 50 Hz // USING FIRST PASS HERE, REDUCE LAG
//       lpf(0.0675, 0.1349, 0.0675, -1.1430, 0.4128)    // 5 Hz LPF @ 50 Hz
//     // : hpf(0.9565, -1.9131, 0.9565, -1.9111, 0.9150),  // 0.5 Hz HPF @ 50 Hz
//     //   lpf(0.0675, 0.1349, 0.0675, -1.1430, 0.4128)    // 5 Hz LPF @ 50 Hz
//     // COMMON OTHER SAMPLE RATES FOR SECOND PASS:
//     // 20 Hz (50ms delay):
//     //   hpf(0.89486, -1.78972, 0.89486, -1.77683, 0.80080),  // 0.5 Hz HPF @ 20 Hz
//     //   lpf(0.29289, 0.58578, 0.29289, -1.30070, 0.17157)    // 5 Hz LPF @ 20 Hz
//     // 100 Hz (10ms delay):
//     //   hpf(0.97803, -1.95606, 0.97803, -1.95558, 0.95654),  // 0.5 Hz HPF @ 100 Hz
//     //   lpf(0.02008, 0.04017, 0.02008, -1.56102, 0.64134)    // 5 Hz LPF @ 100 Hz
// {}

PPGBandpassFilter::PPGBandpassFilter(float sampleRate_)
    : sampleRate(sampleRate_), hpfCutoff(0.5f), lpfCutoff(5.0f)
{
    updateFilterCoefficients();
}

void PPGBandpassFilter::updateFilterCoefficients() {
    // Design Butterworth filters using bilinear transform
    // Q = 0.7071 for Butterworth (1/sqrt(2))
    const float Q = 0.7071f;
    
    // Highpass filter coefficients
    float K_hp = tan(M_PI * hpfCutoff / sampleRate);
    float norm_hp = 1.0f / (1.0f + K_hp / Q + K_hp * K_hp);
    float b0_hp = norm_hp;
    float b1_hp = -2.0f * norm_hp;
    float b2_hp = norm_hp;
    float a1_hp = 2.0f * (K_hp * K_hp - 1.0f) * norm_hp;
    float a2_hp = (1.0f - K_hp / Q + K_hp * K_hp) * norm_hp;
    
    // Lowpass filter coefficients
    float K_lp = tan(M_PI * lpfCutoff / sampleRate);
    float norm_lp = 1.0f / (1.0f + K_lp / Q + K_lp * K_lp);
    float b0_lp = K_lp * K_lp * norm_lp;
    float b1_lp = 2.0f * b0_lp;
    float b2_lp = b0_lp;
    float a1_lp = 2.0f * (K_lp * K_lp - 1.0f) * norm_lp;
    float a2_lp = (1.0f - K_lp / Q + K_lp * K_lp) * norm_lp;
    
    hpf.setCoefficients(b0_hp, b1_hp, b2_hp, a1_hp, a2_hp);
    lpf.setCoefficients(b0_lp, b1_lp, b2_lp, a1_lp, a2_lp);
}

void PPGBandpassFilter::setHeartRateRange(float baselineBPM, float tolerance) {
    // Safety checks
    if (baselineBPM < 30.0f || baselineBPM > 200.0f) {
        // Invalid baseline, use default wide range
        hpfCutoff = 0.5f;   // 30 BPM
        lpfCutoff = 5.0f;   // 300 BPM
        updateFilterCoefficients();
        return;
    }
    
    // Convert BPM to Hz
    float centerHz = baselineBPM / 60.0f;
    
    // Set cutoffs with tolerance (in Hz)
    float toleranceHz = tolerance / 60.0f;
    
    // HPF: baseline - tolerance, but not below 0.3 Hz (18 BPM)
    hpfCutoff = centerHz - toleranceHz;
    if (hpfCutoff < 0.3f) hpfCutoff = 0.3f;
    
    // LPF: baseline + tolerance*2 (allow harmonics), but not above 5 Hz (300 BPM)
    lpfCutoff = centerHz + (toleranceHz * 2.0f);
    if (lpfCutoff > 5.0f) lpfCutoff = 5.0f;
    
    // Sanity check: ensure LPF > HPF
    if (lpfCutoff <= hpfCutoff) {
        lpfCutoff = hpfCutoff + 1.0f;
    }
    
    updateFilterCoefficients();
}

float PPGBandpassFilter::filter(float input) {
    // Cascade: HPF first, then LPF
    float hp_out = hpf.filter(input);
    return lpf.filter(hp_out);
}

void PPGBandpassFilter::reset() {
    hpf.reset();
    lpf.reset();
}