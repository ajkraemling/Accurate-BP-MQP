/*
 * ESP32 Blood Pressure Monitor - Main
 * Real-time BP monitoring with detailed serial output
 */

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "sensors.h"
#include "BPMonitor.h"
#include "SystolicDetector.h"
#include "MAPDetector.h"
#include "filters.h"

// =============================================================================
// REALTIME BP MONITOR CLASS
// =============================================================================

class RealtimeBPMonitor {
private:
    BPMonitor monitor;

    static const int MAX_ALLOCATED_DETECTORS = 100;
    SystolicDetector* detectors[MAX_ALLOCATED_DETECTORS];
    int detectorCount;

    bool resultsReady;
    float finalSystolic;
    float finalDiastolic;
    float finalMAP;
    float finalConfidence;

public:
    RealtimeBPMonitor()
        : detectorCount(0),
          resultsReady(false),
          finalSystolic(0),
          finalDiastolic(0),
          finalMAP(0),
          finalConfidence(0)
    {
        // No filter needed - ppgSensor.read() already provides filtered data
        initializeDetectors();
    }

    ~RealtimeBPMonitor() {
        for (int i = 0; i < detectorCount; i++) {
            delete detectors[i];
        }
    }

    void initializeDetectors() {
        // Use LARGER windows so baseline doesn't track individual pulses
        // Windows of 50-100 samples = 1-2 seconds of history
        int windows[] = {50, 75, 100};
        int minDeviations[] = {5, 8, 10, 15, 20, 25, 30};  // Range of sensitivities
        
        // Create detectors with different sensitivity levels
        for (int w : windows) {
            for (int minDev : minDeviations) {
                if (detectorCount < MAX_ALLOCATED_DETECTORS) {
                    // threshold = 0.0 means use minDeviation only
                    auto* det = new BaselineDetector(w, 0.0, minDev);
                    detectors[detectorCount++] = det;
                    monitor.addDetector(det);
                }
            }
        }
        
        Serial.print("Initialized ");
        Serial.print(detectorCount);
        Serial.println(" detectors");
    }

    void processMeasurement(float pressure, int rawPPG, int filteredPPG, unsigned long timestamp) {
        // Use RAW signal for detectors - they need the DC offset to calculate baseline
        // The filtered signal only has AC component which is too small
        BPMeasurement m;
        m.pressure = pressure;
        m.ppgSignal = rawPPG;  // Use raw signal with DC offset
        m.rawPPGSignal = rawPPG;
        m.timestamp = timestamp;

        BPState prev = monitor.getState();
        monitor.update(m);
        BPState nowState = monitor.getState();

        if (prev == MEASURING && nowState == COMPLETE)
            finalizeResults();
    }

    BPState getCurrentState() { return monitor.getState(); }

    bool getResults(float& sys,float& dia,float& map,float& conf) {
        if (!resultsReady) return false;
        sys = finalSystolic;
        dia = finalDiastolic;
        map = finalMAP;
        conf = finalConfidence;
        return true;
    }

    BPMonitor* getMonitor() { return &monitor; }

    void reset() {
        monitor.reset();
        resultsReady = false;
    }

private:
    void finalizeResults() {
        monitor.getMAPDetector()->detectMAP();
        BPResult r = monitor.getEnsembleResult();

        finalMAP = monitor.getMAP();
        finalSystolic = r.systolic;
        finalConfidence = r.confidence;

        if (finalMAP > 0 && finalSystolic > 0)
            finalDiastolic = (3.0f * finalMAP - finalSystolic) / 2.0f;
        else
            finalDiastolic = monitor.getMAPDetector()->getDiastolic();

        resultsReady =
            (finalSystolic > 0 &&
             finalDiastolic > 0 &&
             finalSystolic > finalDiastolic &&
             (finalSystolic - finalDiastolic) >= 20);
    }
};

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

RealtimeBPMonitor bpMonitor;
PressureSensor pressureSensor;
PPGSensor ppgSensor(PPG_PIN);
BPState lastState = IDLE;

// Calibration state
enum CalibrationState {
    CAL_IDLE,
    CAL_DETECTING_BASELINE,
    CAL_DETECTING_PULSES,
    CAL_COMPLETE
};

CalibrationState calState = CAL_IDLE;
int calSampleCount = 0;
int calPulseCount = 0;
float calBaseline = 0;
float calPeakToPeak = 0;
int calLastRaw = 0;
bool calInPulse = false;
unsigned long calLastPulseTime = 0;

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================

float readPressureSensor();
int readPPGSensor();
void displayResults();
void printStateChange(BPState newState);
bool calibratePulseSensor();
void printCalibrationStatus();
void resetCalibration();

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Wire.begin();

    pressureSensor.begin();
    pressureSensor.calibrate();

    bpMonitor.getMonitor()->reset();

    Serial.println("\n========================================");
    Serial.println("ESP32 Blood Pressure Monitor");
    Serial.println("========================================");
    Serial.println("Calibrating pulse sensor...");
    Serial.println("Please place finger on sensor and remain still.");
    Serial.println();
    
    calState = CAL_DETECTING_BASELINE;
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    static unsigned long lastSample = 0;
    static unsigned long lastDebugPrint = 0;
    static unsigned long lastCalPrint = 0;
    unsigned long now = millis();

    if (now - lastSample >= SAMPLE_RATE_MS) {
        lastSample = now;

        // ===== CALIBRATION PHASE =====
        if (calState != CAL_COMPLETE) {
            if (calibratePulseSensor()) {
                Serial.println("\n✓ Calibration complete!");
                Serial.print("Detected pulse rate: ");
                Serial.print(60000.0 / ((float)(calLastPulseTime - 0) / calPulseCount), 0);
                Serial.println(" BPM");
                Serial.print("Signal amplitude: ");
                Serial.print(calPeakToPeak, 0);
                Serial.println(" units");
                Serial.println("\nReady. Inflate cuff to begin measurement...\n");
                calState = CAL_COMPLETE;
            } else if (now - lastCalPrint > 2000) {
                lastCalPrint = now;
                printCalibrationStatus();
            }
            return;
        }

        // ===== MEASUREMENT PHASE =====
        float pressure = readPressureSensor();
        int rawPPG = readPPGSensor();
        int filteredPPG = ppgSensor.read();  // Get filtered PPG from sensor

        BPState prevState = lastState;
        
        bpMonitor.processMeasurement(pressure, rawPPG, filteredPPG, now);

        BPState state = bpMonitor.getCurrentState();

        // Print state changes
        if (state != prevState) {
            printStateChange(state);
            
            // Reset PPG range tracking when starting measurement
            if (state == MEASURING) {
                // Reset the static variables in the debug print section
                Serial.println("Resetting signal tracking for new measurement...");
            }
        }

        // Print periodic status during measurement
        if (state == MEASURING && (now - lastDebugPrint > 1000)) {
            lastDebugPrint = now;
            
            // Count total detections
            int totalDetections = 0;
            BPMonitor* mon = bpMonitor.getMonitor();
            for (int i = 0; i < mon->getDetectorCount(); i++) {
                totalDetections += mon->getDetector(i)->getDetectionCount();
            }
            
            // Calculate signal statistics - MUST reset when entering MEASURING state
            static int minPPG = 9999;
            static int maxPPG = 0;
            static BPState lastTrackState = IDLE;
            
            // Reset when entering MEASURING state
            if (state == MEASURING && lastTrackState != MEASURING) {
                minPPG = rawPPG;
                maxPPG = rawPPG;
            }
            lastTrackState = state;
            
            if (rawPPG < minPPG) minPPG = rawPPG;
            if (rawPPG > maxPPG) maxPPG = rawPPG;
            int peakToPeak = maxPPG - minPPG;
            
            Serial.print("Measuring... P:");
            Serial.print(pressure, 0);
            Serial.print(" mmHg, PPG:");
            Serial.print(rawPPG);
            Serial.print(" (");
            Serial.print(minPPG);
            Serial.print("-");
            Serial.print(maxPPG);
            Serial.print(", Δ");
            Serial.print(peakToPeak);
            Serial.print("), Det:");
            Serial.println(totalDetections);
        }

        // Handle completion
        if (state == COMPLETE && prevState != COMPLETE) {
            displayResults();
            
            // Wait a moment then reset everything including calibration
            delay(2000);
            bpMonitor.reset();
            resetCalibration();
            
            Serial.println("\n========================================");
            Serial.println("Recalibrating pulse sensor...");
            Serial.println("Please keep finger on sensor.");
            Serial.println("========================================\n");
        }

        lastState = state;
    }
}

// =============================================================================
// SENSOR READ FUNCTIONS
// =============================================================================

float readPressureSensor() {
    return pressureSensor.readGaugePressure();
}

int readPPGSensor() {
    return ppgSensor.readRaw();
}

// =============================================================================
// STATE CHANGE PRINTER
// =============================================================================

void printStateChange(BPState newState) {
    Serial.print("[STATE CHANGE] ");
    switch(newState) {
        case IDLE:
            Serial.println("IDLE - Waiting for pressure...");
            break;
        case INFLATING:
            Serial.println("INFLATING - Cuff is inflating...");
            break;
        case MEASURING:
            Serial.println("MEASURING - Recording during deflation...");
            break;
        case COMPLETE:
            Serial.println("COMPLETE - Measurement finished");
            break;
    }
}

// =============================================================================
// RESULTS DISPLAY
// =============================================================================

void displayResults() {
    float sys, dia, map, conf;

    if (bpMonitor.getResults(sys, dia, map, conf)) {
        Serial.println("\n=== BLOOD PRESSURE RESULTS ===");
        Serial.print("Systolic: "); Serial.print(sys, 1); Serial.println(" mmHg");
        Serial.print("Diastolic: "); Serial.print(dia, 1); Serial.println(" mmHg");
        Serial.print("MAP: "); Serial.print(map, 1); Serial.println(" mmHg");
        Serial.print("Confidence: "); Serial.println(conf, 3);
        Serial.println("==============================");
    } else {
        Serial.println("\n[ERROR] Measurement failed - invalid results");
    }
}

// =============================================================================
// PULSE SENSOR CALIBRATION
// =============================================================================

bool calibratePulseSensor() {
    int rawPPG = readPPGSensor();
    
    // Check for sensor saturation
    if (rawPPG >= 4090) {
        static int saturationWarnings = 0;
        if (saturationWarnings == 0) {
            Serial.println("\n[ERROR] PPG sensor saturated at maximum value!");
            Serial.println("FIXES:");
            Serial.println("  1. Reduce LED brightness if adjustable");
            Serial.println("  2. Adjust finger pressure on sensor");
            Serial.println("  3. Block ambient light");
            Serial.println("  4. Check sensor wiring\n");
        }
        saturationWarnings++;
        if (saturationWarnings > 10) saturationWarnings = 0;  // Reset counter
    }
    
    if (calState == CAL_DETECTING_BASELINE) {
        // Build baseline over first 100 samples (about 5 seconds)
        calBaseline = ((calBaseline * calSampleCount) + rawPPG) / (calSampleCount + 1);
        calSampleCount++;
        
        if (calSampleCount >= 100) {
            Serial.print("Baseline established: ");
            Serial.println(calBaseline, 0);
            Serial.println("Detecting pulses...");
            calState = CAL_DETECTING_PULSES;
            calSampleCount = 0;
        }
        return false;
    }
    
    if (calState == CAL_DETECTING_PULSES) {
        // Detect pulses as significant deviations from baseline
        float deviation = rawPPG - calBaseline;
        float threshold = 20.0;  // Minimum pulse amplitude
        
        // Track peak-to-peak amplitude
        static float minVal = 9999;
        static float maxVal = 0;
        if (rawPPG < minVal) minVal = rawPPG;
        if (rawPPG > maxVal) maxVal = rawPPG;
        calPeakToPeak = maxVal - minVal;
        
        // Detect rising edge (start of pulse)
        bool aboveThreshold = (deviation > threshold);
        
        if (aboveThreshold && !calInPulse) {
            calInPulse = true;
            calPulseCount++;
            calLastPulseTime = millis();
            
            Serial.print(".");  // Progress indicator
            if (calPulseCount % 10 == 0) Serial.println();
        } else if (!aboveThreshold && calInPulse) {
            calInPulse = false;
        }
        
        calSampleCount++;
        
        // Need at least 5 pulses and good amplitude
        if (calPulseCount >= 5 && calPeakToPeak > 30) {
            return true;  // Calibration complete
        }
        
        // Timeout after 30 seconds
        if (calSampleCount > 600) {
            Serial.println("\n[WARNING] Calibration timeout - continuing anyway");
            Serial.println("Signal may be weak. Check sensor placement.");
            return true;
        }
    }
    
    return false;
}

void printCalibrationStatus() {
    Serial.print("Calibrating... Samples: ");
    Serial.print(calSampleCount);
    Serial.print(", Pulses: ");
    Serial.print(calPulseCount);
    Serial.print(", Amplitude: ");
    Serial.println(calPeakToPeak, 0);
}

void resetCalibration() {
    calState = CAL_DETECTING_BASELINE;
    calSampleCount = 0;
    calPulseCount = 0;
    calBaseline = 0;
    calPeakToPeak = 0;
    calLastRaw = 0;
    calInPulse = false;
    calLastPulseTime = 0;
}