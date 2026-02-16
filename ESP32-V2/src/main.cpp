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
#include "MotorControl.h"


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
        initializeDetectors();
    }

    ~RealtimeBPMonitor() {
        for (int i = 0; i < detectorCount; i++) {
            delete detectors[i];
        }
    }

    void initializeDetectors() {
        
        // Short windows 
        int windows[] = {40, 60, 80, 100};
        
        // Very low thresholds to catch weak initial pulses
        int minDeviations[] = {2, 3, 4, 5, 6, 8, 10};
        
        for (int w : windows) {
            for (int minDev : minDeviations) {
                if (detectorCount < MAX_ALLOCATED_DETECTORS) {
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
        // Use FILTERED signal for detectors - it has the AC pulse component
        BPMeasurement m;
        m.pressure = pressure;
        m.ppgSignal = filteredPPG;  // Use filtered signal with AC component
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
unsigned long calFirstPulseTime = 0;

// Measurement tracking
struct MeasurementStats {
    int minPPG;
    int maxPPG;
    int minFiltered;
    int maxFiltered;
    unsigned long startTime;
    bool active;
    
    void reset(int raw, int filtered) {
        minPPG = raw;
        maxPPG = raw;
        minFiltered = filtered;
        maxFiltered = filtered;
        startTime = millis();
        active = true;
    }
    
    void update(int raw, int filtered) {
        if (raw < minPPG) minPPG = raw;
        if (raw > maxPPG) maxPPG = raw;
        if (filtered < minFiltered) minFiltered = filtered;
        if (filtered > maxFiltered) maxFiltered = filtered;
    }
} measureStats = {9999, 0, 9999, 0, 0, false};

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

// ================= AUTOMATIC CUFF CONTROL =================

volatile bool buttonPressed = false;
unsigned long lastButtonTime = 0;

enum AutoMode { AUTO_IDLE, AUTO_INFLATING, AUTO_DEFLATING, AUTO_DUMP };
AutoMode autoMode = AUTO_IDLE;

float lastPressure = 0;
unsigned long lastPressureTime = 0;

const float MAX_PRESSURE = 185.0;
const float OCCLUSION_PRESSURE = 140.0;
const float TARGET_DEFLATE_RATE = 3.0;

void IRAM_ATTR handleButton() {
    unsigned long t = millis();
    if (t - lastButtonTime > 300) {
        buttonPressed = true;
        lastButtonTime = t;
    }
}


// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Wire.begin();

    pressureSensor.begin();
    pressureSensor.calibrate();

    Serial.println("\n========================================");
    Serial.println("ESP32 Blood Pressure Monitor");
    Serial.println("========================================");
    Serial.println("Calibrating pulse sensor...");
    Serial.println("Please place finger on sensor and remain still.");
    Serial.println();
    
    calState = CAL_DETECTING_BASELINE;

    motor.begin();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButton, FALLING);

}

void updateCuffControl(float pressure, BPState state, int totalDetections) {

    unsigned long now = millis();

    float dt = (now - lastPressureTime) / 1000.0f;
    float dPdt = 0;
    if (dt > 0.05f) dPdt = (pressure - lastPressure) / dt;

    lastPressure = pressure;
    lastPressureTime = now;

    if (pressure > MAX_PRESSURE)
        autoMode = AUTO_DUMP;

    if (buttonPressed) {
        buttonPressed = false;
        if (autoMode == AUTO_IDLE)
            autoMode = AUTO_INFLATING;
        else
            autoMode = AUTO_DUMP;
    }

    switch (autoMode) {

    case AUTO_IDLE:
        motor.stopInflation();
        motor.stopDeflation();
        break;

    case AUTO_INFLATING:
        motor.startInflation();

        // stop when pulses disappear (occlusion)
        if (pressure > OCCLUSION_PRESSURE && totalDetections < 2) {
            motor.stopInflation();
            autoMode = AUTO_DEFLATING;
        }
        break;

    case AUTO_DEFLATING:
        motor.startDeflation();

        // regulate deflation speed ~3 mmHg/sec
        if (dPdt < -TARGET_DEFLATE_RATE)
            motor.stopDeflation();
        else
            motor.startDeflation();

        if (state == COMPLETE)
            autoMode = AUTO_DUMP;
        break;

    case AUTO_DUMP:
        motor.emergencyStop();
        if (pressure < 20)
            autoMode = AUTO_IDLE;
        break;
    }
}


// =============================================================================
// LOOP
// =============================================================================

void loop() {
    static unsigned long lastSample = 0;
    static unsigned long lastDebugPrint = 0;
    static unsigned long lastCalPrint = 0;
    static unsigned long lastDiagnostic = 0;
    unsigned long now = millis();

    if (now - lastSample >= SAMPLE_RATE_MS) {
        lastSample = now;

        // ================= CALIBRATION PHASE =================
        if (calState != CAL_COMPLETE) {
            if (calibratePulseSensor()) {
                float avgInterval = 0;
                if (calPulseCount > 1 && calLastPulseTime > calFirstPulseTime) {
                    avgInterval = (float)(calLastPulseTime - calFirstPulseTime) / (calPulseCount - 1);
                }

                Serial.println("\n✓ Calibration complete!");
                if (avgInterval > 0) {
                    Serial.print("Detected pulse rate: ");
                    Serial.print(60000.0 / avgInterval, 0);
                    Serial.println(" BPM");
                }
                Serial.print("Signal amplitude: ");
                Serial.print(calPeakToPeak, 0);
                Serial.println(" units");
                Serial.println("\nReady. Press button to begin automatic measurement...\n");

                calState = CAL_COMPLETE;
            } else if (now - lastCalPrint > 2000) {
                lastCalPrint = now;
                printCalibrationStatus();
            }
            return;
        }

        // ================= MEASUREMENT PHASE =================
        float pressure = readPressureSensor();
        int rawPPG = readPPGSensor();
        int filteredPPG = ppgSensor.read();

        BPState prevState = lastState;

        // ---- PROCESS THE SAMPLE FIRST ----
        bpMonitor.processMeasurement(pressure, rawPPG, filteredPPG, now);

        BPState state = bpMonitor.getCurrentState();

        // ---- COUNT DETECTIONS AFTER PROCESSING ----
        int totalDetections = 0;
        BPMonitor* mon = bpMonitor.getMonitor();
        for (int i = 0; i < mon->getDetectorCount(); i++)
            totalDetections += mon->getDetector(i)->getDetectionCount();

        // ---- NOW CONTROL THE CUFF ----
        updateCuffControl(pressure, state, totalDetections);

        // ================= STATE CHANGE =================
        if (state != prevState) {
            printStateChange(state);

            if (state == MEASURING) {
                Serial.println("Resetting signal tracking for new measurement...");
                measureStats.reset(rawPPG, filteredPPG);
            }
        }

        // ================= TRACKING =================
        if (state == MEASURING && measureStats.active) {
            measureStats.update(rawPPG, filteredPPG);
        }

        // ================= DEBUG PRINT =================
        if (state == MEASURING && (now - lastDebugPrint > 1000)) {
            lastDebugPrint = now;

            int rawRange = measureStats.maxPPG - measureStats.minPPG;
            int filteredRange = measureStats.maxFiltered - measureStats.minFiltered;

            Serial.print("Measuring... P:");
            Serial.print(pressure, 0);
            Serial.print(" mmHg | Raw:");
            Serial.print(rawPPG);
            Serial.print(" (Δ");
            Serial.print(rawRange);
            Serial.print(") | Filt:");
            Serial.print(filteredPPG);
            Serial.print(" (Δ");
            Serial.print(filteredRange);
            Serial.print(") | Det:");
            Serial.println(totalDetections);
        }

        // ================= DIAGNOSTICS =================
        if (state == MEASURING && (now - lastDiagnostic > 5000)) {
            lastDiagnostic = now;
            Serial.print("[DIAGNOSTIC] Raw:");
            Serial.print(rawPPG);
            Serial.print(" | Filtered:");
            Serial.print(filteredPPG);
            Serial.print(" | AC amplitude:");
            Serial.println(measureStats.maxFiltered - measureStats.minFiltered);
        }

        // ================= COMPLETION =================
        if (state == COMPLETE && prevState != COMPLETE) {
            measureStats.active = false;
            displayResults();

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

    // Print detector details BEFORE final results
    Serial.println("\n=== DETECTOR ANALYSIS ===");
    BPMonitor* mon = bpMonitor.getMonitor();
    
    // Show best detection from each detector type
    int detectorTypes[10] = {0}; // Track detections by minDeviation
    float pressures[10] = {0};
    int counts[10] = {0};
    
    for (int i = 0; i < mon->getDetectorCount(); i++) {
        SystolicDetector* det = mon->getDetector(i);
        DetectionRecord best = det->getBestDetection();
        
        if (best.pressure > 0) {
            // Group by threshold 
            int group = (i * 10) / mon->getDetectorCount();
            if (group >= 10) group = 9;
            
            pressures[group] += best.pressure;
            counts[group]++;
        }
    }
    
    Serial.println("Detector groups (by sensitivity):");
    for (int i = 0; i < 10; i++) {
        if (counts[i] > 0) {
            Serial.print("  Group ");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(pressures[i] / counts[i], 1);
            Serial.print(" mmHg (n=");
            Serial.print(counts[i]);
            Serial.println(")");
        }
    }
    
    // Show MAP detector results
    Serial.print("\nMAP Analysis:");
    Serial.print("\n  Detected MAP: ");
    Serial.print(mon->getMAPDetector()->getMAP(), 1);
    Serial.print(" mmHg");
    Serial.print("\n  Detected Systolic (from MAP): ");
    Serial.print(mon->getMAPDetector()->getSystolic(), 1);
    Serial.print(" mmHg");
    Serial.print("\n  Detected Diastolic (from MAP): ");
    Serial.print(mon->getMAPDetector()->getDiastolic(), 1);
    Serial.println(" mmHg");

    if (bpMonitor.getResults(sys, dia, map, conf)) {
        Serial.println("\n=== BLOOD PRESSURE RESULTS ===");
        Serial.print("Systolic: "); Serial.print(sys, 1); Serial.println(" mmHg");
        Serial.print("Diastolic: "); Serial.print(dia, 1); Serial.println(" mmHg");
        Serial.print("MAP: "); Serial.print(map, 1); Serial.println(" mmHg");
        Serial.print("Confidence: "); Serial.println(conf, 3);
        
    
        if (dia < 40 || dia > 100) {
            Serial.println("\n[WARNING] Diastolic out of normal range!");
            Serial.println("This suggests MAP detection may have failed.");
        }
        if (sys < 80 || sys > 180) {
            Serial.println("\n[WARNING] Systolic out of normal range!");
        }
        if ((sys - dia) < 20 || (sys - dia) > 80) {
            Serial.println("\n[WARNING] Pulse pressure unusual!");
            Serial.print("Expected 30-60 mmHg, got: ");
            Serial.println(sys - dia, 1);
        }
        
        Serial.println("==============================");
    } else {
        Serial.println("\n[ERROR] Measurement failed - invalid results");
        Serial.println("Possible issues:");
        Serial.println("  - Weak or no pulse signal detected");
        Serial.println("  - Cuff pressure too high (crushed vessels)");
        Serial.println("  - Sensor not making good contact");
        Serial.println("  - Motion during measurement");
    }
}

// =============================================================================
// PULSE SENSOR CALIBRATION
// =============================================================================

bool calibratePulseSensor() {
    int rawPPG = readPPGSensor();
    int filteredPPG = ppgSensor.read();  // Use FILTERED signal for calibration
    
    // Check for sensor saturation
    static bool saturationWarned = false;
    if (rawPPG >= 4090 && !saturationWarned) {
        Serial.println("\n[WARNING] PPG sensor saturated!");
        Serial.println("Adjust: LED brightness, finger pressure, or ambient light\n");
        saturationWarned = true;
    }
    
    // Reset warning flag when entering new calibration
    if (calState == CAL_DETECTING_BASELINE && calSampleCount == 0) {
        saturationWarned = false;
    }
    
    if (calState == CAL_DETECTING_BASELINE) {
        // Build baseline over first 100 samples (about 5 seconds)
        calBaseline = ((calBaseline * calSampleCount) + filteredPPG) / (calSampleCount + 1);
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
        // Use FILTERED signal - look for AC component peaks
        float deviation = filteredPPG - calBaseline;
        
        // Track peak-to-peak on FILTERED signal
        static float minVal = 0;
        static float maxVal = 0;
        
        if (calSampleCount == 0) {
            minVal = filteredPPG;
            maxVal = filteredPPG;
        }
        
        if (filteredPPG < minVal) minVal = filteredPPG;
        if (filteredPPG > maxVal) maxVal = filteredPPG;
        calPeakToPeak = maxVal - minVal;
        
        // Dynamic threshold based on observed amplitude
        float threshold = calPeakToPeak * 0.3;  // 30% of peak-to-peak
        if (threshold < 5.0) threshold = 5.0;   // Minimum threshold
        
        // Detect peaks (positive deviation above threshold)
        bool aboveThreshold = (deviation > threshold);
        
        if (aboveThreshold && !calInPulse) {
            unsigned long now = millis();
            
            // Check for reasonable heart rate (40-180 BPM = 333-1500 ms intervals)
            if (calPulseCount == 0) {
                calInPulse = true;
                calPulseCount++;
                calFirstPulseTime = now;
                calLastPulseTime = now;
                Serial.print(".");
            } else if (now - calLastPulseTime >= 333 && now - calLastPulseTime <= 1500) {
                calInPulse = true;
                calPulseCount++;
                calLastPulseTime = now;
                
                Serial.print(".");
                if (calPulseCount % 10 == 0) Serial.println();
            }
        } else if (!aboveThreshold && calInPulse) {
            calInPulse = false;
        }
        
        calSampleCount++;
        
        // Need at least 5 pulses with reasonable amplitude
        if (calPulseCount >= 5 && calPeakToPeak > 10) {
            return true;  // Calibration complete
        }
        
        // Timeout after 30 seconds
        if (calSampleCount > 600) {
            Serial.println("\n[WARNING] Calibration timeout");
            if (calPulseCount >= 3) {
                Serial.println("Proceeding with weak signal...");
                return true;
            }
            Serial.println("FAILED - No pulses detected. Check sensor placement!");
            // Still return true to allow trying measurement
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
    calFirstPulseTime = 0;
}