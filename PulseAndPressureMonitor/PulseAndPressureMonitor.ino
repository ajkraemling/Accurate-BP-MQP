
/*
 * ========================================================================================================
 * WEARABLE BLOOD PRESSURE MONITOR
 * ========================================================================================================
 * 
 * This code implements a continuous blood pressure monitoring system using:
 *   1. PPG (Photoplethysmography) sensor - detects heartbeats by reading light changes on finger
 *   2. MPRLS pressure sensor - measures pressure of arm cuff to get bp readings
 * 
 * How it works:
 *   - Calibration: Establishes baseline readings for both sensors over 5 seconds
 *   - Monitoring: Continuously reads both sensors, filters the signals, and detects heartbeats
 *   - The system prints out the cuff pressure at a detected heart beat, so it can be used to measure systolic and diastolic
 * 
 * Features:
 *   - Adaptive threshold-based heartbeat detection with hysteresis
 *   - Moving average filtering to reduce noise in both PPG and pressure signals
 *   - Signal quality monitoring to detect sensor issues (loose fit, movement, etc.)
 *   - Two output modes: DEBUG (full data stream) or normal (heartbeats and pressure only)
 * 
 * Hardware:
 *   - PPG sensor connected to analog pin A0, 5V, GND
 *   - MPRLS pressure sensor connected via I2C, i.e. 5V Vin, GND, SCL, SDA 
 * 
 * ========================================================================================================
 */
#include <Wire.h>
#include "Adafruit_MPRLS.h"

#define RESET_PIN  -1
#define EOC_PIN    -1
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

int PulseSensorPurplePin = 0;

int Signal;

double mmHgConst = 0.75006157584566;

// ===== MODE SELECTION =====
// Set to true for full debug output, false for timestamp+pressure only on heartbeat
bool DEBUG_MODE = true;  // Change this to false for minimal output
// ==========================

// Calibration variables
int minSignal = 1024;
int maxSignal = 0;
double atmPressure = 0;
int baselineSignalRange = 0;
int baselineAverage = 0;  // Store typical signal level with finger present

// Pulse filtering
const int PULSE_FILTER_SIZE = 3; // Size of runnning average
int pulseReadings[PULSE_FILTER_SIZE]; // Running average (latest PULSE_FILTER_SIZE readings)
int readIndex = 0; // Index of oldest readings in pulseReadings
int pulseTotal = 0; // Contains current total of readings in pulseReadings

// Pressure filtering
const int PRESSURE_FILTER_SIZE = 4; // Size of runnning average
float pressureReadings[PRESSURE_FILTER_SIZE]; // Running average (latest PULSE_FILTER_SIZE readings)
int pressureIndex = 0; // Index of oldest readings in pulseReadings
float pressureTotal = 0; // Contains current total of readings in pulseReadings
bool pressureInitialized = false; // Has array filled up with readings yet?
int pressureInitCounter = 0;

// Beat detection
unsigned long lastBeatTime = 0;
int beatWindow = 250;  // Allows for up to 240 bpm, minimum amount of time before we can detect a new beat
bool beatDetected = false; 
bool heartbeatJustOccurred = false;

// Threshold with hysteresis
int lowerThreshold;  // Beat end
int upperThreshold;  // Beat start

// Display timing for debug mode 
unsigned long lastDisplayUpdate = 0;
int displayInterval = 20;

// Quality of signal variables
bool signalQualityGood = false;
unsigned long lastQualityCheck = 0;
const int QUALITY_CHECK_INTERVAL = 2000; // Check every 2 seconds
int recentBeats[5] = {0,0,0,0,0};
int beatIndex = 0;

/*
 * SETUP FUNCTION
 * Runs once at startup to initialize hardware and call calibrate
 */
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n===========================================================================================================");
  Serial.println("********************************************* Initialization **********************************************");
  Serial.println("===========================================================================================================");
  if (!mpr.begin()) {
    Serial.println("ERROR: MPRLS sensor not found!");
    while (1) delay(10);
  }
  
  // Initialize arrays
  for (int i = 0; i < PULSE_FILTER_SIZE; i++) {
    pulseReadings[i] = 0;
  }
  for (int i = 0; i < PRESSURE_FILTER_SIZE; i++) {
    pressureReadings[i] = 0;
  }
  
  calibrate();
}

/*
 * CALIBRATE FUNCTION
 * Finds baseline measurements for both sensors over a 5-second period
 * 
 * Purpose:
 *   - Find PPG signal min/max to set adaptive thresholds
 *   - Calculate average atmospheric pressure as baseline
 *   - Validate that sensors are working properly
 * 
 * Process:
 *   1. Wait 5 seconds for user to position finger
 *   2. Collect 5 seconds of data (~250 samples)
 *   3. Calculate thresholds and baselines
 *   4. Display warnings if signal quality is poor
 */
void calibrate() {
  Serial.println("\n=== CALIBRATION ===");
  Serial.print("  Place finger on sensor...\n   ");
  
  // Blocking while we wait for user to place finger on sensor
  // We may want to replace this with sensing if the finger is placed correctly, but unsure
  for (int i = 5; i > 0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(1000);
  }
  Serial.println("\nCalibrating...");
  
  unsigned long calibrationStart = millis();
  int sampleCount = 0;
  long sumSignal = 0;
  
  while (millis() - calibrationStart < 5000) {
    // Read signal from pulse sensor
    Signal = analogRead(PulseSensorPurplePin);
    
    // Find upper and lower threshold values for pulse sensor reading
    if (Signal > maxSignal) maxSignal = Signal;
    if (Signal < minSignal && Signal > 10) minSignal = Signal;
    sumSignal += Signal;
    
    // Add all pressure sensor readings to take average
    atmPressure += mpr.readPressure();
    sampleCount++;
    
    // Update progress
    if (sampleCount % 100 == 0) Serial.print(".");
    
    delay(20);
  }
  
  int range = maxSignal - minSignal;
  baselineAverage = sumSignal / sampleCount;  // Store average signal level
  
  // More sensitive thresholds with hysteresis
  upperThreshold = minSignal + (range * 0.46);
  lowerThreshold = minSignal + (range * 0.40);  // Lower threshold for beat end
  
  // Get average pressure reading for absolute atmospheric pressure
  atmPressure = atmPressure / sampleCount;
  
  Serial.println("\nCalibration complete!");
  Serial.println("   Signal Range: " + String(minSignal) + " - " + String(maxSignal) + " (Range: " + String(range) + ")");
  Serial.println("   Average Signal: " + String(baselineAverage));
  Serial.println("   Upper Threshold: " + String(upperThreshold));
  Serial.println("   Lower Threshold: " + String(lowerThreshold));
  Serial.println("   Baseline Pressure: " + String(atmPressure, 1) + " hPa");
  
  // Check if pulse sensor has good signal
  if (range < 20) {
    Serial.println("\nWARNING: Weak pulse signal detected!");
    Serial.println("PPG sensor may be too loose or poorly positioned.");
    delay(20);
  } else if (range > 800) {
    Serial.println("\nWARNING: Signal may be saturated!");
    Serial.println("PPG sensor may be too tight or too much light may be getting into the finger cuff");
    delay(20);
  }
  
  // Print appropriate header based on mode
  if (DEBUG_MODE) {
    Serial.println("=== DEBUG MODE ===");
    Serial.println("========================================================================");
    Serial.println(" Heartbeat? | Raw PPG Signal |  Pulse Signal  |  Pressure (mmHg)");
    Serial.println("========================================================================");
  } else {
    Serial.println("=== HEARTBEAT LOGGING MODE ===");
  }
}

/*
 * GET FILTERED PULSE
 * Applies a moving average filter to the raw PPG signal to reduce noise
 * 
 * Parameters:
 *   ppgSignal - Current raw reading from PPG sensor
 * 
 * Returns:
 *   Filtered signal value (average of last PULSE_FILTER_SIZE readings)
 * 
 * Method:
 *   Uses a circular buffer and running sum for O(1) efficiency
 *   Each new reading replaces the oldest reading in the buffer
 */
int getFilteredPulse(int ppgSignal) {
  pulseTotal = pulseTotal - pulseReadings[readIndex];
  pulseReadings[readIndex] = ppgSignal;
  pulseTotal = pulseTotal + pulseReadings[readIndex]; // Keep an updated value of pulseTotal of pulseReadings
  readIndex = (readIndex + 1) % PULSE_FILTER_SIZE; // Cycle through, replacing old value
  
  return pulseTotal / PULSE_FILTER_SIZE; // Return pulse average over last PULSE_FILTER_SIZE readings
}

/*
 * GET FILTERED PRESSURE
 * Applies a moving average filter to the raw pressure signal to smooth readings
 * 
 * Parameters:
 *   rawPressure - Current raw reading from pressure sensor (hPa)
 * 
 * Returns:
 *   Filtered pressure value (average of last PRESSURE_FILTER_SIZE readings)
 * 
 * Method:
 *   Uses circular buffer with running sum for efficiency
 *   Includes initialization check to ensure filter is filled before use
 */
float getFilteredPressure(float rawPressure) {
   pressureTotal = pressureTotal - pressureReadings[pressureIndex];
  pressureReadings[pressureIndex] = rawPressure;
  pressureTotal = pressureTotal + pressureReadings[pressureIndex];
  pressureIndex = (pressureIndex + 1) % PRESSURE_FILTER_SIZE;
  
  // Initialize pressure baseline after filter is filled
  if (!pressureInitialized) {
    pressureInitCounter++;
    if (pressureInitCounter > PRESSURE_FILTER_SIZE + 5) {
      pressureInitialized = true;
    }
  }
  
  return pressureTotal / PRESSURE_FILTER_SIZE; // Return average 
}

/*
 * CHECK SIGNAL QUALITY
 * Analyzes recent data to determine if sensor readings are reliable
 * 
 * Checks three quality indicators:
 *   1. Signal Range: Is the PPG signal strong enough? (20-800 range)
 *   2. Heart Rate: Are beats being detected at a reasonable rate? (40-180 bpm)
 *   3. Stability: Has the signal remained consistent? (no sudden shifts)
 * 
 * If any check fails, prints a warning message
 * Updates global flag 'signalQualityGood'
 */
void checkSignalQuality() {
  // Check signal range (already warned in calibration)
  int range = maxSignal - minSignal;
  bool rangeOK = (range >= 20 && range <= 800);
  
  // Serial.println("\n=== Signal Quality Check ===");
  // Serial.print("Recent beats array: ");
  // for (int i = 0; i < 5; i++) {
  //   Serial.print(recentBeats[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Check if we're getting beats at reasonable intervals
  int avgBeatInterval = 0;
  int validBeats = 0;
  for (int i = 0; i < 5; i++) {
    if (recentBeats[i] > 0) {
      avgBeatInterval += recentBeats[i];
      validBeats++;
    }
  }
  
  // Serial.print("Valid beats counted: ");
  // Serial.println(validBeats);
  
  bool heartRateOK = false;
  if (validBeats >= 3) {
    avgBeatInterval /= validBeats;
    int bpm = 60000 / avgBeatInterval;
    // Serial.print("BPM: ");
    // Serial.println(bpm);
    heartRateOK = (bpm >= 40 && bpm <= 180);
  }
  
  // Detect signal variance (try to detect if finger slipped off)
  int currentAvg = Signal;
  bool stabilityOK = abs(currentAvg - baselineAverage) < (maxSignal - minSignal) * 0.6;
  
  // Serial.print("RangeOk: ");
  // Serial.println(rangeOK);
  // Serial.print("heartRateOK: ");
  // Serial.println(heartRateOK);
  // Serial.print("stabilityOK: ");
  // Serial.println(stabilityOK);

  signalQualityGood = rangeOK && heartRateOK && stabilityOK;
  
  if (!signalQualityGood) {
    Serial.println("\n*** WARNING: Poor signal quality detected ***");
    if (!rangeOK) Serial.println("    - Signal range issue");
    if (!heartRateOK) Serial.println("    - Heart rate out of normal range or no beats detected");
    if (!stabilityOK) Serial.println("    - Signal unstable (finger may have moved)");
  }
}

// For testing/ getting averages and min max over 200 readings
// int numTestReadings = 200;
// int counter = 0;
// long int totals = 0;
// int minTest = 1024;
// int maxTest = 0;


/*
 * MAIN LOOP
 * Continuously reads sensors, detects heartbeats, and outputs data
 * 
 * Process each cycle (runs every 50ms):
 *   1. Read and filter both sensors
 *   2. Detect heartbeats using threshold crossing with hysteresis
 *   3. Track beat intervals for quality monitoring
 *   4. Periodically check signal quality
 *   5. Output data (format depends on DEBUG_MODE setting)
 */
void loop() {
  int rawPPGSignal = analogRead(PulseSensorPurplePin);
  Signal = getFilteredPulse(rawPPGSignal);

  float rawPressureSignal = mpr.readPressure();
  float pressure_abs_hPa = getFilteredPressure(rawPressureSignal);
  float pressure_mmHg = (pressure_abs_hPa - atmPressure) * mmHgConst;
  
  // Only process after pressure is initialized
  if (!pressureInitialized) {
    delay(50);
    return;
  }

  // For testing  
  // counter++;
  // if (maxTest < rawPPGSignal) maxTest = rawPPGSignal;
  // if (minTest > rawPPGSignal && rawPPGSignal > 2) minTest = rawPPGSignal;
  // totals = totals + rawPPGSignal;
  // if (counter > numTestReadings) {
  //   Serial.print("\nAverage PPG Reading: ");
  //   Serial.println(totals / counter);
  //   Serial.print("Min: ");
  //   Serial.println(minTest);
  //   Serial.print("Max: ");
  //   Serial.println(maxTest);
  //   while(1) {} // block forever, effectively over
  // }

  // Pulse sensor beat detection
  unsigned long currentTime = millis();
  heartbeatJustOccurred = false;
  
  // Detect rising edge (beat start)
  if (Signal > upperThreshold && !beatDetected && (currentTime - lastBeatTime > beatWindow)) {
    beatDetected = true;
    heartbeatJustOccurred = true;

    unsigned long beatInterval = currentTime - lastBeatTime;
  
    // Store the interval (only if it's reasonable - not the first beat)
    if (lastBeatTime > 0) {  
      recentBeats[beatIndex] = beatInterval;
      // Serial.print("Stored beat interval: ");
      // Serial.print(beatInterval);
      // Serial.print(" ms at index ");
      // Serial.println(beatIndex);
      beatIndex = (beatIndex + 1) % 5;
    } else {
      Serial.println("First beat detected, not storing interval");
    }

    lastBeatTime = currentTime;
  } 
  // Detect falling edge (beat end)
  else if (Signal < lowerThreshold && beatDetected) {
    beatDetected = false;
  }

  // Periodic signal quality check
  if (currentTime - lastQualityCheck > QUALITY_CHECK_INTERVAL) {
    checkSignalQuality();
    lastQualityCheck = currentTime;
  }
  
  if (DEBUG_MODE) {
    // Debug mode, print all information
    if (currentTime - lastDisplayUpdate >= displayInterval) {
      lastDisplayUpdate = currentTime;
      
      String heartbeatStr = beatDetected ? "   YES    " : "    NO    "; // Maybe replace beatDetected with heartbeatJustDetected
      
      String signalStr = String(Signal);
      while (signalStr.length() < 4) signalStr = " " + signalStr; // Adds spaces for consistent padding
      
      int pressureInt = round(pressure_mmHg);
      String pressureStr = String(pressureInt);
      while (pressureStr.length() < 4) pressureStr = " " + pressureStr; // Adds spaces for consistent padding
      
      Serial.println(heartbeatStr + " |     " + rawPPGSignal + "       |        " + signalStr + "       |        " + pressureStr);
    }
    
  } else {
    // Heartbeat logging mode: only print timestamp and pressure when heartbeat occurs
    if (heartbeatJustOccurred) {
      int pressureInt = round(pressure_mmHg);
      Serial.println(String(currentTime) + "ms: Pressure at heartbeat: " + String(pressureInt)) + " mmHg";
    }
  }
  
  delay(50);
}