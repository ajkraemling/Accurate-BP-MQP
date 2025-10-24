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
bool DEBUG_MODE = false;  // Change this to false for minimal output
// ==========================

// Calibration variables
int minSignal = 1024;
int maxSignal = 0;
double atmPressure = 0;
int baselineSignalRange = 0;
int baselineAverage = 0;  // Store typical signal level with finger present

// Pulse filtering
const int PULSE_FILTER_SIZE = 3;
int pulseReadings[PULSE_FILTER_SIZE];
int readIndex = 0;
int pulseTotal = 0;

// Pressure filtering
const int PRESSURE_FILTER_SIZE = 4;
float pressureReadings[PRESSURE_FILTER_SIZE];
int pressureIndex = 0;
float pressureTotal = 0;
bool pressureInitialized = false;
int pressureInitCounter = 0;

// Beat detection
unsigned long lastBeatTime = 0;
int beatWindow = 250;  // Allows for up to 240 bpm
bool beatDetected = false;
bool heartbeatJustOccurred = false;

// Threshold with hysteresis
int lowerThreshold;  // Beat end
int upperThreshold;  // Beat start

// Display timing for debug mode 
unsigned long lastDisplayUpdate = 0;
int displayInterval = 20;

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

// This smooths by using the average of the last four readings. Should we use median instead? (Removes influence from large outliers, requires more computation and quicker readings)
int getFilteredPulse(int ppgSignal) {
  pulseTotal = pulseTotal - pulseReadings[readIndex];
  pulseReadings[readIndex] = ppgSignal;
  pulseTotal = pulseTotal + pulseReadings[readIndex]; // Keep an updated value of pulseTotal of pulseReadings
  readIndex = (readIndex + 1) % PULSE_FILTER_SIZE; // Cycle through, replacing old value
  
  return pulseTotal / PULSE_FILTER_SIZE; // Return pulse average over last PULSE_FILTER_SIZE readings
}

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

// For testing/ getting averages and min max over 200 readings
// int numTestReadings = 200;
// int counter = 0;
// long int totals = 0;
// int minTest = 1024;
// int maxTest = 0;

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
    lastBeatTime = currentTime;
  } 
  // Detect falling edge (beat end)
  else if (Signal < lowerThreshold && beatDetected) {
    beatDetected = false;
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