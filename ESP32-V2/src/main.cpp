#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPRLS.h>

MAX30105 ppg;
Adafruit_MPRLS mpr = Adafruit_MPRLS(); 

// -------- SETTINGS ----------
#define SAMPLE_RATE 50       // pressure sample rate (Hz)
#define CALIBRATION_TIME 30000   // 30 sec
#define TARGET_DEFLATE_RATE 3    // mmHg per second deflation
#define MAX_SAMPLES 1500         // store up to 1500 samples

// Data buffers
float pressure[MAX_SAMPLES];
float oscillation[MAX_SAMPLES];
float filteredOsc[MAX_SAMPLES];

unsigned long lastSample = 0;

// Utility function: simple moving average
float smooth(float current, float prev, float alpha = 0.1) {
  return alpha * current + (1 - alpha) * prev;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing MAX30102...");
  if (!ppg.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 NOT FOUND");
    while (1);
  }

  ppg.setup();
  ppg.setPulseAmplitudeRed(0x3F);
  ppg.setPulseAmplitudeIR(0);
  ppg.setSampleRate(100);
  ppg.setPulseWidth(411);

  Serial.println("Initializing MPRLS...");
  if (!mpr.begin()) {
    Serial.println("MPRLS NOT FOUND");
    while (1);
  }

  Serial.println("Starting 30-second PPG calibration...");
  unsigned long start = millis();
  while (millis() - start < CALIBRATION_TIME) {
    if (ppg.available()) {
      ppg.getRed();  // just stabilize finger placement
      ppg.nextSample();
    }
  }
  Serial.println("Calibration complete.");
  delay(500);
}

// ----------------------- OSCILLOMETRIC MEASUREMENT -------------------------
void loop() {
  Serial.println("Begin BP Measurement");
  Serial.println("Inflate cuff to 160 - 180 mmHg and release slowly...");
  delay(3000);

  int idx = 0;
  float prevPressure = 0;

  // -------------------- DATA COLLECTION (deflation phase) --------------------
  while (idx < MAX_SAMPLES) {

    if (millis() - lastSample >= (1000 / SAMPLE_RATE)) {
      lastSample = millis();

      // Read pressure in hPa → convert to mmHg
      float p_hPa = mpr.readPressure();
      float p_mmHg = p_hPa * 0.750062;

      pressure[idx] = p_mmHg;

      // Get one PPG sample (RED LED)
      if (ppg.available()) {
        uint32_t red = ppg.getRed();
        ppg.nextSample();

        // Oscillometric pulse = AC component
        float ac = red - smooth(red, red);
        oscillation[idx] = ac;
      }

      // Filter oscillation
      if (idx == 0) {
        filteredOsc[idx] = oscillation[idx];
      } else {
        filteredOsc[idx] = smooth(oscillation[idx], filteredOsc[idx - 1], 0.3);
      }

      idx++;
    }
  }

  Serial.println("Data collection complete.");
  Serial.println("Processing oscillometric envelope...");

  // -------------------- FIND MAP (max oscillation) --------------------------
  float maxOsc = 0;
  int maxIndex = 0;
  for (int i = 0; i < idx; i++) {
    float absOsc = abs(filteredOsc[i]);
    if (absOsc > maxOsc) {
      maxOsc = absOsc;
      maxIndex = i;
    }
  }
  float MAP = pressure[maxIndex];

  // -------------------- ESTIMATE SYS & DIA ------------------------------
  //
  // Standard ratios:
  // SYS = 0.55 * MAP
  // DIA = 0.85 * MAP
  //
  // Better: search oscillation crossing these thresholds
  //
  float sysTarget = maxOsc * 0.55;
  float diaTarget = maxOsc * 0.85;

  float systolic = -1;
  float diastolic = -1;

  // systolic occurs BEFORE MAP peak
  for (int i = 0; i < maxIndex; i++) {
    if (abs(filteredOsc[i]) > sysTarget) {
      systolic = pressure[i];
      break;
    }
  }

  // diastolic occurs AFTER MAP peak
  for (int i = maxIndex; i < idx; i++) {
    if (abs(filteredOsc[i]) < diaTarget) {
      diastolic = pressure[i];
      break;
    }
  }

  Serial.println(F("------------- RESULTS ---------------"));
  Serial.print("MAP:       "); Serial.println(MAP);
  Serial.print("SYS:       "); Serial.println(systolic);
  Serial.print("DIA:       "); Serial.println(diastolic);
  Serial.println(F("-------------------------------------"));

  delay(5000);
}



// #include <Arduino.h>
// #include "MAX30105.h"

// /* 
//   MAX30105 Breakout: Output all the raw Red/IR/Green readings
//   By: Nathan Seidle @ SparkFun Electronics
//   Date: October 2nd, 2016
//   https://github.com/sparkfun/MAX30105_Breakout

//   Outputs all Red/IR/Green values.

//   Hardware Connections (Breakoutboard to Arduino):
//   -5V = 5V (3.3V is allowed)
//   -GND = GND
//   -SDA = A4 (or SDA)
//   -SCL = A5 (or SCL)
//   -INT = Not connected

//   The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
//   but it will also run at 3.3V.

//   This code is released under the [MIT License](http://opensource.org/licenses/MIT).
// */

// // #include <Wire.h>

// // MAX30105 particleSensor;

// // #define debug Serial //Uncomment this line if you're using an Uno or ESP
// // //#define debug SerialUSB //Uncomment this line if you're using a SAMD21

// // void setup()
// // {
// //   debug.begin(115200);
// //   debug.println("MAX30105 Basic Readings Example");

// //   // Initialize sensor
// //   if (particleSensor.begin() == false)
// //   {
// //     debug.println("MAX30105 was not found. Please check wiring/power. ");
// //     while (1);
// //   }

// //   particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive
// // }

// // void loop()
// // {
// //   debug.print(" R[");
// //   debug.print(particleSensor.getRed());
// //   debug.print("] IR[");
// //   debug.print(particleSensor.getIR());
// //   debug.print("] G[");
// //   debug.print(particleSensor.getGreen());
// //   debug.print("]");


// //   debug.println();
// // }


// /*
//   Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
//   By: Nathan Seidle @ SparkFun Electronics
//   Date: October 2nd, 2016
//   https://github.com/sparkfun/MAX30105_Breakout

//   This is a demo to show the reading of heart rate or beats per minute (BPM) using
//   a Penpheral Beat Amplitude (PBA) algorithm.

//   It is best to attach the sensor to your finger using a rubber band or other tightening
//   device. Humans are generally bad at applying constant pressure to a thing. When you
//   press your finger against the sensor it varies enough to cause the blood in your
//   finger to flow differently which causes the sensor readings to go wonky.

//   Hardware Connections (Breakoutboard to Arduino):
//   -5V = 5V (3.3V is allowed)
//   -GND = GND
//   -SDA = A4 (or SDA)
//   -SCL = A5 (or SCL)
//   -INT = Not connected

//   The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
//   but it will also run at 3.3V.
// */

// #include <Wire.h>
// #include "heartRate.h"


// MAX30105 particleSensor;

// const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
// byte rates[RATE_SIZE]; //Array of heart rates
// byte rateSpot = 0;
// long lastBeat = 0; //Time at which the last beat occurred

// float beatsPerMinute;
// int beatAvg;

// void setup()
// {
//   Serial.begin(115200);
//   Serial.println("Initializing...");

//   // Initialize sensor
//   if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
//   {
//     Serial.println("MAX30105 was not found. Please check wiring/power. ");
//     while (1);
//   }
//   Serial.println("Place your index finger on the sensor with steady pressure.");

//   particleSensor.setup(); //Configure sensor with default settings
//   particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
//   particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
// }

// void loop()
// {
//   long irValue = particleSensor.getIR();

//   if (checkForBeat(irValue) == true)
//   {
//     //We sensed a beat!
//     long delta = millis() - lastBeat;
//     lastBeat = millis();

//     beatsPerMinute = 60 / (delta / 1000.0);

//     if (beatsPerMinute < 255 && beatsPerMinute > 20)
//     {
//       rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
//       rateSpot %= RATE_SIZE; //Wrap variable

//       //Take average of readings
//       beatAvg = 0;
//       for (byte x = 0 ; x < RATE_SIZE ; x++)
//         beatAvg += rates[x];
//       beatAvg /= RATE_SIZE;
//     }
//   }

//   Serial.print("IR=");
//   Serial.print(irValue);
//   // Serial.print(", BPM=");
//   // Serial.print(beatsPerMinute);
//   // Serial.print(", Avg BPM=");
//   // Serial.print(beatAvg);

//   // if (irValue < 50000)
//   //   Serial.print(" No finger?");

//   Serial.println();
// }

