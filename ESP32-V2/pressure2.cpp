#include <Arduino.h>

#define ADC_PIN 4     // Use GPIO34 for analog input
#define VCC 5.0        // Sensor is powered at 5V
#define DIVIDER_SCALE 1.5555   // (R1+R2)/R2 with 100k + 180k

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);   // ESP32 ADC: 0–4095
}

void loop() {
    int adcRaw = analogRead(ADC_PIN);
    float vAdc = (adcRaw / 4095.0) * 3.3;       // Voltage read at ESP32 pin
    float vSensor = vAdc * DIVIDER_SCALE;      // Reconstruct actual sensor output

    // Convert voltage → pressure (kPa)
    float pressure_kPa = (vSensor / VCC - 0.04) / 0.09;

    // Optional: convert to mmHg (for blood pressure)
    float pressure_mmHg = pressure_kPa * 7.50062;

    Serial.print("ADC Raw: ");
    Serial.print(adcRaw);
    Serial.print(" | Sensor Voltage: ");
    Serial.print(vSensor, 3);
    Serial.print(" V | Pressure: ");
    Serial.print(pressure_kPa, 2);
    Serial.print(" kPa | ");
    Serial.print(pressure_mmHg, 1);
    Serial.println(" mmHg");

    delay(200);
}
