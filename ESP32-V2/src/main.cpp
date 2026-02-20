#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "sensors.h"
#include "display.h"
#include "DisplayPresenter.h"

#include "MotorController.h"

PressureSensor pressureSensor;
PPGSensor ppgSensor(PPG_PIN);

Display lcd;
DisplayPresenter presenter(&lcd);

MotorController motor;

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Start");

    pressureSensor.begin();
    pressureSensor.calibrate();
    ppgSensor.resetFilter();

    lcd.begin();

    motor.begin();

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Serial.println("Time,Pressure,RawPPG,FilteredPPG,Button");
    // Serial.println("Ready - hold button to inflate");
    Serial.println("Time,Pressure,rawPPGSignal,PPGSignal");
}

void loop()
{
    float pressure    = pressureSensor.readGaugePressure();
    int rawPPG        = ppgSensor.readRaw();
    int filteredPPG   = ppgSensor.read();
    int buttonPressed = !digitalRead(BUTTON_PIN); // LOW when pressed (INPUT_PULLUP)

    if (buttonPressed) {
        motor.startInflation(255);
    } else {
        motor.stopInflation();
    }

    Serial.print(millis());
    Serial.print(",");
    Serial.print(pressure, 2);
    Serial.print(",");
    Serial.print(rawPPG);
    Serial.print(",");
    Serial.print(filteredPPG);
    Serial.print(",");
    Serial.println(buttonPressed);

    delay(SAMPLE_RATE_MS);
}