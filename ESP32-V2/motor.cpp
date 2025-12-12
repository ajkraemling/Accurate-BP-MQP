#include <Arduino.h>
#include <Wire.h>

int pin = 9;


void setup() {
    pinMode(pin, OUTPUT);
}

void loop(){
    digitalWrite(pin, LOW);
    delay(500);
    digitalWrite(pin, HIGH);
    delay(500);
}
