#include <Arduino.h>

const int ENA = 23;
const int IN1 = 22;
const int IN2 = 21;

const int pwmChan = 0;
const int pwmFreq = 20000;   // 20 kHz
const int pwmRes  = 8;       // 0-255

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  ledcSetup(pwmChan, pwmFreq, pwmRes);
  ledcAttachPin(ENA, pwmChan);
  ledcWrite(pwmChan, 200);   // ~78% duty
}

void loop() {}
