#include <Arduino.h>

const int ENA = 23;
const int IN1 = 21;
const int IN2 = 22;

const int pwmChan = 0;
const int pwmFreq = 20000;
const int pwmRes  = 8;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Air Pump Test Starting ===");
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  // Test 1: Suck air (reverse direction)
  Serial.println("Test 1: SUCK air");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);  // Changed from HIGH
  digitalWrite(ENA, HIGH);
  delay(3000);
  
  digitalWrite(ENA, LOW);
  Serial.println("Pump stopped");
  delay(2000);
  
  // Test 2: Blow air (forward direction)
  Serial.println("Test 2: BLOW air");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(ENA, HIGH);
  delay(3000);
  
  digitalWrite(ENA, LOW);
  delay(2000);
  
  // Test 3: PWM control (blowing)
  Serial.println("Test 3: PWM blow at 200/255");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcSetup(pwmChan, pwmFreq, pwmRes);
  ledcAttachPin(ENA, pwmChan);
  ledcWrite(pwmChan, 200);
  
  Serial.println("=== Test Complete ===");
}

void loop() {}