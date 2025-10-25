#include <Arduino.h>

int retVal = true;

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   pinMode(15, OUTPUT); //initalize LED with Pin Location
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(15, HIGH); //LED turns on
  delay(100);
  digitalWrite(15,LOW); //LED turns off
  delay(100);
}
