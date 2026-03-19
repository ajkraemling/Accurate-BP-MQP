#include "MotorController.h"

MotorController::MotorController()
: inflating(false), deflating(false) {}

#ifdef ARDUINO
void MotorController::begin() {
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    pinMode(SOLENOID_IN3_PIN, OUTPUT);
    pinMode(SOLENOID_IN4_PIN, OUTPUT);
    pinMode(4, OUTPUT);

    stopInflation();
    stopDeflation();
}

void MotorController::startInflation() {
    stopDeflation();
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    digitalWrite(4, LOW);

    inflating = true;
}

void MotorController::stopInflation() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    inflating = false;
}

void MotorController::startDeflation() {
    stopInflation();
    digitalWrite(4, HIGH);

    deflating = true;
}

void MotorController::stopDeflation() {
    closeSolenoid();
    deflating = false;
}

void MotorController::emergencyStop() {
    stopInflation();
    openSolenoid();

    inflating = false;
    deflating = true;
}

void MotorController::closeSolenoid() {
    // Its a default open solenoid
    digitalWrite(SOLENOID_IN3_PIN, HIGH);
    digitalWrite(SOLENOID_IN4_PIN, LOW);
}

void MotorController::openSolenoid() {
    digitalWrite(SOLENOID_IN3_PIN, LOW);
    digitalWrite(SOLENOID_IN4_PIN, LOW);
}

#else
void MotorController::begin() {}
void MotorController::startInflation() {}
void MotorController::stopInflation() {}
void MotorController::startDeflation() {}
void MotorController::stopDeflation() {}
void MotorController::emergencyStop() {}
void MotorController::openSolenoid() {}
void MotorController::closeSolenoid() {}
#endif