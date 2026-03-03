#include "MotorController.h"

MotorController::MotorController()
: inflating(false), deflating(false) {}

#ifdef ARDUINO
void MotorController::begin() {
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    pinMode(SLOW_SOLENOID_IN3_PIN, OUTPUT);
    pinMode(SLOW_SOLENOID_IN4_PIN, OUTPUT);
    pinMode(FAST_SOLENOID_CONTROL, OUTPUT);

    stopInflation();
    stopDeflation();
}

void MotorController::startInflation() {
    stopDeflation();
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);

    inflating = true;
}

void MotorController::stopInflation() {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    inflating = false;
}

void MotorController::startDeflation() {
    stopInflation();
    openSlowSolenoid();

    deflating = true;
}

void MotorController::stopDeflation() {
    closeSlowSolenoid();
    closeFastSolenoid();
    deflating = false;
}

void MotorController::emergencyStop() {
    stopInflation();
    openSlowSolenoid();
    openFastSolenoid();

    inflating = false;
    deflating = true;
}

void MotorController::closeSlowSolenoid() {
    // Its a default open solenoid
    digitalWrite(SLOW_SOLENOID_IN3_PIN, HIGH);
    digitalWrite(SLOW_SOLENOID_IN4_PIN, LOW);
}

void MotorController::openSlowSolenoid() {
    digitalWrite(SLOW_SOLENOID_IN3_PIN, LOW);
    digitalWrite(SLOW_SOLENOID_IN4_PIN, LOW);
}

void MotorController::openFastSolenoid() {
    // Needs to go through a transistor still
    digitalWrite(FAST_SOLENOID_CONTROL, LOW);
}

void MotorController::closeFastSolenoid() {
    // Needs to go through a transistor still
    digitalWrite(FAST_SOLENOID_CONTROL, HIGH);
}

#else
void MotorController::begin() {}
void MotorController::startInflation() {}
void MotorController::stopInflation() {}
void MotorController::startDeflation() {}
void MotorController::stopDeflation() {}
void MotorController::emergencyStop() {}
void MotorController::setMotor(bool on) {}
void MotorController::setMotorDirection(bool forward) {}
void MotorController::closeSlowSolenoid() {}
void MotorController::openSlowSolenoid() {}
void MotorController::openFastSolenoid() {}
void MotorController::closeFastSolenoid() {}
#endif