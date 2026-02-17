#include "MotorController.h"

MotorController::MotorController()
: inflating(false), deflating(false), motorSpeed(0), valveOpening(0) {}
#ifdef ARDUINO
void MotorController::begin() {

    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    pinMode(SOLENOID_IN3_PIN, OUTPUT);
    pinMode(SOLENOID_IN4_PIN, OUTPUT);

    ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(MOTOR_ENA_PIN, MOTOR_PWM_CHANNEL);

    ledcSetup(SOLENOID_PWM_CHANNEL, SOLENOID_PWM_FREQ, SOLENOID_PWM_RES);
    ledcAttachPin(SOLENOID_ENB_PIN, SOLENOID_PWM_CHANNEL);

    emergencyStop();
}

void MotorController::startInflation(int speed) {
    // if (inflating) return;

    stopDeflation();
    setMotorDirection(true);
    setMotorSpeed(speed);
    // inflating = true;
}

void MotorController::stopInflation() {
    // if (!inflating) return;
    setMotorSpeed(0);
    // inflating = false;
}

void MotorController::startDeflation(int rate) {
    // if (deflating) return;

    stopInflation();
    setSolenoidOpening(rate);
    // deflating = true;
}

void MotorController::stopDeflation() {
    // if (!deflating) return;
    setSolenoidOpening(0);
    // deflating = false;
}

void MotorController::emergencyStop() {
    setMotorSpeed(0);
    setSolenoidOpening(255);
    // inflating = false;
    // deflating = false;
}

void MotorController::setMotorSpeed(int speed) {
    motorSpeed = constrain(speed, 0, 255);
    ledcWrite(MOTOR_PWM_CHANNEL, motorSpeed);
}

void MotorController::setMotorDirection(bool forward) {
    digitalWrite(MOTOR_IN1_PIN, forward ? HIGH : LOW);
    digitalWrite(MOTOR_IN2_PIN, forward ? LOW : HIGH);
}

void MotorController::setSolenoidOpening(int opening) {

    valveOpening = constrain(opening, 0, 255);

    if (valveOpening == 0) {
        digitalWrite(SOLENOID_IN3_PIN, LOW);
        digitalWrite(SOLENOID_IN4_PIN, LOW);
        ledcWrite(SOLENOID_PWM_CHANNEL, 0);
    } else {
        digitalWrite(SOLENOID_IN3_PIN, HIGH);
        digitalWrite(SOLENOID_IN4_PIN, LOW);
        ledcWrite(SOLENOID_PWM_CHANNEL, valveOpening);
    }
}
#else
void MotorController::begin() {
    // Empty
}

void MotorController::startInflation(int speed) {
    // Empty
}

void MotorController::stopInflation() {
    // Empty
}

void MotorController::startDeflation(int rate) {
    // Empty
}

void MotorController::stopDeflation() {
    // Empty
}

void MotorController::emergencyStop() {
    // Empty
}

void MotorController::setMotorSpeed(int speed) {
    // Empty
}

void MotorController::setMotorDirection(bool forward) {
    // Empty
}

void MotorController::setSolenoidOpening(int opening) {
    // Empty
}
#endif