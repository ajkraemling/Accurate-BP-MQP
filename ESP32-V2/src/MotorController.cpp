#include "MotorController.h"

MotorController::MotorController()
: inflating(false), deflating(false), motorSpeed(0), valveOpening(0) {}
#ifdef ARDUINO
void MotorController::begin() {

    pinMode(2, OUTPUT);
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
    closeSlowSolenoid();
    closeFastSolenoid();

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
    // Cap rate so valve never fully de-energizes and dumps too fast
    // Tune this max value - lower = slower deflation
    openSlowSolenoid();
}

void MotorController::stopDeflation() {
    // if (!deflating) return;
    closeSlowSolenoid();
    closeFastSolenoid();
    // deflating = false;
}

void MotorController::emergencyStop() {
    setMotorSpeed(0);
    openSlowSolenoid();
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

void MotorController::openSlowSolenoid() {
    // Partially/fully OPEN - reduce power to let valve open proportionally
    digitalWrite(SOLENOID_IN3_PIN, LOW);
    digitalWrite(SOLENOID_IN4_PIN, LOW);
    ledcWrite(SOLENOID_PWM_CHANNEL, 0);
}

void MotorController::closeSlowSolenoid() {
    // Partially/fully OPEN - reduce power to let valve open proportionally
    digitalWrite(SOLENOID_IN3_PIN, HIGH);
    digitalWrite(SOLENOID_IN4_PIN, LOW);
    ledcWrite(SOLENOID_PWM_CHANNEL, 255);
}

void MotorController::openFastSolenoid() {
    digitalWrite(2, 0);
}

void MotorController::closeFastSolenoid() {
    digitalWrite(2, 1);
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