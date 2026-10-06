#include "motor.h"
#include <esp_arduino_version.h>

void motorInit() {
    // Set direction pins as outputs
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    // Initial state: stop
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    // Configure PWM for speed pins
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // ESP32 Arduino core 3.x API
    ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(PWMA, 0);
    ledcWrite(PWMB, 0);
#else
    // ESP32 Arduino core 2.x API
    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWMA, PWM_CHANNEL_LEFT);
    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWMB, PWM_CHANNEL_RIGHT);
    ledcWrite(PWM_CHANNEL_LEFT, 0);
    ledcWrite(PWM_CHANNEL_RIGHT, 0);
#endif
}

static void writeLeftPWM(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PWMA, duty);
#else
    ledcWrite(PWM_CHANNEL_LEFT, duty);
#endif
}

static void writeRightPWM(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PWMB, duty);
#else
    ledcWrite(PWM_CHANNEL_RIGHT, duty);
#endif
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Clamp values to [-PWM_MAX_DUTY, PWM_MAX_DUTY]
    leftSpeed = constrain(leftSpeed, -PWM_MAX_DUTY, PWM_MAX_DUTY);
    rightSpeed = constrain(rightSpeed, -PWM_MAX_DUTY, PWM_MAX_DUTY);

    // Handle inversion
    if (INVERT_LEFT_MOTOR)  leftSpeed = -leftSpeed;
    if (INVERT_RIGHT_MOTOR) rightSpeed = -rightSpeed;

    // Left Motor Direction & PWM
    if (leftSpeed > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        writeLeftPWM(leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        writeLeftPWM(-leftSpeed);
    } else {
        // Active Brake at 0 speed
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, HIGH);
        writeLeftPWM(0);
    }

    // Right Motor Direction & PWM
    if (rightSpeed > 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        writeRightPWM(rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        writeRightPWM(-rightSpeed);
    } else {
        // Active Brake at 0 speed
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, HIGH);
        writeRightPWM(0);
    }
}

void motorForward(int speed) {
    setMotorSpeeds(speed, speed);
}

void motorBackward(int speed) {
    setMotorSpeeds(-speed, -speed);
}

void motorTurnLeft(int speed) {
    setMotorSpeeds(-speed, speed);
}

void motorTurnRight(int speed) {
    setMotorSpeeds(speed, -speed);
}

void motorBrake() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    writeLeftPWM(0);
    writeRightPWM(0);
}

void motorCoast() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    writeLeftPWM(0);
    writeRightPWM(0);
}
