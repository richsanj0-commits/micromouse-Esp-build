#include <Arduino.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

#define L_ENC_A 36
#define L_ENC_B 39
#define R_ENC_A 34
#define R_ENC_B 35

#define I2C_SDA 21
#define I2C_SCL 22

#define XSHUT_L  23
#define XSHUT_LF 19
#define XSHUT_F  18
#define XSHUT_RF 17
#define XSHUT_R  16

#define BTN1 4
#define BTN2 13

// PWM Channels for ESP32
#define CH_LEFT  0
#define CH_RIGHT 1

// ==========================================
// MOTOR FUNCTIONS
// ==========================================
void forward(int speed) {
    // Left motor forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(CH_LEFT, speed);

    // Right motor forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(CH_RIGHT, speed);
}

void backward(int speed) {
    // Left motor backward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(CH_LEFT, speed);

    // Right motor backward
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(CH_RIGHT, speed);
}

void stopMotors() {
    // Active brake
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    ledcWrite(CH_LEFT, 0);
    ledcWrite(CH_RIGHT, 0);
}

// ==========================================
// SETUP & LOOP
// ==========================================
void setup() {
    Serial.begin(115200);

    // Direction pins
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    // Buttons
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

    // Configure 20kHz PWM on ESP32
    ledcSetup(CH_LEFT, 20000, 8);
    ledcAttachPin(PWMA, CH_LEFT);

    ledcSetup(CH_RIGHT, 20000, 8);
    ledcAttachPin(PWMB, CH_RIGHT);

    stopMotors();
    Serial.println("ESP32 Micromouse Ready! Starting in 2 seconds...");
    delay(2000);
}

void loop() {
    Serial.println("Moving FORWARD...");
    forward(150);      // Speed between 0 and 255
    delay(2000);

    Serial.println("STOP");
    stopMotors();
    delay(1000);

    Serial.println("Moving BACKWARD...");
    backward(150);
    delay(2000);

    Serial.println("STOP");
    stopMotors();
    delay(2000);
}
