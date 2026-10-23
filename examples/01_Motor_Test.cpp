#include <Arduino.h>

// --- PIN DEFINITIONS ---
#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

#define BTN1 13
#define BTN2 4

const int pwmFreq = 5000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmResolution = 8; // 0-255

void setupMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);

  ledcAttachPin(PWMA, pwmChannelLeft);
  ledcAttachPin(PWMB, pwmChannelRight);
}

void setMotorLeft(int speed) {
  if (speed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(pwmChannelLeft, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(pwmChannelLeft, -speed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(pwmChannelLeft, 0);
  }
}

void setMotorRight(int speed) {
  if (speed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(pwmChannelRight, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(pwmChannelRight, -speed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(pwmChannelRight, 0);
  }
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

void setup() {
  Serial.begin(115200);
  setupMotors();
  stopMotors();

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  Serial.println("Motor Test Ready! Btn1=FWD, Btn2=BWD");
}

void loop() {
  if (digitalRead(BTN1) == LOW) {
    delay(50);
    if (digitalRead(BTN1) == LOW) {
      Serial.println("Moving Forward for 5s...");
      setMotorLeft(120);
      setMotorRight(120);
      delay(5000);
      stopMotors();
      while (digitalRead(BTN1) == LOW) delay(10);
    }
  }

  if (digitalRead(BTN2) == LOW) {
    delay(50);
    if (digitalRead(BTN2) == LOW) {
      Serial.println("Moving Backward for 5s...");
      setMotorLeft(-120);
      setMotorRight(-120);
      delay(5000);
      stopMotors();
      while (digitalRead(BTN2) == LOW) delay(10);
    }
  }
}
