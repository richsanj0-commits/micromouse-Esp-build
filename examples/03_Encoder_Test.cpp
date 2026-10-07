#include <Arduino.h>

// --- PIN DEFINITIONS ---
// Motor Driver (TB6612)
#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

// Encoders (Swapped Left/Right & Inverted A/B for correct signs)
#define L_ENC_A 35
#define L_ENC_B 34
#define R_ENC_A 36
#define R_ENC_B 39

// Buttons
#define BTN2 4  // Button to reset encoders

// --- ENCODER VARIABLES ---
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Interrupt Service Routines for Encoders
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void setup() {
  Serial.begin(115200);

  // --- MOTOR PINS FIX (Prevents wheels from being stiff) ---
  // Since we aren't using the motors, if the ESP32 leaves these pins floating,
  // the TB6612 motor driver might go into "brake" mode.
  // Setting them to OUTPUT and LOW disables the driver and lets the wheels coast.
  pinMode(PWMA, OUTPUT); digitalWrite(PWMA, LOW);
  pinMode(AIN1, OUTPUT); digitalWrite(AIN1, LOW);
  pinMode(AIN2, OUTPUT); digitalWrite(AIN2, LOW);
  
  pinMode(PWMB, OUTPUT); digitalWrite(PWMB, LOW);
  pinMode(BIN1, OUTPUT); digitalWrite(BIN1, LOW);
  pinMode(BIN2, OUTPUT); digitalWrite(BIN2, LOW);
  // ---------------------------------------------------------

  // Initialize encoders (Phase A & B)
  pinMode(L_ENC_A, INPUT);
  pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT);
  pinMode(R_ENC_B, INPUT);
  
  // Attach interrupts to Phase A of both encoders
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, CHANGE);

  // Button with internal pullup
  pinMode(BTN2, INPUT_PULLUP);

  Serial.println("==================================================");
  Serial.println("Encoder Calibration Mode (Sensors & Motors Disabled)");
  Serial.println("Streaming live Encoder values...");
  Serial.println("Press Button 2 (GPIO 4) to RESET ENCODERS to 0");
  Serial.println("==================================================");
}

unsigned long lastPrintTime = 0;

void loop() {
  // Check Button 2 (GPIO 4, Active LOW) - RESET ENCODERS
  if (digitalRead(BTN2) == LOW) {
    delay(50); // Debounce
    if (digitalRead(BTN2) == LOW) {
      
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      Serial.println("\n>>> [BTN 2 - GPIO 4] ENCODERS RESET TO 0 <<<\n");
      
      // Wait for button release
      while (digitalRead(BTN2) == LOW) {
        delay(10);
      }
      delay(200);
    }
  }

  // Live print encoder values every 150ms
  if (millis() - lastPrintTime >= 150) {
    lastPrintTime = millis();
    Serial.printf("[Live] Enc L: %6ld | Enc R: %6ld\n", leftEncoderCount, rightEncoderCount);
  }
}
