#include <Arduino.h>

// ==========================================
// ENCODER & BUTTON PINS
// ==========================================
#define L_ENC_A 36   // Left Encoder C1
#define L_ENC_B 39   // Left Encoder C2
#define R_ENC_A 34   // Right Encoder C1
#define R_ENC_B 35   // Right Encoder C2

#define BTN1 4       // Reset Button (Press to reset count to 0)
#define BTN2 13      // Secondary Button
#define LED_PIN 2

// 64-bit atomic tick counters
volatile int64_t leftTicks  = 0;
volatile int64_t rightTicks = 0;

// ==========================================
// QUADRATURE DECODER ISRs
// ==========================================
void IRAM_ATTR isrLeftEncoder() {
    int a = digitalRead(L_ENC_A);
    int b = digitalRead(L_ENC_B);
    if (a == b) {
        leftTicks++;
    } else {
        leftTicks--;
    }
}

void IRAM_ATTR isrRightEncoder() {
    int a = digitalRead(R_ENC_A);
    int b = digitalRead(R_ENC_B);
    if (a == b) {
        rightTicks++;
    } else {
        rightTicks--;
    }
}

// Button state tracking for reset
bool lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // GPIO 34, 35, 36, 39 are input only (using external 4.7k pullups to 3.3V)
    pinMode(L_ENC_A, INPUT);
    pinMode(L_ENC_B, INPUT);
    pinMode(R_ENC_A, INPUT);
    pinMode(R_ENC_B, INPUT);

    // Buttons (active LOW)
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

    // Attach interrupts on Channel A transitions
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), isrLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), isrRightEncoder, CHANGE);

    Serial.println("\n==============================================");
    Serial.println("     MICROMOUSE ENCODER CALIBRATION TEST      ");
    Serial.println("==============================================");
    Serial.println("Instructions:");
    Serial.println("1. Mark a point on each wheel.");
    Serial.println("2. Press BTN1 (GPIO4) to zero the count.");
    Serial.println("3. Rotate the wheel exactly 1 full turn (360 deg).");
    Serial.println("4. Note the tick count displayed in the serial monitor.");
    Serial.println("==============================================\n");
}

void loop() {
    // Check reset button (BTN1) with debounce
    int btnVal = digitalRead(BTN1);
    if (btnVal != lastBtnState) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (btnVal == LOW && lastBtnState == HIGH) {
            noInterrupts();
            leftTicks  = 0;
            rightTicks = 0;
            interrupts();
            Serial.println("\n>>> [RESET] Encoder counters set to 0 <<<\n");
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
        }
    }
    lastBtnState = btnVal;

    // Read volatile values safely
    noInterrupts();
    int64_t currentLeft  = leftTicks;
    int64_t currentRight = rightTicks;
    interrupts();

    // Print values every 100ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.printf("LEFT Ticks: %6lld  |  RIGHT Ticks: %6lld  (Press BTN1 to Reset)\n",
                      currentLeft, currentRight);
    }
}
