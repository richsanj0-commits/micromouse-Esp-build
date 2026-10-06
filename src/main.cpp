#include <Arduino.h>
#include "pinout.h"
#include "motor.h"

// =============================================================================
// MICROMOUSE MOTOR TEST PROGRAM
// =============================================================================

// Default test speed (0 - 255)
int testSpeed = 150;

// State machine for automated forward/backward sequence
enum TestState {
    STATE_IDLE,
    STATE_ARMED,
    STATE_FORWARD,
    STATE_PAUSE1,
    STATE_BACKWARD,
    STATE_PAUSE2,
    STATE_COMPLETE
};

TestState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long lastLedToggle = 0;
bool ledState = false;

// Button debounce helpers
bool lastBtn1State = HIGH;
bool lastBtn2State = HIGH;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
const unsigned long debounceDelay = 50;

void printMenu() {
    Serial.println("\n==============================================");
    Serial.println("       MICROMOUSE ESP32 - MOTOR TEST          ");
    Serial.println("==============================================");
    Serial.println("Hardware Pin Mapping:");
    Serial.println("  Left Motor  : PWMA=27, AIN1=14, AIN2=26");
    Serial.println("  Right Motor : PWMB=32, BIN1=25, BIN2=33");
    Serial.println("  Buttons     : BTN1=GPIO4, BTN2=GPIO13");
    Serial.println("  Status LED  : GPIO2");
    Serial.println("----------------------------------------------");
    Serial.printf("Current Test Speed: %d / 255\n", testSpeed);
    Serial.println("Commands (via Serial Monitor):");
    Serial.println("  't' : Run Auto Test (Forward -> Brake -> Backward -> Stop)");
    Serial.println("  'f' : Drive Forward");
    Serial.println("  'b' : Drive Backward");
    Serial.println("  'l' : Turn Left");
    Serial.println("  'r' : Turn Right");
    Serial.println("  's' : Stop / Dynamic Brake");
    Serial.println("  '+' : Increase Speed by 15");
    Serial.println("  '-' : Decrease Speed by 15");
    Serial.println("Hardware Buttons:");
    Serial.println("  BTN1 (GPIO4)  : Start Auto Test Sequence");
    Serial.println("  BTN2 (GPIO13) : Emergency STOP");
    Serial.println("==============================================\n");
}

void startAutoTest() {
    Serial.println("\n[AUTO TEST] Arming test sequence in 1 second...");
    currentState = STATE_ARMED;
    stateStartTime = millis();
}

void stopAll() {
    motorBrake();
    currentState = STATE_IDLE;
    digitalWrite(LED_PIN, LOW);
    Serial.println("[MOTORS] STOPPED (Brake Engaged)");
}

void handleSerialCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    switch (cmd) {
        case 't':
        case 'T':
            startAutoTest();
            break;

        case 'f':
        case 'F':
            currentState = STATE_IDLE;
            Serial.printf("[MOTORS] Moving FORWARD at speed %d\n", testSpeed);
            motorForward(testSpeed);
            break;

        case 'b':
        case 'B':
            currentState = STATE_IDLE;
            Serial.printf("[MOTORS] Moving BACKWARD at speed %d\n", testSpeed);
            motorBackward(testSpeed);
            break;

        case 'l':
        case 'L':
            currentState = STATE_IDLE;
            Serial.printf("[MOTORS] Turning LEFT at speed %d\n", testSpeed);
            motorTurnLeft(testSpeed);
            break;

        case 'r':
        case 'R':
            currentState = STATE_IDLE;
            Serial.printf("[MOTORS] Turning RIGHT at speed %d\n", testSpeed);
            motorTurnRight(testSpeed);
            break;

        case 's':
        case 'S':
        case ' ':
            stopAll();
            break;

        case '+':
        case '=':
            testSpeed = min(255, testSpeed + 15);
            Serial.printf("[CONFIG] Speed increased to: %d\n", testSpeed);
            break;

        case '-':
        case '_':
            testSpeed = max(30, testSpeed - 15);
            Serial.printf("[CONFIG] Speed decreased to: %d\n", testSpeed);
            break;

        case 'h':
        case 'H':
        case '?':
            printMenu();
            break;

        default:
            break;
    }
}

void handleButtons() {
    // Read raw button states (Active LOW due to INPUT_PULLUP)
    int reading1 = digitalRead(BTN1);
    int reading2 = digitalRead(BTN2);

    // Button 1 debounce (Start Test)
    if (reading1 != lastBtn1State) {
        lastDebounceTime1 = millis();
    }
    if ((millis() - lastDebounceTime1) > debounceDelay) {
        if (reading1 == LOW && lastBtn1State == HIGH) {
            Serial.println("\n[BUTTON 1 PRESSED] Starting Auto Test...");
            startAutoTest();
        }
    }
    lastBtn1State = reading1;

    // Button 2 debounce (Emergency Stop)
    if (reading2 != lastBtn2State) {
        lastDebounceTime2 = millis();
    }
    if ((millis() - lastDebounceTime2) > debounceDelay) {
        if (reading2 == LOW && lastBtn2State == HIGH) {
            Serial.println("\n[BUTTON 2 PRESSED] Emergency Stop Triggered!");
            stopAll();
        }
    }
    lastBtn2State = reading2;
}

void updateAutoTest() {
    unsigned long now = millis();

    switch (currentState) {
        case STATE_IDLE:
            // Heartbeat blink when idle (1 Hz)
            if (now - lastLedToggle >= 500) {
                lastLedToggle = now;
                ledState = !ledState;
                digitalWrite(LED_PIN, ledState);
            }
            break;

        case STATE_ARMED:
            // Fast blink during arming countdown
            if (now - lastLedToggle >= 100) {
                lastLedToggle = now;
                ledState = !ledState;
                digitalWrite(LED_PIN, ledState);
            }
            if (now - stateStartTime >= 1000) {
                Serial.printf("[AUTO TEST] PHASE 1: FORWARD at speed %d (1.5 sec)\n", testSpeed);
                motorForward(testSpeed);
                digitalWrite(LED_PIN, HIGH);
                currentState = STATE_FORWARD;
                stateStartTime = now;
            }
            break;

        case STATE_FORWARD:
            if (now - stateStartTime >= 1500) {
                Serial.println("[AUTO TEST] PHASE 2: BRAKE PAUSE (0.5 sec)");
                motorBrake();
                digitalWrite(LED_PIN, LOW);
                currentState = STATE_PAUSE1;
                stateStartTime = now;
            }
            break;

        case STATE_PAUSE1:
            if (now - stateStartTime >= 500) {
                Serial.printf("[AUTO TEST] PHASE 3: BACKWARD at speed %d (1.5 sec)\n", testSpeed);
                motorBackward(testSpeed);
                digitalWrite(LED_PIN, HIGH);
                currentState = STATE_BACKWARD;
                stateStartTime = now;
            }
            break;

        case STATE_BACKWARD:
            if (now - stateStartTime >= 1500) {
                Serial.println("[AUTO TEST] PHASE 4: FINAL BRAKE");
                motorBrake();
                digitalWrite(LED_PIN, LOW);
                currentState = STATE_PAUSE2;
                stateStartTime = now;
            }
            break;

        case STATE_PAUSE2:
            if (now - stateStartTime >= 500) {
                Serial.println("[AUTO TEST] Sequence Complete! Robot stopped.\n");
                stopAll();
            }
            break;

        case STATE_COMPLETE:
            stopAll();
            break;
    }
}

void setup() {
    // 1. Initialize Serial Communication
    Serial.begin(115200);
    delay(200);

    // 2. Initialize Status LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // 3. Initialize User Buttons with internal pullups
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

    // 4. Initialize Motor Driver pins and PWM
    motorInit();

    // 5. Print startup diagnostics and instructions
    printMenu();
}

void loop() {
    handleSerialCommands();
    handleButtons();
    updateAutoTest();
}
