#pragma once

#include <Arduino.h>
#include "pinout.h"

// =============================================================================
// MOTOR DRIVER CONFIGURATION & CONTROL
// =============================================================================

// PWM Configuration
// 20 kHz ultrasonic frequency eliminates motor whine
#define PWM_FREQ         20000
#define PWM_RESOLUTION   8       // 8-bit duty resolution: 0 to 255
#define PWM_MAX_DUTY     255

// LEDC Channels (for ESP32 core 2.x)
#define PWM_CHANNEL_LEFT  0
#define PWM_CHANNEL_RIGHT 1

// Motor direction inversion flags
// Set to true if a motor spins backwards when told to go forward
#define INVERT_LEFT_MOTOR  false
#define INVERT_RIGHT_MOTOR false

/**
 * @brief Initialize GPIOs and PWM channels for TB6612 motor driver.
 */
void motorInit();

/**
 * @brief Set individual motor speeds (-255 to +255).
 *        Positive = Forward, Negative = Backward, 0 = Stop.
 * 
 * @param leftSpeed  Duty/speed for left motor (-255 to 255)
 * @param rightSpeed Duty/speed for right motor (-255 to 255)
 */
void setMotorSpeeds(int leftSpeed, int rightSpeed);

/**
 * @brief Drive both motors forward at the specified speed.
 * 
 * @param speed Speed duty cycle (0 to 255)
 */
void motorForward(int speed);

/**
 * @brief Drive both motors backward at the specified speed.
 * 
 * @param speed Speed duty cycle (0 to 255)
 */
void motorBackward(int speed);

/**
 * @brief Turn left in place (left motor backward, right motor forward).
 * 
 * @param speed Speed duty cycle (0 to 255)
 */
void motorTurnLeft(int speed);

/**
 * @brief Turn right in place (left motor forward, right motor backward).
 * 
 * @param speed Speed duty cycle (0 to 255)
 */
void motorTurnRight(int speed);

/**
 * @brief Dynamic brake - shorts motor terminals through low-side FETs.
 *        Stops the robot immediately with resistance.
 */
void motorBrake();

/**
 * @brief Coast/float - disconnects motor terminals.
 *        Allows wheels to spin freely to a stop.
 */
void motorCoast();
