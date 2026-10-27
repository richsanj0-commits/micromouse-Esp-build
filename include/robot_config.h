// =========================================================================
// ROBOT CONFIG - measured / CAD values shared by all sketches
// =========================================================================
#pragma once
#include <Arduino.h>

// --- ENCODERS (4x quadrature, verified on robot 2026-10-27) ---
#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 36
#define R_ENC_B 39
#define L_ENC_DIR  1   // forward = positive (checked with motors driving, 2026-10-27)
#define R_ENC_DIR -1

const float CPR_LEFT  = 2772.1f;   // counts per wheel revolution
const float CPR_RIGHT = 2772.3f;

// --- GEOMETRY (from cad/micromouse_assembly.step) ---
const float WHEEL_DIAMETER_MM = 34.0f;   // CAD value
const float TRACK_WIDTH_MM    = 75.3f;   // wheel centre to centre
const float CELL_SIZE_MM      = 180.0f;

// --- DERIVED ---
const float MM_PER_COUNT_L = PI * WHEEL_DIAMETER_MM / CPR_LEFT;    // ~0.0385
const float MM_PER_COUNT_R = PI * WHEEL_DIAMETER_MM / CPR_RIGHT;
const float COUNTS_PER_CELL = CELL_SIZE_MM / (PI * WHEEL_DIAMETER_MM) * CPR_LEFT;  // ~4672
const float COUNTS_PER_90_TURN = (PI * TRACK_WIDTH_MM / 4.0f) / (PI * WHEEL_DIAMETER_MM) * CPR_LEFT;  // ~1535
