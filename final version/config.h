#pragma once

// ================================================================
//  MICROMOUSE CONFIG — shared across all phases
//  Tune values marked with [TUNE] during each test phase
// ================================================================

// ================== Motor Pins ==================
#define PWMA        32
#define AIN1        25
#define AIN2        33
#define PWMB        12
#define BIN1        14
#define BIN2        27
#define STBY        26

// ================== Encoder Pins ==================
#define ENC_LA      18    // Left encoder channel A
#define ENC_LB      19    // Left encoder channel B
#define ENC_RA       5    // Right encoder channel A
#define ENC_RB       4    // Right encoder channel B

// ================== IR Sensor Pins ==================
#define IR_RIGHT_PIN  36  // GP2Y0D120X (2D120X) — right
#define IR_CENTER_PIN 39  // GP2Y0A21   (2Y0A21) — center (longer range)
#define IR_LEFT_PIN   35  // GP2Y0D120X (2D120X) — left

// ================== MPU6050 ==================
#define MPU_SDA     21
#define MPU_SCL     22
#define MPU_ADDR    0x68  // AD0 low; use 0x69 if AD0 is pulled high

// ================== Encoder / Odometry ==================
// [TUNE] after Phase 1c encoder test
// CPR = counts per revolution at the WHEEL (after gearbox)
// To find: mark wheel, rotate exactly once, read count printed to serial
#define ENCODER_CPR         360.0f   // placeholder — tune this first!
#define WHEEL_DIAMETER_MM    32.0f   // [TUNE] measure your wheel
#define WHEEL_BASE_MM        85.0f   // [TUNE] distance between wheel centers

// Derived — do not edit
#define MM_PER_COUNT  (PI * WHEEL_DIAMETER_MM / ENCODER_CPR)

// ================== PWM (ESP32 LEDC) ==================
#define PWM_FREQ        20000   // 20 kHz — above audible range
#define PWM_RESOLUTION  8       // 8-bit → 0–255
#define PWM_CH_A        0       // LEDC channel for motor A
#define PWM_CH_B        1       // LEDC channel for motor B

// ================== Maze ==================
#define MAZE_SIZE       16      // 16×16 standard; change to 8 for half-size
#define GOAL_ROW        7       // center cell row  (0-indexed)
#define GOAL_COL        7       // center cell col

// ================== Touch Sensor (sprint trigger) ==================
#define TOUCH_PIN       T0      // GPIO4 — change if occupied
#define TOUCH_THRESHOLD 40      // [TUNE] lower = more sensitive

// ================== Debug ==================
#define SERIAL_BAUD     115200