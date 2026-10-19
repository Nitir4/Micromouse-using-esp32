// ================================================================
//  PHASE 1d — MPU6050 Diagnostics
//  Tests: I2C connection, raw gyro/accel, yaw integration
//
//  LIBRARY REQUIRED:
//  Install via Arduino Library Manager:
//  → "MPU6050" by Electronic Cats  (or "I2Cdev + MPU6050" by jrowberg)
//
//  HOW TO USE:
//  1. Upload, open Serial Monitor @ 115200
//  2. Let robot sit still for 3 seconds (gyro calibration)
//  3. Rotate robot 90° by hand → verify yaw reads ~90°
//  4. Send 'c' to recalibrate gyro bias
//  5. Open Serial Plotter to watch yaw drift over time
//     (acceptable drift: < 2°/minute at rest)
//
//  VERIFY:
//  - Yaw increases CW or CCW consistently (note your convention)
//  - Accel Z ≈ 1.0 g when flat
//  - Gyro XYZ ≈ 0 when stationary (after calibration)
// ================================================================

#include <Wire.h>
#include <MPU6050.h>
#include "config.h"

MPU6050 mpu(MPU_ADDR);

// ----------------------------------------------------------------
//  Gyro calibration
//  Averages N samples at rest to find bias
// ----------------------------------------------------------------
#define CALIB_SAMPLES 500

float gyroBiasZ = 0;  // deg/s bias on Z axis (yaw)
float yaw       = 0;  // integrated yaw angle (degrees)
uint32_t lastUs = 0;

void calibrateGyro() {
    Serial.println("[CAL] Keep robot STILL — calibrating gyro...");
    long sumZ = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sumZ += gz;
        if (i % 50 == 0) Serial.print(".");
        delay(4);
    }
    // MPU6050 default gyro range is ±250 deg/s, sensitivity = 131 LSB/deg/s
    gyroBiasZ = (sumZ / (float)CALIB_SAMPLES) / 131.0f;
    yaw = 0;
    lastUs = micros();
    Serial.printf("\n[CAL] Done. Gyro Z bias: %.4f deg/s\n", gyroBiasZ);
}

// ----------------------------------------------------------------
//  Setup
// ----------------------------------------------------------------
void setup() {
    Serial.begin(SERIAL_BAUD);
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.setClock(400000);  // 400 kHz fast mode

    Serial.println("=== PHASE 1d: MPU6050 Diagnostics ===");
    Serial.print("[INIT] Initializing MPU6050 at 0x");
    Serial.println(MPU_ADDR, HEX);

    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("[ERROR] MPU6050 NOT found! Check wiring.");
        Serial.println("  SDA → GPIO 21,  SCL → GPIO 22");
        Serial.println("  VCC → 3.3V,     GND → GND");
        while (true) delay(500);
    }
    Serial.println("[OK] MPU6050 connected.");

    // Set ranges
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);    // ±250 deg/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);    // ±2 g
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);               // 42Hz low-pass

    calibrateGyro();

    Serial.println("\nCommands:");
    Serial.println("  c → recalibrate gyro");
    Serial.println("  z → zero yaw");
    Serial.println("  r → print raw values");
}

// ----------------------------------------------------------------
//  Loop
// ----------------------------------------------------------------
void loop() {
    // Integrate yaw from gyro Z
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) / 1e6f;
    lastUs = nowUs;

    float gyroZ_dps = (gz / 131.0f) - gyroBiasZ;
    yaw += gyroZ_dps * dt;

    // Convert accel to g
    float aX = ax / 16384.0f;
    float aY = ay / 16384.0f;
    float aZ = az / 16384.0f;

    // Serial Plotter (100Hz)
    static uint32_t lastPlot = 0;
    if (millis() - lastPlot >= 10) {
        lastPlot = millis();
        Serial.print("Yaw:"); Serial.print(yaw, 2);
        Serial.print("\tGyroZ:"); Serial.print(gyroZ_dps, 3);
        Serial.print("\tAccX:"); Serial.print(aX, 3);
        Serial.print("\tAccY:"); Serial.print(aY, 3);
        Serial.print("\tAccZ:"); Serial.println(aZ, 3);
    }

    // Human-readable every 500ms
    static uint32_t lastHuman = 0;
    if (millis() - lastHuman >= 500) {
        lastHuman = millis();
        Serial.println("---");
        Serial.printf("  Yaw:   %7.2f deg\n", yaw);
        Serial.printf("  GyroZ: %7.3f deg/s  (raw: %d)\n", gyroZ_dps, gz);
        Serial.printf("  Accel: X=%.3fg  Y=%.3fg  Z=%.3fg\n", aX, aY, aZ);
    }

    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'c') calibrateGyro();
        if (cmd == 'z') { yaw = 0; Serial.println("[ZERO] Yaw zeroed"); }
        if (cmd == 'r') {
            Serial.printf("[RAW] ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d\n",
                          ax, ay, az, gx, gy, gz);
        }
    }

    delay(5);  // ~200Hz loop max
}
