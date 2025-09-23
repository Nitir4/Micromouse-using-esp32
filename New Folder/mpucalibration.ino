#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

float angleZ = 0;   // integrated yaw angle
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("Calibrating gyro for 3 seconds...");
  delay(3000);
  calibrate();
  Serial.println("Calibration done.");

  lastTime = micros();
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Subtract offsets
  int16_t gz_rel = gz - gz_offset;

  // Convert raw gyro to deg/sec (MPU6050 scale: 131 LSB/°/s at ±250dps)
  float rateZ = gz_rel / 131.0;

  // Integrate angle
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  angleZ += rateZ * dt;  // angle in degrees

  // Print values (use rateZ in PID)
  Serial.print("Yaw Rate (deg/s): ");
  Serial.print(rateZ);
  Serial.print(" | Angle: ");
  Serial.println(angleZ);

  delay(10);
}

void calibrate() {
  const int samples = 500;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(5);
  }

  gx_offset = gx_sum / samples;
  gy_offset = gy_sum / samples;
  gz_offset = gz_sum / samples;
}
