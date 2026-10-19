#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// ================= WIFI =================
const char *ssid = "YOUR_WIFI_SSID";
const char *password = "YOUR_WIFI_PASSWORD";

// ================= IR =================
const int irPins[3] = {36, 39, 35};
int rawValues[3];
float voltages[3];
float distances[3];

// ================= MOTOR =================
#define PWMA 32
#define AIN1 25
#define AIN2 33

#define PWMB 12
#define BIN1 14
#define BIN2 27

#define STBY 26

// ================= ENCODERS =================
#define ENC_LA 18
#define ENC_LB 19
#define ENC_RA 5
#define ENC_RB 4

#define LEFT_DIR_SIGN   1
#define RIGHT_DIR_SIGN  1

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ================= MPU =================
int16_t ax, ay, az, gx, gy, gz;
float rateZ = 0, angleZ = 0, gyroOffset = 0;

// ================= PID =================
float Kp_dist = 0.5, Ki_dist = 0.0, Kd_dist = 0.1;
float Kp_head = 2.0, Ki_head = 0.0, Kd_head = 0.2;

float distIntegral = 0, lastDistError = 0;
float headIntegral = 0, lastHeadError = 0;

// ================= TIMING =================
unsigned long lastTime;
float dt;
unsigned long lastPrint = 0;

// ================= ROBOT CONFIG =================
const float wheelCircumference = 20.42;
const int ticksPerRev = 360;
const float cmPerTick = wheelCircumference / ticksPerRev;

long targetTicks;

// ================= HELPERS =================
int readStable(int pin) {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delay(2);  // small smoothing delay
  }
  return sum / 10;
}

float voltageToDistance_GP2Y0A21(float v) {
  if (v > 0.42) return 27.728 / (v - 0.42);
  else return 80;
}

float voltageToDistance_GP2D120(float v) {
  if (v > 0.1) return 13.0 / v;
  else return 30;
}

// ================= ENCODER ISR =================
void IRAM_ATTR leftEncoderISR() {
  bool a = digitalRead(ENC_LA);
  bool b = digitalRead(ENC_LB);
  if (a == b) leftTicks += LEFT_DIR_SIGN;
  else        leftTicks -= LEFT_DIR_SIGN;
}

void IRAM_ATTR rightEncoderISR() {
  bool a = digitalRead(ENC_RA);
  bool b = digitalRead(ENC_RB);
  if (a == b) rightTicks += RIGHT_DIR_SIGN;
  else        rightTicks -= RIGHT_DIR_SIGN;
}

// ================= MOTOR =================
void setMotor(int pwmPin, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // WIFI + OTA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.begin();

  Serial.print("OTA Ready | IP: ");
  Serial.println(WiFi.localIP());

  // MPU
  Wire.begin();
  mpu.initialize();

  analogSetAttenuation(ADC_11db);

  // Encoders
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), rightEncoderISR, RISING);

  // Motors
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Gyro calibration
  delay(3000);
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroOffset = sum / 1000.0;

  targetTicks = 50.0 / cmPerTick;
  lastTime = micros();
}

// ================= LOOP =================
void loop() {

  // OTA handler (important)
  ArduinoOTA.handle();

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // MPU
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  rateZ = (gz - gyroOffset) / 131.0;
  angleZ += rateZ * dt;

  // IR
  for (int i = 0; i < 3; i++) {
    rawValues[i] = readStable(irPins[i]);
    voltages[i] = rawValues[i] * (3.3 / 4095.0);

    if (i == 1)
      distances[i] = voltageToDistance_GP2Y0A21(voltages[i]);
    else
      distances[i] = voltageToDistance_GP2D120(voltages[i]);
  }

  // PID
  long avgTicks = (leftTicks + rightTicks) / 2;
  long distError = targetTicks - avgTicks;

  distIntegral += distError * dt;
  float distDerivative = (distError - lastDistError) / dt;
  float speedCmd = Kp_dist * distError + Ki_dist * distIntegral + Kd_dist * distDerivative;
  lastDistError = distError;

  float headingError = -angleZ;
  headIntegral += headingError * dt;
  float headDerivative = (headingError - lastHeadError) / dt;
  float turnCmd = Kp_head * headingError + Ki_head * headIntegral + Kd_head * headDerivative;
  lastHeadError = headingError;

  int baseSpeed = constrain(speedCmd, -150, 150);
  int leftPWM  = constrain(baseSpeed + turnCmd, -255, 255);
  int rightPWM = constrain(baseSpeed - turnCmd, -255, 255);

  setMotor(PWMA, AIN1, AIN2, leftPWM);
  setMotor(PWMB, BIN1, BIN2, rightPWM);

  if (abs(distError) < 10) {
    setMotor(PWMA, AIN1, AIN2, 0);
    setMotor(PWMB, BIN1, BIN2, 0);
  }

  // Debug every 3s
  if (millis() - lastPrint >= 3000) {
    lastPrint = millis();

    Serial.println("------ STATUS ------");
    Serial.print("L: "); Serial.print(leftTicks);
    Serial.print(" R: "); Serial.println(rightTicks);

    Serial.print("Angle: "); Serial.println(angleZ);

    for (int i = 0; i < 3; i++) {
      Serial.print("IR "); Serial.print(i);
      Serial.print(": "); Serial.print(distances[i]);
      Serial.println(" cm");
    }
    Serial.println("--------------------\n");
  }
}