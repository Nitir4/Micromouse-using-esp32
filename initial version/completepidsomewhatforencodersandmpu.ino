#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// --- WiFi credentials ---
const char* ssid = "ESP32_MPU";
const char* password = "12345678";

// --- Web server ---
WebServer server(80);

// --- Motor pins (TB6612FNG) ---
#define PWMA 32   // Left Motor PWM
#define AIN1 25   // Left Motor IN1
#define AIN2 33   // Left Motor IN2

#define PWMB 12   // Right Motor PWM
#define BIN1 14   // Right Motor IN1
#define BIN2 27   // Right Motor IN2

#define STBY 26   // Standby pin

// --- Encoders ---
#define ENC_LA 18  // Left Encoder A
#define ENC_LB 19  // Left Encoder B
#define ENC_RA 5   // Right Encoder A  (strap pin; OK to use, avoid forcing levels at boot)
#define ENC_RB 4   // Right Encoder B

// If a wheel counts backward, flip the sign here (1 or -1)
#define LEFT_DIR_SIGN   1
#define RIGHT_DIR_SIGN  1

// --- Encoder counters ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// --- MPU values ---
int16_t ax, ay, az, gx, gy, gz;
float rateZ = 0, angleZ = 0, gyroOffset = 0;

// --- Timing ---
unsigned long lastTime;
float dt;

// --- PID variables ---
float Kp_dist = 0.5, Ki_dist = 0.0, Kd_dist = 0.1;
float Kp_head = 2.0, Ki_head = 0.0, Kd_head = 0.2;

float distIntegral = 0, lastDistError = 0;
float headIntegral = 0, lastHeadError = 0;

// --- Robot config ---
const float wheelCircumference = 20.42;  // cm
const int ticksPerRev = 360;             // encoder spec
const float cmPerTick = wheelCircumference / ticksPerRev;

long targetTicks;

// ================== Encoder ISRs (correct quadrature) ===================
// Trigger on A's RISING edge; decide direction by comparing A and B.
// If A == B -> one direction; else the other. If reversed, flip *_DIR_SIGN above.
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

// ================== Motor helper ===================
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

// ================== Web handlers ===================
void handleRoot() {
  // Atomic-ish snapshot for printing (32-bit transfers on ESP32 are single-cycle; good enough)
  long l = leftTicks, r = rightTicks;

  String html = "<html><body><h2>ESP32 PID Control</h2>";
  html += "Ticks L: " + String(l) + " R: " + String(r) + "<br>";
  html += "Angle: " + String(angleZ, 2) + "<br>";
  html += "Kp_dist: " + String(Kp_dist) + " Ki_dist: " + String(Ki_dist) + " Kd_dist: " + String(Kd_dist) + "<br>";
  html += "Kp_head: " + String(Kp_head) + " Ki_head: " + String(Ki_head) + " Kd_head: " + String(Kd_head) + "<br>";
  html += "<form action='/setpid'>"
          "dist Kp:<input name='kpd' value='" + String(Kp_dist) + "'><br>"
          "dist Ki:<input name='kid' value='" + String(Ki_dist) + "'><br>"
          "dist Kd:<input name='kdd' value='" + String(Kd_dist) + "'><br>"
          "head Kp:<input name='kph' value='" + String(Kp_head) + "'><br>"
          "head Ki:<input name='kih' value='" + String(Ki_head) + "'><br>"
          "head Kd:<input name='kdh' value='" + String(Kd_head) + "'><br>"
          "<input type='submit' value='Update'>"
          "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleSetPID() {
  if (server.hasArg("kpd")) Kp_dist = server.arg("kpd").toFloat();
  if (server.hasArg("kid")) Ki_dist = server.arg("kid").toFloat();
  if (server.hasArg("kdd")) Kd_dist = server.arg("kdd").toFloat();
  if (server.hasArg("kph")) Kp_head = server.arg("kph").toFloat();
  if (server.hasArg("kih")) Ki_head = server.arg("kih").toFloat();
  if (server.hasArg("kdh")) Kd_head = server.arg("kdh").toFloat();
  handleRoot();
}

// ================== Setup ===================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  // Encoders
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);

  // Trigger on RISING edge of A only (stable, half the ISR rate)
  attachInterrupt(digitalPinToInterrupt(ENC_LA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), rightEncoderISR, RISING);

  // Motors
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // enable motor driver

  // --- WiFi Access Point ---
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Web server routes
  server.on("/", handleRoot);
  server.on("/setpid", handleSetPID);
  server.begin();

  // Gyro calibration
  Serial.println("Hold still for gyro calibration...");
  delay(3000);
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroOffset = sum / 1000.0;
  Serial.println("Gyro calibrated!");

  // Target distance
  targetTicks = 50.0 / cmPerTick;
  lastTime = micros();
}

// ================== Main Loop ===================
void loop() {
  server.handleClient();

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // --- Gyro integration ---
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  rateZ = (gz - gyroOffset) / 131.0;
  angleZ += rateZ * dt;

  // --- Distance PID ---
  long avgTicks = (leftTicks + rightTicks) / 2;
  long distError = targetTicks - avgTicks;

  distIntegral += distError * dt;
  float distDerivative = (distError - lastDistError) / dt;
  float speedCmd = Kp_dist * distError + Ki_dist * distIntegral + Kd_dist * distDerivative;
  lastDistError = distError;

  // --- Heading PID ---
  float headingError = 0 - angleZ;
  headIntegral += headingError * dt;
  float headDerivative = (headingError - lastHeadError) / dt;
  float turnCmd = Kp_head * headingError + Ki_head * headIntegral + Kd_head * headDerivative;
  lastHeadError = headingError;

  // --- Combine ---
  int baseSpeed = constrain(speedCmd, -150, 150);
  int leftPWM  = constrain(baseSpeed + turnCmd, -255, 255);
  int rightPWM = constrain(baseSpeed - turnCmd, -255, 255);

  setMotor(PWMA, AIN1, AIN2, leftPWM);
  setMotor(PWMB, BIN1, BIN2, rightPWM);

  // --- Stop when reached ---
  if (abs(distError) < 10) {
    setMotor(PWMA, AIN1, AIN2, 0);
    setMotor(PWMB, BIN1, BIN2, 0);
    Serial.println("Target reached.");
    while (1) { server.handleClient(); }  // keep WiFi alive
  }
}
