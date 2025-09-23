#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050.h>

// ================== Motor pins ==================
#define PWMA 32
#define AIN1 25
#define AIN2 33
#define PWMB 12
#define BIN1 14
#define BIN2 27
#define STBY 26

// ================== Encoder pins ==================
#define ENC_LA 18
#define ENC_LB 19
#define ENC_RA 5
#define ENC_RB 4

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ================== MPU ==================
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float gyroOffset = 0;
float angleZ = 0;
float rateZ = 0;

// ================== Timing ==================
unsigned long lastTime;
float dt;

// ================== Robot config ==================
const int ticksPerRev = 600;
const float wheelDiameter_cm = 2.3;
const float wheelCircumference_cm = 3.14159 * wheelDiameter_cm;
const float cmPerTick = wheelCircumference_cm / (float)ticksPerRev;

// ================== PID variables ==================
float Kp_dist = 2.0;
float Kp_head = 1.5;

// ================== WiFi and server ==================
const char* ssid = "ESP32_MPU";
const char* password = "12345678";
WebServer server(80);

// ================== Encoder ISRs ==================
void IRAM_ATTR leftEncoderISR() {
  int b = digitalRead(ENC_LB);
  if (b == HIGH) leftTicks++; else leftTicks--;
}

void IRAM_ATTR rightEncoderISR() {
  int b = digitalRead(ENC_RB);
  if (b == HIGH) rightTicks++; else rightTicks--;
}

// ================== Motor helper ==================
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

// ================== Update MPU heading ==================
void updateHeading() {
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  rateZ = (gz - gyroOffset) / 131.0; // deg/sec
  angleZ += rateZ * dt;
}

// ================== Move 29 cm ==================
void moveOneCell() {
  long targetTicks = (long)(29.0 / cmPerTick);
  leftTicks = 0;
  rightTicks = 0;

  updateHeading();
  float targetHeading = angleZ;

  while (true) {
    updateHeading();

    // Distance and heading errors
    float avgTicks = (leftTicks + rightTicks) / 2.0;
    float errorDist = targetTicks - avgTicks;
    float errorHead = targetHeading - angleZ;

    // PID control
    float controlDist = Kp_dist * errorDist;
    float controlHead = Kp_head * errorHead;

    int leftPWM = controlDist - controlHead;
    int rightPWM = controlDist + controlHead;

    leftPWM = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    setMotor(PWMA, AIN1, AIN2, leftPWM);
    setMotor(PWMB, BIN1, BIN2, rightPWM);

    // Stop when within 0.5 cm
    if (errorDist <= 0.5 / cmPerTick) break;
  }

  setMotor(PWMA, AIN1, AIN2, 0);
  setMotor(PWMB, BIN1, BIN2, 0);

  Serial.print("Reached 29 cm. Left ticks: ");
  Serial.print(leftTicks);
  Serial.print(" Right ticks: ");
  Serial.println(rightTicks);
}

// ================== Web server ==================
void handleRoot() {
  String html = "<html><body><h2>Micromouse PID Tuning</h2>";
  html += "Kp_dist: <input name='kpd' value='" + String(Kp_dist) + "'><br>";
  html += "Kp_head: <input name='kph' value='" + String(Kp_head) + "'><br>";
  html += "<input type='submit' value='Update' form='pidForm'>";
  html += "<form id='pidForm' action='/setpid'></form>";
  html += "<br><button onclick='fetch(\"/move\")'>Move 29cm</button>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleSetPID() {
  if (server.hasArg("kpd")) Kp_dist = server.arg("kpd").toFloat();
  if (server.hasArg("kph")) Kp_head = server.arg("kph").toFloat();
  handleRoot();
}

void handleMove() {
  moveOneCell();
  server.send(200, "text/plain", "Moved 29cm!");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Motor setup
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Encoder setup
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), rightEncoderISR, CHANGE);

  // MPU setup
  mpu.initialize();
  Serial.println("Calibrating gyro...");
  delay(3000);
  long sum = 0;
  for (int i = 0; i < 1000; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  gyroOffset = sum / 1000.0;
  Serial.println("Gyro calibrated!");
  lastTime = micros();

  // WiFi AP
  WiFi.softAP(ssid, password);
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());

  // Web server
  server.on("/", handleRoot);
  server.on("/setpid", handleSetPID);
  server.on("/move", handleMove);
  server.begin();
}

// ================== Main loop ==================
void loop() {
  server.handleClient();
}
