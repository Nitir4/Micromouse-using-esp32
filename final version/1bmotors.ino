// ================================================================
//  PHASE 1b — Motor Diagnostics (TB6612FNG)
//  Tests: direction, speed ramp, both motors independently
//
//  HOW TO USE:
//  1. Lift robot off ground (wheels in air)
//  2. Upload & open Serial Monitor @ 115200
//  3. Send commands via Serial Monitor:
//       'f' → both forward
//       'b' → both backward
//       'l' → left motor only
//       'r' → right motor only
//       's' → stop
//       'R' → full ramp test (auto sequence)
//       '?' → print help
//
//  VERIFY:
//  - Both wheels spin in correct direction for 'f'
//  - If a wheel spins backwards on 'f', swap its AIN1/AIN2 wires
// ================================================================

#include "config.h"

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static inline void pwmInitMotorPins() {
    ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);
}

static inline void pwmWriteMotorA(int duty) {
    ledcWrite(PWMA, duty);
}

static inline void pwmWriteMotorB(int duty) {
    ledcWrite(PWMB, duty);
}
#else
static inline void pwmInitMotorPins() {
    ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWMA, PWM_CH_A);
    ledcAttachPin(PWMB, PWM_CH_B);
}

static inline void pwmWriteMotorA(int duty) {
    ledcWrite(PWM_CH_A, duty);
}

static inline void pwmWriteMotorB(int duty) {
    ledcWrite(PWM_CH_B, duty);
}
#endif

// ----------------------------------------------------------------
//  Motor driver init
// ----------------------------------------------------------------
void motorsInit() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);  // take out of standby

    pwmInitMotorPins();
}

// ----------------------------------------------------------------
//  Low-level motor control
//  speed: -255 (full reverse) to +255 (full forward)
// ----------------------------------------------------------------
void setMotorA(int speed) {   // LEFT motor
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        speed = -speed;
    }
    pwmWriteMotorA(speed);
}

void setMotorB(int speed) {   // RIGHT motor
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        speed = -speed;
    }
    pwmWriteMotorB(speed);
}

void stopMotors() {
    // Coast stop (both inputs LOW)
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    pwmWriteMotorA(0);
    pwmWriteMotorB(0);
}

void brakeMotors() {
    // Active brake (both inputs HIGH)
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
    pwmWriteMotorA(255);
    pwmWriteMotorB(255);
}

// ----------------------------------------------------------------
//  Ramp test — sweeps speed 0→255→0 on both motors
// ----------------------------------------------------------------
void rampTest() {
    Serial.println("[RAMP] 0 → 255 forward ...");
    for (int s = 0; s <= 255; s += 5) {
        setMotorA(s); setMotorB(s);
        Serial.printf("  PWM: %3d\n", s);
        delay(30);
    }
    delay(500);
    Serial.println("[RAMP] 255 → 0 ...");
    for (int s = 255; s >= 0; s -= 5) {
        setMotorA(s); setMotorB(s);
        delay(30);
    }
    delay(500);
    Serial.println("[RAMP] 0 → -255 reverse ...");
    for (int s = 0; s >= -255; s -= 5) {
        setMotorA(s); setMotorB(s);
        delay(30);
    }
    delay(500);
    Serial.println("[RAMP] -255 → 0 ...");
    for (int s = -255; s <= 0; s += 5) {
        setMotorA(s); setMotorB(s);
        delay(30);
    }
    stopMotors();
    Serial.println("[RAMP] Done.");
}

void printHelp() {
    Serial.println("=== Motor Test Commands ===");
    Serial.println("  f  → both forward  (PWM 180)");
    Serial.println("  b  → both backward (PWM 180)");
    Serial.println("  l  → left motor only forward");
    Serial.println("  r  → right motor only forward");
    Serial.println("  s  → coast stop");
    Serial.println("  x  → active brake");
    Serial.println("  R  → full ramp test");
    Serial.println("  ?  → this help");
}

// ----------------------------------------------------------------
//  Setup & Loop
// ----------------------------------------------------------------
void setup() {
    Serial.begin(SERIAL_BAUD);
    motorsInit();
    stopMotors();
    Serial.println("=== PHASE 1b: Motor Diagnostics ===");
    Serial.println("LIFT ROBOT OFF GROUND before testing!");
    printHelp();
}

void loop() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    switch (cmd) {
        case 'f':
            Serial.println("[CMD] Both forward @ 180");
            setMotorA(180); setMotorB(180);
            break;
        case 'b':
            Serial.println("[CMD] Both backward @ 180");
            setMotorA(-180); setMotorB(-180);
            break;
        case 'l':
            Serial.println("[CMD] Left motor only @ 180");
            setMotorA(180); setMotorB(0);
            break;
        case 'r':
            Serial.println("[CMD] Right motor only @ 180");
            setMotorA(0); setMotorB(180);
            break;
        case 's':
            Serial.println("[CMD] Coast stop");
            stopMotors();
            break;
        case 'x':
            Serial.println("[CMD] Active brake");
            brakeMotors();
            break;
        case 'R':
            Serial.println("[CMD] Starting ramp test...");
            rampTest();
            break;
        case '?':
            printHelp();
            break;
        default:
            break;
    }
}
