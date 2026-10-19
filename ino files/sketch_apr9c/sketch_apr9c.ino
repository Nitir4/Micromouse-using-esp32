// ================================================================
//  MICROMOUSE — Phase 2 Unified Sketch
//  ESP32 + TB6612FNG + N20 Quadrature + MPU6050 + Sharp IR
//
//  Fixes applied:
//  [1] dt zero guard (prevents derivative spike on first frame)
//  [2] Integrator windup reset on stop
//  [3] rateZ used for straight-line heading correction (drift-free)
//  [4] angleZ used only during turns (short duration = tiny drift)
//  [5] IR sensors fully disabled during turn, re-enabled after
//  [6] State machine: IDLE → STRAIGHT → TURNING → WALL_ALIGN
// ================================================================

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// ================================================================
//  PIN DEFINITIONS
// ================================================================
#define PWMA 32
#define AIN1 25
#define AIN2 33
#define PWMB 12
#define BIN1 14
#define BIN2 27
#define STBY 26

#define ENC_LA 18
#define ENC_LB 19
#define ENC_RA 5
#define ENC_RB 4

const int irPins[3] = {36, 39, 35};  // RIGHT=36, CENTER=39, LEFT=35
#define IR_RIGHT  0
#define IR_CENTER 1
#define IR_LEFT   2

// ================================================================
//  ROBOT CONFIG  — tune after Phase 1 calibration
// ================================================================
const float wheelCircumference = 20.42f;   // cm  [TUNE]
const int   ticksPerRev        = 360;       // CPR [TUNE]
const float cmPerTick          = wheelCircumference / ticksPerRev;
const float CELL_SIZE_CM       = 18.0f;     // maze cell size [TUNE]
const float CORRIDOR_WIDTH_CM  = 16.0f;     // inner wall-to-wall [TUNE]

// ================================================================
//  STATE MACHINE
// ================================================================
enum RobotState {
    IDLE,
    STRAIGHT,    // moving forward one cell, rateZ keeps heading
    TURNING,     // 90° turn using angleZ, IR disabled
    WALL_ALIGN,  // post-turn: use IR to correct lateral + heading
    STOPPED
};

RobotState state = IDLE;
int turnDirection = 0;  // +1 = left, -1 = right

// ================================================================
//  IR SENSORS
// ================================================================
float distances[3];   // cm
bool  irEnabled = true;

int readStable(int pin) {
    int sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += analogRead(pin);
        delay(2);
    }
    return sum / 10;
}

float voltageToDistance_GP2Y0A21(float v) {
    if (v > 0.42f) return 27.728f / (v - 0.42f);
    return 80.0f;
}

float voltageToDistance_GP2D120(float v) {
    if (v > 0.1f) return 13.0f / v;
    return 30.0f;
}

void updateIR() {
    if (!irEnabled) return;
    for (int i = 0; i < 3; i++) {
        int raw = readStable(irPins[i]);
        float v = (raw / 4095.0f) * 3.3f;
        distances[i] = (i == IR_CENTER)
            ? voltageToDistance_GP2Y0A21(v)
            : voltageToDistance_GP2D120(v);
    }
}

// Wall-based lateral correction
// Returns a turn command offset: positive = steer right, negative = steer left
float wallAlignCorrection() {
    float l = distances[IR_LEFT];
    float r = distances[IR_RIGHT];
    // Only correct if both walls are visible (within sensor range)
    bool leftValid  = (l < 25.0f);
    bool rightValid = (r < 25.0f);

    if (leftValid && rightValid) {
        return (l - r) * 1.5f;   // symmetric correction [TUNE gain]
    } else if (leftValid) {
        return (l - (CORRIDOR_WIDTH_CM / 2.0f)) * 1.0f;
    } else if (rightValid) {
        return -((CORRIDOR_WIDTH_CM / 2.0f) - r) * 1.0f;
    }
    return 0.0f;
}

// ================================================================
//  ENCODERS
// ================================================================
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

#define LEFT_DIR_SIGN   1
#define RIGHT_DIR_SIGN  1

void IRAM_ATTR leftEncoderISR() {
    if (digitalRead(ENC_LB) == LOW) leftTicks += LEFT_DIR_SIGN;
    else                             leftTicks -= LEFT_DIR_SIGN;
}

void IRAM_ATTR rightEncoderISR() {
    if (digitalRead(ENC_RB) == LOW) rightTicks += RIGHT_DIR_SIGN;
    else                             rightTicks -= RIGHT_DIR_SIGN;
}

void resetEncoders() {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
}

// ================================================================
//  MOTORS
// ================================================================
void setMotor(int pwmPin, int in1, int in2, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (speed < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        speed = -speed;
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    analogWrite(pwmPin, speed);
}

void stopMotors() {
    setMotor(PWMA, AIN1, AIN2, 0);
    setMotor(PWMB, BIN1, BIN2, 0);
}

// ================================================================
//  MPU6050
// ================================================================
int16_t ax, ay, az, gx, gy, gz;
float rateZ  = 0;   // deg/s  — drift-free, used for straight correction
float angleZ = 0;   // deg    — integrated, used only during turns
float gyroOffset = 0;

void calibrateGyro() {
    Serial.println("Calibrating gyro — keep still...");
    delay(2000);
    long sum = 0;
    for (int i = 0; i < 1000; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sum += gz;
        delay(2);
    }
    gyroOffset = sum / 1000.0f;
    angleZ = 0;
    Serial.printf("Gyro calibrated. Offset: %.2f\n", gyroOffset);
}

// ================================================================
//  PID STATE
// ================================================================

// --- Straight PID (distance) ---
float Kp_dist = 0.5f, Ki_dist = 0.0f, Kd_dist = 0.1f;
float distIntegral = 0, lastDistError = 0;

// --- Straight heading correction (uses rateZ, NOT angleZ) ---
// We want rateZ == 0 while going straight → damp any yaw rotation instantly
float Kp_rate = 3.0f;   // [TUNE] gain on yaw rate damping

// --- Turn PID (uses angleZ for total angle covered) ---
float Kp_turn = 4.0f, Ki_turn = 0.0f, Kd_turn = 0.3f;
float turnIntegral = 0, lastTurnError = 0;
float turnTarget = 0;   // degrees (+90 or -90)

// ================================================================
//  TIMING
// ================================================================
unsigned long lastTime  = 0;
unsigned long lastPrint = 0;
float dt = 0;

// ================================================================
//  MOTION COMMANDS  (call these to trigger movement)
// ================================================================
long targetTicks = 0;

void startStraight(float distanceCm) {
    resetEncoders();
    distIntegral  = 0;
    lastDistError = 0;
    // do NOT reset angleZ here — rateZ correction doesn't need it
    targetTicks = (long)(distanceCm / cmPerTick);
    irEnabled   = true;
    state       = STRAIGHT;
    Serial.printf("[STRAIGHT] Target: %.1f cm (%ld ticks)\n", distanceCm, targetTicks);
}

void startTurn(int direction) {
    // direction: +1 = left (CCW), -1 = right (CW)
    turnDirection = direction;
    angleZ        = 0;    // zero here — only used for this turn's duration
    turnIntegral  = 0;
    lastTurnError = 0;
    turnTarget    = direction * 90.0f;
    irEnabled     = false;   // ← IR disabled for entire turn
    state         = TURNING;
    Serial.printf("[TURN] %s 90°  IR DISABLED\n", direction > 0 ? "LEFT" : "RIGHT");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    analogSetAttenuation(ADC_11db);

    // Encoders
    pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP);
    pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_LA), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RA), rightEncoderISR, RISING);

    // Motors
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

    calibrateGyro();

    lastTime = micros();

    // Example: move one cell forward then turn right
    // startStraight(CELL_SIZE_CM);
    // After straight completes, call: startTurn(-1);
    state = IDLE;
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
    // ---- Timing ----
    unsigned long now = micros();
    dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    // [FIX 1] dt zero guard — skip pathological first frame
    if (dt < 0.0001f) return;

    // ---- MPU ----
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    rateZ  = (gz - gyroOffset) / 131.0f;   // deg/s — always computed, no drift
    angleZ += rateZ * dt;                   // deg   — only meaningful during TURNING

    // ---- IR (skipped during turns) ----
    updateIR();

    // ================================================================
    //  STATE MACHINE
    // ================================================================
    switch (state) {

        // ------------------------------------------------------------
        case IDLE:
        case STOPPED:
            stopMotors();
            break;

        // ------------------------------------------------------------
        case STRAIGHT: {
            long avgTicks = (leftTicks + rightTicks) / 2;
            long distError = targetTicks - avgTicks;

            // Distance PID
            distIntegral += distError * dt;
            float distDeriv = (distError - lastDistError) / dt;
            float speedCmd  = Kp_dist * distError
                            + Ki_dist * distIntegral
                            + Kd_dist * distDeriv;
            lastDistError = distError;

            // [FIX 3] Heading: damp yaw RATE to zero (drift-free)
            // rateZ != 0 means currently rotating → correct immediately
            float turnCmd = -Kp_rate * rateZ;

            // Wall-based lateral correction (blended in when walls visible)
            turnCmd += wallAlignCorrection() * 0.3f;  // [TUNE blend factor]

            int baseSpeed = constrain((int)speedCmd, -150, 150);
            int leftPWM   = constrain(baseSpeed - (int)turnCmd, -255, 255);
            int rightPWM  = constrain(baseSpeed + (int)turnCmd, -255, 255);

            setMotor(PWMA, AIN1, AIN2, leftPWM);
            setMotor(PWMB, BIN1, BIN2, rightPWM);

            // Stop condition
            if (abs(distError) < 10) {
                stopMotors();
                // [FIX 2] Reset integrals to prevent windup on next move
                distIntegral  = 0;
                lastDistError = 0;
                state = IDLE;
                Serial.println("[STRAIGHT] Done.");
            }
            break;
        }

        // ------------------------------------------------------------
        case TURNING: {
            // IR is DISABLED — pure gyro control
            // angleZ was zeroed at turn start → tracks only this turn's rotation
            float turnError = turnTarget - angleZ;

            turnIntegral += turnError * dt;
            float turnDeriv = (turnError - lastTurnError) / dt;
            float turnCmd   = Kp_turn * turnError
                            + Ki_turn * turnIntegral
                            + Kd_turn * turnDeriv;
            lastTurnError = turnError;

            int turnPWM = constrain((int)turnCmd, -200, 200);

            // Spin in place: one motor forward, one backward
            setMotor(PWMA, AIN1, AIN2, -turnPWM);   // left
            setMotor(PWMB, BIN1, BIN2,  turnPWM);   // right

            // Done when within 2° of target
            if (abs(turnError) < 2.0f) {
                stopMotors();
                turnIntegral  = 0;
                lastTurnError = 0;
                irEnabled     = true;   // ← re-enable IR now that turn is complete
                state         = WALL_ALIGN;
                Serial.printf("[TURN] Done. angleZ=%.2f°  IR RE-ENABLED\n", angleZ);
            }
            break;
        }

        // ------------------------------------------------------------
        case WALL_ALIGN: {
            // Short phase: use IR to correct lateral position + any residual
            // heading error before next cell move
            // Runs for ~300ms then hands off
            static uint32_t alignStart = 0;
            if (alignStart == 0) alignStart = millis();

            float correction = wallAlignCorrection();

            // Gentle correction pulses — not full speed
            int leftPWM  = constrain(-(int)correction, -80, 80);
            int rightPWM = constrain( (int)correction, -80, 80);
            setMotor(PWMA, AIN1, AIN2, leftPWM);
            setMotor(PWMB, BIN1, BIN2, rightPWM);

            if (millis() - alignStart > 300) {
                alignStart = 0;
                stopMotors();
                state = IDLE;
                Serial.println("[WALL_ALIGN] Done. Ready for next cell.");
            }
            break;
        }
    }

    // ================================================================
    //  SERIAL MONITOR — every 500ms
    // ================================================================
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        Serial.printf("[STATE] %s\n",
            state==IDLE?"IDLE":state==STRAIGHT?"STRAIGHT":
            state==TURNING?"TURNING":state==WALL_ALIGN?"WALL_ALIGN":"STOPPED");
        Serial.printf("  Enc L:%ld R:%ld  rateZ:%.2f°/s  angleZ:%.2f°\n",
                      leftTicks, rightTicks, rateZ, angleZ);
        if (irEnabled) {
            Serial.printf("  IR  R:%.1fcm C:%.1fcm L:%.1fcm\n",
                          distances[IR_RIGHT], distances[IR_CENTER], distances[IR_LEFT]);
        } else {
            Serial.println("  IR  [DISABLED — mid-turn]");
        }
    }
}