// ================================================================
//  PHASE 1c — Encoder Diagnostics (Quadrature, N20 motors)
//  Tests: count direction, CPR measurement, RPM
//
//  HOW TO USE:
//  1. Upload, open Serial Monitor @ 115200
//  2. Send 'R' to start a slow ramp (wheels in air)
//  3. Manually rotate one wheel exactly ONE full turn
//     → note the count printed → that's your real CPR
//  4. Update ENCODER_CPR in config.h with measured value
//  5. Send 'r' to reset counts at any time
//
//  VERIFY:
//  - Counts increase when moving forward
//  - If a count goes negative on forward, swap ENC_xA / ENC_xB
//  - Left and right counts should be roughly equal at same PWM
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
//  Encoder state (volatile — modified in ISR)
// ----------------------------------------------------------------
volatile long encL = 0;
volatile long encR = 0;

// Quadrature decode for LEFT encoder
void IRAM_ATTR isrLA() {
    if (digitalRead(ENC_LB) == LOW) encL++;
    else                             encL--;
}

// Quadrature decode for RIGHT encoder
void IRAM_ATTR isrRA() {
    if (digitalRead(ENC_RB) == LOW) encR++;
    else                             encR--;
}

// ----------------------------------------------------------------
//  Motor helpers (minimal, for spin test)
// ----------------------------------------------------------------
void motorsInit() {
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
    pwmInitMotorPins();
}

void setMotors(int speedA, int speedB) {
    auto drive = [](int s, int in1, int in2, int pwmPin) {
        s = constrain(s, -255, 255);
        if (s >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
        else        { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); s = -s; }
        if (pwmPin == PWMA) pwmWriteMotorA(s);
        else                pwmWriteMotorB(s);
    };
    drive(speedA, AIN1, AIN2, PWMA);
    drive(speedB, BIN1, BIN2, PWMB);
}

void stopMotors() {
    digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
    digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
    pwmWriteMotorA(0); pwmWriteMotorB(0);
}

// ----------------------------------------------------------------
//  CPR measurement helper
//  Runs motor at low speed for 2 seconds, prints counts
// ----------------------------------------------------------------
void measureCPR() {
    Serial.println("[CPR] Spinning both motors @ PWM 120 for 3 sec...");
    Serial.println("      Count the wheel rotations visually!");
    noInterrupts(); encL = 0; encR = 0; interrupts();
    setMotors(120, 120);
    uint32_t t0 = millis();
    while (millis() - t0 < 3000) {
        long l, r;
        noInterrupts(); l = encL; r = encR; interrupts();
        Serial.printf("  L: %6ld  R: %6ld\r", l, r);
        delay(100);
    }
    stopMotors();
    long l, r;
    noInterrupts(); l = encL; r = encR; interrupts();
    Serial.printf("\n[CPR] Final counts — L: %ld  R: %ld\n", l, r);
    Serial.println("  → Divide by number of rotations = your CPR");
    Serial.println("  → Update ENCODER_CPR in config.h");
}

// ----------------------------------------------------------------
//  Setup & Loop
// ----------------------------------------------------------------
void setup() {
    Serial.begin(SERIAL_BAUD);

    // Encoder pins with pull-ups
    pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP);
    pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);

    // Attach interrupts on channel A edges only (quadrature)
    attachInterrupt(digitalPinToInterrupt(ENC_LA), isrLA, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RA), isrRA, RISING);

    motorsInit();
    stopMotors();

    Serial.println("=== PHASE 1c: Encoder Diagnostics ===");
    Serial.println("Commands:");
    Serial.println("  r → reset counts");
    Serial.println("  f → forward @ 150 for 2s");
    Serial.println("  b → backward @ 150 for 2s");
    Serial.println("  c → CPR measurement spin");
    Serial.println("  s → stop motors");
}

uint32_t lastPrint = 0;

void loop() {
    // Continuous print of counts + RPM
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        static long prevL = 0, prevR = 0;
        static uint32_t prevT = 0;

        long l, r;
        noInterrupts(); l = encL; r = encR; interrupts();

        uint32_t now = millis();
        float dt = (now - prevT) / 1000.0f;
        float rpmL = 0, rpmR = 0;
        if (dt > 0 && ENCODER_CPR > 0) {
            rpmL = ((l - prevL) / ENCODER_CPR) / dt * 60.0f;
            rpmR = ((r - prevR) / ENCODER_CPR) / dt * 60.0f;
        }
        prevL = l; prevR = r; prevT = now;

        // Serial Plotter format
        Serial.print("EncL:"); Serial.print(l);
        Serial.print("\tEncR:"); Serial.print(r);
        Serial.print("\tRPM_L:"); Serial.print(rpmL, 1);
        Serial.print("\tRPM_R:"); Serial.println(rpmR, 1);
    }

    if (!Serial.available()) return;
    char cmd = Serial.read();

    switch (cmd) {
        case 'r':
            noInterrupts(); encL = 0; encR = 0; interrupts();
            Serial.println("[RESET] Encoder counts zeroed");
            break;
        case 'f':
            Serial.println("[FWD] Running @ 150 for 2s");
            noInterrupts(); encL = 0; encR = 0; interrupts();
            setMotors(150, 150);
            delay(2000);
            stopMotors();
            { long l, r; noInterrupts(); l=encL; r=encR; interrupts();
              Serial.printf("[FWD] Done — L: %ld  R: %ld\n", l, r); }
            break;
        case 'b':
            Serial.println("[REV] Running @ -150 for 2s");
            noInterrupts(); encL = 0; encR = 0; interrupts();
            setMotors(-150, -150);
            delay(2000);
            stopMotors();
            { long l, r; noInterrupts(); l=encL; r=encR; interrupts();
              Serial.printf("[REV] Done — L: %ld  R: %ld\n", l, r); }
            break;
        case 'c':
            measureCPR();
            break;
        case 's':
            stopMotors();
            Serial.println("[STOP]");
            break;
    }
}
