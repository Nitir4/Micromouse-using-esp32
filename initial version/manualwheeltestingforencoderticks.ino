// --- Encoder Pins (your setup) ---
#define ENC_LA 18  // Left Encoder A
#define ENC_LB 19  // Left Encoder B
#define ENC_RA 5   // Right Encoder A
#define ENC_RB 4   // Right Encoder B

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// --- Left Encoder ISR ---
void IRAM_ATTR leftEncoderISR() {
  int b = digitalRead(ENC_LB);
  if (b == HIGH) leftTicks++;
  else leftTicks--;
}

// --- Right Encoder ISR ---
void IRAM_ATTR rightEncoderISR() {
  int b = digitalRead(ENC_RB);
  if (b == HIGH) rightTicks++;
  else rightTicks--;
}

void setup() {
  Serial.begin(115200);

  // Encoder pin modes
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_LA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), rightEncoderISR, RISING);

  Serial.println("Rotate each wheel manually for one full revolution...");
  Serial.println("I will print encoder tick counts every 2 seconds.");
}

void loop() {
  static long prevLeft = 0, prevRight = 0;

  delay(2000); // check every 2 sec

  long currentLeft = leftTicks;
  long currentRight = rightTicks;

  Serial.print("Left wheel ticks = ");
  Serial.print(currentLeft - prevLeft);
  Serial.print(" | Right wheel ticks = ");
  Serial.println(currentRight - prevRight);

  prevLeft = currentLeft;
  prevRight = currentRight;
}
