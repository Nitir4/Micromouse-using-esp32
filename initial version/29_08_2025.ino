// Pin definitions
#define rightMotorF 8      // Right motor forward pin
#define rightMotorB 7      // Right motor backward pin
#define rightMotorPWM 6     // Right motor PWM pin (ENA)
#define leftMotorF 9        // Left motor forward pin
#define leftMotorB 10       // Left motor backward pin
#define leftMotorPWM 11     // Left motor PWM pin (ENB)

// Define IR sensor array pins
const int numSensors = 6;
const int irSensors[numSensors] = {A2,A1,A3,A4,A5,3};

// PID parameters
float kp = 30;    // Proportional gain
float ki = 1;     // Integral gain
float kd = 23;    // Derivative gain

const int baseSpeedLeft= 80;
const int baseSpeedRight= 100;

// PID variables
long integral = 0;
int previousError = 0;

void setup() {
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  
  for (int i = 0; i < numSensors; i++) {
    pinMode(irSensors[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {
  int sensorStates[numSensors];
  for (int i = 0; i < numSensors; i++) {
    sensorStates[i] = digitalRead(irSensors[i]);
  }

  // Check for finish condition
//   if (checkFinishCondition(sensorStates)) {
//     // Move forward a bit
//     driveMotors(190,120);  
//     delay(1000);  // Move forward for a short duration

// Serial.print("Sensors: ");
// for (int i = 0; i < numSensors; i++) {
//   Serial.print(sensorStates[i]);
//   Serial.print(" ");
// }
// Serial.println();

//     // If still all HIGH, stop the robot
//     if (checkFinishCondition(sensorStates)) {
//       stopMotors();
//       Serial.println("Finished: All sensors HIGH.");
//       while (true); // Halt the program to stop the robot
//     }
//   }

  int error = calculateError(sensorStates);
  int motorSpeedDifference = calculatePID(error);

  // Adjust motor speeds
  int leftSpeed = baseSpeedLeft + motorSpeedDifference;
  int rightSpeed = baseSpeedRight - motorSpeedDifference;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 180);

  driveMotors(leftSpeed, rightSpeed);

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | PID: ");
  Serial.print(motorSpeedDifference);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);

  delay(10);
}

int calculateError(int sensorStates[]) {
  int weights[numSensors] = {500, 500, 0, 0, -500, -500};
  
  long weightedSum = 0;
  int activeSensors = 0;

  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == HIGH) { 
      weightedSum += weights[i];
      activeSensors++;
    }
  }

  if (activeSensors == 0) {
    return previousError;
  }

  int error = weightedSum / activeSensors;
  return error;
}

int calculatePID(int error) {
  integral += error;
  integral = constrain(integral, -1000, 1000);
  int derivative = error - previousError;
  float PID = (kp * error) + (ki * integral) + (kd * derivative);
  previousError = error;
  return (int) PID;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, leftSpeed);
  } else {
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, -leftSpeed);
  }

  if (rightSpeed > 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, rightSpeed);
  } else {
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, HIGH);
    analogWrite(rightMotorPWM, -rightSpeed);
  }
}

// Function to check if all sensors are HIGH
bool checkFinishCondition(int sensorStates[]) {
  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == LOW) {
      return false;
    }
  }
  return true;
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);

  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 0);
}
