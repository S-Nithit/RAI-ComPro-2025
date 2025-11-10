#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>

MPU6050 mpu(Wire);
Servo armServo;
Servo clawServo;

// ====== Motor Pins ======
#define PIN_ENC_A 3 
#define PIN_EN_A A0
#define PIN_IN1_A 5
#define PIN_IN2_A 6

#define PIN_EN_B A1
#define PIN_IN3_B 7
#define PIN_IN4_B 4

byte buttonPins[] = {9, 8, 13, 12};
#define BTN_A 0 
#define BTN_B 1 
#define BTN_S1 2 
#define BTN_S2 3 

#define PIN_SERVO_ARM 11
#define PIN_SERVO_CLAW 10

// ====== Encoder Variable ======
volatile long volatile_encA_ticks = 0;
void isr_encA() { volatile_encA_ticks++; }

// ====== Motion Parameters ======
const float wheelBase = 15.0;
const float pulsesPerCM = 2.0;

struct PIDGains {
  float Kp, Ki, Kd;
};

// --- Turn ---
PIDGains turnGains = {3.0, 0.0, 22.0};
// --- Straight ---
PIDGains straightGains = {2.5, 0.25, 0.5};
// --- Distance ---
PIDGains distGains = {10.0, 0.03, 0.5};

// ====== Bias ======
float biasA = 5;
float biasB = 0;

// ====== Gyro Variables ======
float currentAngle = 0;
unsigned long lastTime = 0;

int lastButtonStates[4] = {HIGH, HIGH, HIGH, HIGH};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(10000, true); // (10ms timeout)

  // ====== Motor Setup ======
  pinMode(PIN_EN_A, OUTPUT);
  pinMode(PIN_IN1_A, OUTPUT);
  pinMode(PIN_IN2_A, OUTPUT);
  pinMode(PIN_EN_B, OUTPUT);
  pinMode(PIN_IN3_B, OUTPUT);
  pinMode(PIN_IN4_B, OUTPUT);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  
  Serial.println("Setting up button pins...");
  for (int i = 0; i < 4; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // ====== Servo Setup ======
  armServo.attach(PIN_SERVO_ARM);
  clawServo.attach(PIN_SERVO_CLAW);

  // ====== MPU Setup ======
  Serial.println("Init MPU...");
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU init failed! Code: ");
    Serial.println(status);
    while (1);
  }
  Serial.println("MPU OK.");

  Serial.println("Calibrating gyro...");
  delay(2000);
  mpu.calcOffsets();
  Serial.println("Calibration done.");

  PIDGains* gainsPtr = &turnGains;
  Serial.print("-> Checking Turn Kp (via pointer): ");
  Serial.println(gainsPtr->Kp); 
  Serial.println("-------------------");

  lastTime = millis();
  armServo.write(10);
  clawServo.write(10);
  delay(100);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isr_encA, RISING);
}
void loop() {
  mpu.update();
  currentAngle = mpu.getAngleZ();

  int currentButtonStates[4];
  for (int i = 0; i < 4; i++) {
    currentButtonStates[i] = digitalRead(buttonPins[i]);
  }
  if (currentButtonStates[BTN_A] == LOW && lastButtonStates[BTN_A] == HIGH) {
    runMissionA();
  }
  else if (currentButtonStates[BTN_B] == LOW && lastButtonStates[BTN_B] == HIGH) {
    runMissionB();
  }
  for (int i = 0; i < 4; i++) {
    lastButtonStates[i] = currentButtonStates[i];
  }

  delay(20);
}

// ====== Mission Sequences ======
void runMissionA() {
  move_cm(20);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(70);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  pickupCube();
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(10);
  turn_deg(90, false);
  move_cm(20);
  turn_deg(90, false);
  move_cm(20);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(90);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(70);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  dropCube();
  move_cm(10);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(70);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(90);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(70);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
}

void runMissionB() {
  move_cm(20);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  pickupCube();
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(10);
  turn_deg(90, false);
  move_cm(30);
  turn_deg(90, false);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(90);
  turn_deg(90, true);
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(70);
  turn_deg(90, true);
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(700);
  motorOff();
  delay(500);
  dropCube();
  move_cm(10);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(70);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(90);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
  turn_deg(90, true);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(90);
  turn_deg(90, false);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  move_cm(30);
}


// ====== Servo/Action Functions ======
void dropCube() {
  move_cm(25);
  delay(500);
  armServo.write(90);
  delay(500);
  clawServo.write(10);
  delay(500);
  motorBrake();
  delay(500);
  motorOff();
  delay(500);
  armServo.write(10);
  delay(500);
}

void pickupCube() {
  armServo.write(90);
  delay(500);
  move_cm(25);
  delay(500);
  clawServo.write(70);
  delay(500);
  armServo.write(10);
  delay(500);
}

// ====== Motor Control Functions ======
void motorBrake() {
  analogWrite(PIN_EN_A, 255);
  digitalWrite(PIN_IN1_A, LOW);
  digitalWrite(PIN_IN2_A, HIGH);
  analogWrite(PIN_EN_B, 255);
  digitalWrite(PIN_IN3_B, LOW);
  digitalWrite(PIN_IN4_B, HIGH);
}

void motorOff() {
  analogWrite(PIN_EN_A, 0);
  digitalWrite(PIN_IN1_A, LOW);
  digitalWrite(PIN_IN2_A, LOW);
  analogWrite(PIN_EN_B, 0);
  digitalWrite(PIN_IN3_B, LOW);
  digitalWrite(PIN_IN4_B, LOW);
}


// ====== MOVE FORWARD/BACKWARD
void move_cm(float distanceCM) {
  const int startupPWM = 128;
  const int minRunningPWM = 128;
  const int maxPWM = 255;

  long targetPulses = (long)(abs(distanceCM) * pulsesPerCM);
  if (targetPulses == 0) targetPulses = 1;
  long rampPulses = max(10L, targetPulses * 0.3);

  bool moveForward = (distanceCM > 0);

  // PID (Distance)
  long errDist = 0, lastErrDist = 0;
  float iTermDist = 0, dTermDist = 0;
  float pwmOut = 0;

  // PID (Angle)
  float errAngle = 0, lastErrAngle = 0;
  float iTermAngle = 0, dTermAngle = 0;
  float pidCorrection = 0;

  float correctedPWM_A = 0;
  float correctedPWM_B = 0;
  unsigned long lastPrintTime = 0;

  // Reset
  volatile_encA_ticks = 0;
  mpu.update();
  float targetAngle = mpu.getAngleZ();
  currentAngle = targetAngle;
  lastTime = millis();

  Serial.print("Move: ");
  Serial.print(distanceCM);
  Serial.println(" cm");

  while (true) {
    long ticksSnapshot;
    noInterrupts();
    ticksSnapshot = volatile_encA_ticks;
    interrupts();

    if (ticksSnapshot >= targetPulses) {
      break;
    }

    // (A) Gyro
    mpu.update();
    currentAngle = mpu.getAngleZ();

    // (B) PID (Angle) 
    errAngle = targetAngle - currentAngle;
    iTermAngle = constrain(iTermAngle + errAngle, -300, 300);
    dTermAngle = errAngle - lastErrAngle;

    pidCorrection = (straightGains.Kp * errAngle) +
                     (straightGains.Ki * iTermAngle) +
                     (straightGains.Kd * dTermAngle);

    // (C) PID (Distance) 
    errDist = targetPulses - ticksSnapshot;
    iTermDist = constrain(iTermDist + errDist, -10000, 10000);
    dTermDist = errDist - lastErrDist;
    pwmOut = (distGains.Kp * errDist) + (distGains.Ki * iTermDist) + (distGains.Kd * dTermDist);
    pwmOut = constrain(pwmOut, 0, maxPWM);

    // (D) Ramp Down
    long rampUpSpeed = map(ticksSnapshot, 0, rampPulses, startupPWM, maxPWM);
    long remainingPulses = targetPulses - ticksSnapshot;
    long rampDownSpeed = map(remainingPulses, 0, rampPulses, minRunningPWM, maxPWM);
    long maxAllowedSpeed = min(rampUpSpeed, rampDownSpeed);
    maxAllowedSpeed = constrain(maxAllowedSpeed, minRunningPWM, maxPWM);
    pwmOut = min(pwmOut, maxAllowedSpeed);

    if (moveForward) {
      digitalWrite(PIN_IN1_A, HIGH); digitalWrite(PIN_IN2_A, LOW);
      digitalWrite(PIN_IN3_B, HIGH); digitalWrite(PIN_IN4_B, LOW);
    } else {
      digitalWrite(PIN_IN1_A, LOW); digitalWrite(PIN_IN2_A, HIGH);
      digitalWrite(PIN_IN3_B, LOW); digitalWrite(PIN_IN4_B, HIGH);
    }

    // (E) Apply correction
    correctedPWM_A = pwmOut - pidCorrection + biasA;
    correctedPWM_B = pwmOut + pidCorrection + biasB;
    correctedPWM_A = constrain(correctedPWM_A, 0, maxPWM);
    correctedPWM_B = constrain(correctedPWM_B, 0, maxPWM);

    analogWrite(PIN_EN_A, correctedPWM_A);
    analogWrite(PIN_EN_B, correctedPWM_B);

    lastErrDist = errDist;
    lastErrAngle = errAngle;

    if (millis() - lastPrintTime > 100) {
      Serial.print("Ticks:"); Serial.print(ticksSnapshot);
      Serial.print("/"); Serial.print(targetPulses);
      Serial.print(" | Angle:"); Serial.print(currentAngle, 1);
      Serial.print(" (Err:"); Serial.print(errAngle, 1); Serial.print(")");
      Serial.print(" | PWMA:"); Serial.print(correctedPWM_A, 0);
      Serial.print(" | PWMB:"); Serial.println(correctedPWM_B, 0);
      lastPrintTime = millis();
    }
    delay(10);
  }

  // (F) Hard Brake
  analogWrite(PIN_EN_A, 0);
  analogWrite(PIN_EN_B, 0);
  digitalWrite(PIN_IN1_A, HIGH); digitalWrite(PIN_IN2_A, HIGH);
  digitalWrite(PIN_IN3_B, HIGH); digitalWrite(PIN_IN4_B, HIGH);
  delay(100);
  motorOff();

  Serial.println("Move complete.\n");
}

void move_cm_backward(float distanceCM) {
  move_cm(-distanceCM);
}


// ====== TURN LEFT/RIGHT
void turn_deg(float degree, bool turnRight) {
  mpu.update();
  float startAngle = mpu.getAngleZ();
  float targetAngle = turnRight ? (startAngle - degree) : (startAngle + degree);

  Serial.print("Turn: ");
  Serial.print(degree);
  Serial.println(" deg");

  float maxTurnPWM = 140;
  float minTurnPWM = 128;
  float stopThreshold = 10.0;

  unsigned long lastPrintTime = 0;
  float turnPWM = 0;
  float err = 0;
  float lastErr = 0;

  while (true) {
    mpu.update();
    currentAngle = mpu.getAngleZ();

    err = targetAngle - currentAngle;

    if (abs(err) < stopThreshold) {
      break;
    }
    
    float P_term = turnGains.Kp * err;
    float D_term = turnGains.Kd * (err - lastErr);
    float pidOut = P_term + D_term; // (Ki = 0.0)

    lastErr = err;
    turnPWM = abs(pidOut);
    turnPWM = constrain(turnPWM, minTurnPWM, maxTurnPWM);

    if (pidOut > 0) {
      digitalWrite(PIN_IN1_A, LOW); digitalWrite(PIN_IN2_A, HIGH);
      digitalWrite(PIN_IN3_B, HIGH); digitalWrite(PIN_IN4_B, LOW);
    } else {
      digitalWrite(PIN_IN1_A, HIGH); digitalWrite(PIN_IN2_A, LOW);
      digitalWrite(PIN_IN3_B, LOW); digitalWrite(PIN_IN4_B, HIGH);
    }

    analogWrite(PIN_EN_A, turnPWM);
    analogWrite(PIN_EN_B, turnPWM);

    if (millis() - lastPrintTime > 100) {
      Serial.print("Tgt: "); Serial.print(targetAngle, 2);
      Serial.print(" | Cur: "); Serial.print(currentAngle, 2);
      Serial.print(" | Err: "); Serial.print(err, 2);
      Serial.print(" | PWM: "); Serial.println(turnPWM, 0);
      lastPrintTime = millis();
    }

    delay(5);
  }

  // Hard Brake
  analogWrite(PIN_EN_A, 0);
  analogWrite(PIN_EN_B, 0);
  digitalWrite(PIN_IN1_A, HIGH);
  digitalWrite(PIN_IN2_A, HIGH);
  digitalWrite(PIN_IN3_B, HIGH);
  digitalWrite(PIN_IN4_B, HIGH);
  delay(50);
  motorOff();

  Serial.println("Turn complete.\n");
}
