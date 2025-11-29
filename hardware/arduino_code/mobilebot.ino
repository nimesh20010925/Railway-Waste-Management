#include <ESP32Servo.h>
#include <NewPing.h>

// ==========================================================
// ---------------------- PIN DEFINITIONS --------------------
// ==========================================================

// ---- Motor driver pins (L298N) ----
const int LeftMotorForward = 17;
const int LeftMotorBackward = 16;
const int RightMotorForward = 18;
const int RightMotorBackward = 19;
const int ENA = 4;    // Left motor speed control (PWM)
const int ENB = 21;   // Right motor speed control (PWM)

// ---- Ultrasonic Sensor ----
#define TRIG_PIN 12
#define ECHO_PIN 14
#define MAX_DISTANCE 200  // cm

// ---- Servo Motor (Ultrasonic Mount) ----
#define SERVO_PIN 13

// ---- IR Sensors for Edge Detection ----
#define IR_LEFT 32
#define IR_RIGHT 33
#define IR_FRONT_LEFT 34
#define IR_FRONT_RIGHT 35

// ==========================================================
// ---------------------- GLOBAL VARIABLES ------------------
// ==========================================================

boolean goesForward = false;
int speedSet = 150;        // Motor speed (0–255)
int safeDistance = 15;     // Stop distance in cm
int centerPos = 90;        // Servo center position
const int SCAN_DELAY = 500; // Servo move delay

// ---- PWM Channels ----
#define ENA_CH 0
#define ENB_CH 1

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo servoMotor;

// ==========================================================
// ---------------------- SETUP FUNCTION --------------------
// ==========================================================

void setup() {
  Serial.begin(115200);

  // ---- Servo Setup ----
  ESP32PWM::allocateTimer(0);
  servoMotor.setPeriodHertz(50);  // 50Hz = standard servo frequency
  servoMotor.attach(SERVO_PIN, 1000, 2000);
  servoMotor.write(centerPos);    // Center servo
  delay(500);

  // ---- Motor Setup ----
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);

  // PWM setup for ESP32
  ledcSetup(ENA_CH, 1000, 8); // 1kHz, 8-bit resolution
  ledcSetup(ENB_CH, 1000, 8);
  ledcAttachPin(ENA, ENA_CH);
  ledcAttachPin(ENB, ENB_CH);

  // ---- IR Sensors Setup ----
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FRONT_LEFT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);

  Serial.println("🚗 Robot Initialized...");
  delay(1500);
}

// ==========================================================
// ---------------------- MAIN LOOP -------------------------
// ==========================================================

void loop() {
  int distance = readPing();  // Read ultrasonic distance

  // ---- Continuous IR Sensor Reading ----
  bool leftOK = digitalRead(IR_LEFT);
  bool rightOK = digitalRead(IR_RIGHT);
  bool frontLeftOK = digitalRead(IR_FRONT_LEFT);
  bool frontRightOK = digitalRead(IR_FRONT_RIGHT);

  // ---- Debug Monitor ----
  Serial.print("Distance: "); Serial.print(distance);
  Serial.print(" | IR L="); Serial.print(leftOK);
  Serial.print(" R="); Serial.print(rightOK);
  Serial.print(" FL="); Serial.print(frontLeftOK);
  Serial.print(" FR="); Serial.println(frontRightOK);

  // ==========================================================
  // ---------------------- EDGE DETECTION ---------------------
  // ==========================================================
  if (!frontLeftOK || !frontRightOK) {
    moveStop();
    delay(200);
    moveBackward();
    delay(400);
    moveStop();
    delay(200);

    if (!frontLeftOK && frontRightOK) {
      turnRight();
      delay(500);
    } else if (!frontRightOK && frontLeftOK) {
      turnLeft();
      delay(500);
    } else {
      moveBackward();
      delay(500);
    }
    moveStop();
  }

  else if (!leftOK) {
    moveStop();
    delay(100);
    turnRight();
    delay(400);
    moveStop();
  }

  else if (!rightOK) {
    moveStop();
    delay(100);
    turnLeft();
    delay(400);
    moveStop();
  }

  // ==========================================================
  // ---------------------- OBSTACLE AVOIDANCE ----------------
  // ==========================================================
  else if (distance <= safeDistance) {
    moveStop();
    delay(200);
    moveBackward();
    delay(300);
    moveStop();
    delay(300);

    int rightDist = lookRight();
    int leftDist = lookLeft();
    servoMotor.write(centerPos);
    delay(200);

    if (rightDist > leftDist) {
      turnRight();
      delay(600);
    } else {
      turnLeft();
      delay(600);
    }
    moveStop();
  }

  // ==========================================================
  // ---------------------- NORMAL FORWARD --------------------
  // ==========================================================
  else {
    moveForward();
  }

  delay(50);
}

// ==========================================================
// ---------------------- HELPER FUNCTIONS ------------------
// ==========================================================

int readPing() {
  delay(40);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = MAX_DISTANCE;
  return cm;
}

int lookRight() {
  servoMotor.write(40);
  delay(SCAN_DELAY);
  int dist = readPing();
  servoMotor.write(centerPos);
  delay(100);
  return dist;
}

int lookLeft() {
  servoMotor.write(140);
  delay(SCAN_DELAY);
  int dist = readPing();
  servoMotor.write(centerPos);
  delay(100);
  return dist;
}

// ==========================================================
// ---------------------- MOTOR FUNCTIONS --------------------
// ==========================================================

void moveForward() {
  if (!goesForward) {
    goesForward = true;
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
    ledcWrite(ENA_CH, speedSet);
    ledcWrite(ENB_CH, speedSet);
  }
}

void moveBackward() {
  goesForward = false;
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  ledcWrite(ENA_CH, speedSet);
  ledcWrite(ENB_CH, speedSet);
}

void moveStop() {
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  ledcWrite(ENA_CH, 0);
  ledcWrite(ENB_CH, 0);
}

void turnRight() {
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  ledcWrite(ENA_CH, speedSet);
  ledcWrite(ENB_CH, speedSet);
}

void turnLeft() {
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  ledcWrite(ENA_CH, speedSet);
  ledcWrite(ENB_CH, speedSet);
}
