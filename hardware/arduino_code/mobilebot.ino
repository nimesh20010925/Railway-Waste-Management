#include <Arduino.h>
#include <ESP32Servo.h>
#include <NewPing.h>

// Pin definitions
#define TRIG_PIN  12
#define ECHO_PIN  14
#define SERVO_PIN 19
#define MOTOR_IN1 13
#define MOTOR_IN2 12
#define MOTOR_IN3 14
#define MOTOR_IN4 27
#define MOTOR_ENA 26
#define MOTOR_ENB 25
#define IR1_PIN   23
#define IR2_PIN   22
#define IR3_PIN   21
#define IR4_PIN   4
#define MAX_DISTANCE 200 // Maximum distance for ultrasonic sensor

// Create servo and ultrasonic objects
Servo myServo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Attach servo
  myServo.attach(SERVO_PIN);

  // Set motor control pins as outputs
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);

  // Set IR sensor pins as inputs
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);

  // Initialize motor driver with PWM for speed control
  analogWrite(MOTOR_ENA, 128); // 50% duty cycle for motors
  analogWrite(MOTOR_ENB, 128);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
}

void loop() {
  // Read ultrasonic sensor
  unsigned int distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.println(distance);

  // Control servo based on distance
  if (distance > 0 && distance < 20) {
    myServo.write(90); // Stop at 90 degrees if obstacle is close
  } else {
    myServo.write(0); // Move to 0 degrees otherwise
  }

  // Read IR sensors and control motors
  int ir1 = digitalRead(IR1_PIN);
  int ir2 = digitalRead(IR2_PIN);
  int ir3 = digitalRead(IR3_PIN);
  int ir4 = digitalRead(IR4_PIN);

  if (ir1 == LOW || ir2 == LOW) {
    // Obstacle on left, turn right
    analogWrite(MOTOR_ENA, 128);
    analogWrite(MOTOR_ENB, 128);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, HIGH);
    digitalWrite(MOTOR_IN4, LOW);
  } else if (ir3 == LOW || ir4 == LOW) {
    // Obstacle on right, turn left
    analogWrite(MOTOR_ENA, 128);
    analogWrite(MOTOR_ENB, 128);
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, HIGH);
  } else {
    // Move forward if no obstacles
    analogWrite(MOTOR_ENA, 128);
    analogWrite(MOTOR_ENB, 128);
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, HIGH);
    digitalWrite(MOTOR_IN4, LOW);
  }

  delay(100); // Small delay for stability
}