#include <Arduino.h>
#include <Servo.h>

// -------------------- Pin Definitions --------------------
#define ENA 4
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define ENB 5

#define TRIG_PIN 12
#define ECHO_PIN 14

#define SERVO_PIN 13

#define IR_LEFT 25
#define IR_RIGHT 26
#define IR_FRONT_LEFT 27
#define IR_FRONT_RIGHT 33

// -------------------- Variables --------------------
Servo myservo;

long duration;
float distance;
int irLeft, irRight, irFrontLeft, irFrontRight;

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // IR sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FRONT_LEFT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);

  // Servo setup
  myservo.attach(SERVO_PIN);
  myservo.write(90); // Neutral position

  Serial.println("System initialized: Obstacle Avoidance Active");
}

// -------------------- Functions --------------------
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  distance = duration * 0.034 / 2;
  return distance;
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
}

// -------------------- Main Loop --------------------
void loop() {
  float distance = getDistance();
  irLeft = digitalRead(IR_LEFT);
  irRight = digitalRead(IR_RIGHT);
  irFrontLeft = digitalRead(IR_FRONT_LEFT);
  irFrontRight = digitalRead(IR_FRONT_RIGHT);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | IRs: ");
  Serial.print(irLeft);
  Serial.print(irRight);
  Serial.print(irFrontLeft);
  Serial.println(irFrontRight);

  // Avoid platform edge or obstacle
  if (distance < 15 || irFrontLeft == LOW || irFrontRight == LOW) {
    stopMotors();
    myservo.write(0); // move servo to avoid obstacle (optional)
    delay(300);
    moveBackward();
    delay(400);
    turnRight();
    delay(400);
  }
  else if (irLeft == LOW) {
    turnRight();
    delay(300);
  }
  else if (irRight == LOW) {
    turnLeft();
    delay(300);
  }
  else {
    moveForward();
  }

  delay(100);
}
