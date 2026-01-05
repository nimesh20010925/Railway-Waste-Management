#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

/* =============================
   WIFI CONFIG
   ============================= */
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

/* =============================
   ULTRASONIC
   ============================= */
#define TRIG_PIN 5
#define ECHO_PIN 18
#define DETECT_DISTANCE 12  // cm

/* =============================
   SERVO PINS (4 SERVOS)
   ============================= */
#define LEFT_ARM_SERVO_PIN      13
#define LEFT_GRIPPER_SERVO_PIN  12
#define RIGHT_ARM_SERVO_PIN     14
#define RIGHT_GRIPPER_SERVO_PIN 27

Servo leftArm;
Servo leftGripper;
Servo rightArm;
Servo rightGripper;

/* =============================
   ROBOT STATES
   ============================= */
enum RobotState {
  IDLE,
  OBJECT_DETECTED,
  WAIT_FOR_AI,
  GRAB_OBJECT,
  RESET_ARM
};

RobotState state = IDLE;

/* =============================
   WEB SERVER
   ============================= */
WebServer server(80);

/* =============================
   ULTRASONIC FUNCTION
   ============================= */
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;

  return duration * 0.034 / 2;
}

/* =============================
   ARM ACTIONS
   ============================= */
void openGrippers() {
  leftGripper.write(90);
  rightGripper.write(90);
}

void closeGrippers() {
  leftGripper.write(40);
  rightGripper.write(40);
}

void armsDown() {
  leftArm.write(120);
  rightArm.write(60);  // mirrored
}

void armsUp() {
  leftArm.write(90);
  rightArm.write(90);
}

void armsToBin() {
  leftArm.write(60);
  rightArm.write(120);
}

/* =============================
   GRAB + DROP SEQUENCE
   ============================= */
void grabAndDrop() {
  openGrippers();
  delay(400);

  armsDown();
  delay(700);

  closeGrippers();
  delay(600);

  armsUp();
  delay(600);

  armsToBin();
  delay(700);

  openGrippers();
  delay(600);

  armsUp();
  delay(500);
}

/* =============================
   RESET
   ============================= */
void resetArm() {
  openGrippers();
  armsUp();
}

/* =============================
   HTTP HANDLERS
   ============================= */
void handleGrab() {
  state = GRAB_OBJECT;
  server.send(200, "text/plain", "GRAB COMMAND RECEIVED");
}

void handleReset() {
  state = RESET_ARM;
  server.send(200, "text/plain", "RESET COMMAND RECEIVED");
}

/* =============================
   SETUP
   ============================= */
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  leftArm.attach(LEFT_ARM_SERVO_PIN);
  leftGripper.attach(LEFT_GRIPPER_SERVO_PIN);
  rightArm.attach(RIGHT_ARM_SERVO_PIN);
  rightGripper.attach(RIGHT_GRIPPER_SERVO_PIN);

  resetArm();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  server.on("/grab", handleGrab);
  server.on("/reset", handleReset);
  server.begin();

  Serial.println("HTTP server started");
}

/* =============================
   LOOP
   ============================= */
void loop() {
  server.handleClient();

  long distance = getDistance();

  switch (state) {

    case IDLE:
      if (distance > 0 && distance < DETECT_DISTANCE) {
        Serial.println("Object detected");
        state = OBJECT_DETECTED;
      }
      break;

    case OBJECT_DETECTED:
      Serial.println("Waiting for AI decision...");
      state = WAIT_FOR_AI;
      break;

    case WAIT_FOR_AI:
      // Waiting for Python YOLO command
      break;

    case GRAB_OBJECT:
      Serial.println("Grabbing object...");
      grabAndDrop();
      state = IDLE;
      break;

    case RESET_ARM:
      resetArm();
      state = IDLE;
      break;
  }

  delay(50);
}
