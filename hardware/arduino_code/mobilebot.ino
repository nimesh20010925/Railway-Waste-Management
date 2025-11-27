#include <Robojax_L298N_DC_motor.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <WiFi.h>

// ==================== FLAG STRUCTURE ====================
typedef struct {
  uint8_t BTFlag         : 1;
  uint8_t L298NFlag      : 1;
  uint8_t HCSR04Flag     : 1;
  uint8_t LEDFlag        : 1;
  uint8_t ServoFlag      : 1;
  uint8_t initial_Flag   : 1;
  uint8_t FunctionFlag   : 1;
  uint8_t back_light_Flag : 1;
  uint8_t front_light_Flag : 1;
} vFlag_t;

vFlag_t flag = {0};
BluetoothSerial SerialBT;

// ==================== WiFi & YOLO Server ====================
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASS";
WiFiServer server(8080);

// ==================== SERVO SETUP ====================
Servo servoScan, armLeft, armRight, gripLeft, gripRight;
#define SERVO_SCAN_PIN      13
#define ARM_LEFT_PIN        32
#define ARM_RIGHT_PIN       33
#define GRIP_LEFT_PIN       25
#define GRIP_RIGHT_PIN      26

// Servo positions (tuned for smooth motion)
const int ARM_UP     = 60;
const int ARM_DOWN   = 120;
const int ARM_NEUTRAL= 90;
const int GRIP_OPEN  = 60;
const int GRIP_CLOSED= 110;

// ==================== PINS ====================
#define LED_BUILTIN         2
#define TRIGPIN_PIN         12
#define ECHO_PIN            14
#define back_light          21
#define front_light         22
#define CAM_TRIGGER         23

// IR Edge Sensors
#define IR_LEFT             34
#define IR_RIGHT            35
#define IR_FRONT_LEFT       36
#define IR_FRONT_RIGHT      39

// L298N Motor Driver (using library properly now)
const int IN1 = 16, IN2 = 17, ENA = 4;
const int IN3 = 18, IN4 = 19, ENB = 5;
const int CHA = 0, CHB = 1;
Robojax_L298N_DC_motor motor(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB, true); // true = debug on

// ==================== GLOBAL VARIABLES ====================
#define MAX_DISTANCE        200
#define MAX_SPEED           220
#define OBSTACLE_DISTANCE   35
#define EDGE_DETECT         LOW

int currentSpeed = 0;
int targetSpeed = 0;
unsigned long previousMillis = 0;
const long interval = 10;  // Motor ramp interval

int distance = 0;
unsigned long lastSensorRead = 0;

// Task handles
TaskHandle_t TaskSensor, TaskBluetooth;

// ==================== NON-BLOCKING TIMERS ====================
unsigned long actionTimer = 0;
bool actionInProgress = false;
enum ActionState { IDLE, APPROACH, LOWER_ARM, CLOSE_GRIP, LIFT_ARM, REVERSE, RELEASE, RESET_ARM };
ActionState pickupState = IDLE;

// ==================== SMOOTH MOTOR CONTROL ====================
void setMotorSpeed(int speed) {
  targetSpeed = constrain(speed, -255, 255);
}

void updateMotorSpeed() {
  unsigned long now = millis();
  if (now - previousMillis >= interval) {
    previousMillis = now;
    if (currentSpeed < targetSpeed) currentSpeed += 8;
    else if (currentSpeed > targetSpeed) currentSpeed -= 8;
    currentSpeed = constrain(currentSpeed, -255, 255);

    if (currentSpeed > 0) {
      motor.rotate(1, abs(currentSpeed), 1);  // Motor 1 Forward
      motor.rotate(2, abs(currentSpeed), 1);  // Motor 2 Forward
    } else if (currentSpeed < 0) {
      motor.rotate(1, abs(currentSpeed), 2);  // Backward
      motor.rotate(2, abs(currentSpeed), 2);
    } else {
      motor.brake(1);
      motor.brake(2);
    }
  }
}

void Forward()  { setMotorSpeed(180); }
void Reverse()  { setMotorSpeed(-160); }
void Left()     { motor.rotate(1, 140, 1); motor.rotate(2, 140, 2); }
void Right()    { motor.rotate(1, 140, 2); motor.rotate(2, 140, 1); }
void Stop()     { setMotorSpeed(0); }

// ==================== SMOOTH SERVO CONTROL ====================
void smoothServo(Servo &servo, int target, int stepDelay = 15) {
  int current = servo.read();
  while (abs(current - target) > 2) {
    current += (current < target) ? 2 : -2;
    servo.write(current);
    delay(stepDelay);
  }
  servo.write(target);
}

// ==================== ROBOT ARM ACTIONS ====================
void liftArmUp()    { smoothServo(armLeft, ARM_UP); smoothServo(armRight, ARM_UP); }
void lowerArm()     { smoothServo(armLeft, ARM_DOWN); smoothServo(armRight, ARM_DOWN); }
void openGrip()     { smoothServo(gripLeft, GRIP_OPEN); smoothServo(gripRight, GRIP_OPEN); }
void closeGrip()    { smoothServo(gripLeft, GRIP_CLOSED); smoothServo(gripRight, GRIP_CLOSED); }
void resetArm()     { smoothServo(armLeft, ARM_NEUTRAL); smoothServo(armRight, ARM_NEUTRAL); }

// ==================== NON-BLOCKING PICKUP SEQUENCE ====================
void startPickup() {
  if (actionInProgress) return;
  actionInProgress = true;
  pickupState = APPROACH;
  actionTimer = millis();
  Serial.println("Starting smooth waste pickup sequence...");
}

void updatePickupSequence() {
  if (!actionInProgress) return;

  unsigned long now = millis();

  switch (pickupState) {
    case APPROACH:     Forward(); if (now - actionTimer > 800)  { Stop(); pickupState = LOWER_ARM; actionTimer = now; } break;
    case LOWER_ARM:    lowerArm(); if (now - actionTimer > 900)  { pickupState = CLOSE_GRIP; actionTimer = now; } break;
    case CLOSE_GRIP:   closeGrip(); if (now - actionTimer > 800) { pickupState = LIFT_ARM; actionTimer = now; } break;
    case LIFT_ARM:     liftArmUp(); if (now - actionTimer > 1000) { pickupState = REVERSE; actionTimer = now; } break;
    case REVERSE:      Reverse(); if (now - actionTimer > 1000) { Stop(); pickupState = RELEASE; actionTimer = now; } break;
    case RELEASE:      openGrip(); if (now - actionTimer > 800)  { pickupState = RESET_ARM; actionTimer = now; } break;
    case RESET_ARM:    resetArm(); if (now - actionTimer > 800)  { pickupState = IDLE; actionInProgress = false; Serial.println("Waste pickup completed smoothly!"); } break;
    case IDLE: break;
  }
}

// ==================== EDGE DETECTION (Non-blocking) ====================
bool checkEdges() {
  if (digitalRead(IR_FRONT_LEFT) == EDGE_DETECT || digitalRead(IR_FRONT_RIGHT) == EDGE_DETECT) {
    Serial.println("FRONT EDGE DETECTED! Emergency reverse + turn");
    Stop(); delay(100);
    Reverse(); delay(500);
    Left(); delay(600);
    Stop();
    return true;
  }
  if (digitalRead(IR_LEFT) == EDGE_DETECT) { Right(); delay(500); Stop(); return true; }
  if (digitalRead(IR_RIGHT) == EDGE_DETECT) { Left(); delay(500); Stop(); return true; }
  return false;
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("WasteBot_BT");
  Serial.println("Smart Waste Collection Robot v2.0 - SMOOTH EDITION");

  // Initialize pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIGPIN_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(back_light, OUTPUT);
  pinMode(front_light, OUTPUT);
  pinMode(CAM_TRIGGER, OUTPUT);
  digitalWrite(front_light, HIGH);
  digitalWrite(CAM_TRIGGER, LOW);

  // Servos
  servoScan.attach(SERVO_SCAN_PIN, 500, 2400);
  armLeft.attach(ARM_LEFT_PIN);
  armRight.attach(ARM_RIGHT_PIN);
  gripLeft.attach(GRIP_LEFT_PIN);
  gripRight.attach(GRIP_RIGHT_PIN);
  servoScan.write(90);
  resetArm();
  openGrip();

  // Motor
  motor.begin();

  // WiFi
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());
    server.begin();
  } else {
    Serial.println("\nWiFi Failed");
  }

  // Tasks
  xTaskCreatePinnedToCore(taskSensor, "SensorTask", 4096, NULL, 2, &TaskSensor, 1);
  xTaskCreatePinnedToCore(taskBluetooth, "BTTask", 4096, NULL, 2, &TaskBluetooth, 0);

  Serial.println("Robot Ready - Autonomous Mode Active");
}

// ==================== MAIN LOOP ====================
void loop() {
  updateMotorSpeed();
  updatePickupSequence();
  checkEdges();

  // Normal patrol if not picking up
  if (!actionInProgress && !checkEdges()) {
    if (distance > OBSTACLE_DISTANCE) {
      Forward();
    } else {
      Stop();
      // Trigger camera only when close enough
      digitalWrite(CAM_TRIGGER, HIGH);
      delay(100);
      WiFiClient client = server.available();
      if (client) {
        String cmd = client.readStringUntil('\n');
        cmd.trim();
        if (cmd == "PICK") startPickup();
        client.stop();
      }
      digitalWrite(CAM_TRIGGER, LOW);
    }
  }

  vTaskDelay(10);
}

// ==================== SENSOR TASK ====================
void taskSensor(void *pvParameters) {
  for (;;) {
    servoScan.write(90);
    digitalWrite(TRIGPIN_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    distance = (duration > 0) ? duration / 58 : MAX_DISTANCE;

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ==================== BLUETOOTH TASK ====================
void taskBluetooth(void *pvParameters) {
  for (;;) {
    if (SerialBT.available()) {
      String cmd = SerialBT.readStringUntil('\n');
      cmd.trim();
      Stop();
      actionInProgress = false;  // Cancel auto pickup if manual

      if (cmd == "FS") Forward();
      else if (cmd == "BS") Reverse();
      else if (cmd == "LS") Left();
      else if (cmd == "RS") Right();
      else if (cmd == "S") Stop();
      else if (cmd == "PU") startPickup();
      else if (cmd == "LED_ON") digitalWrite(front_light, HIGH);
      else if (cmd == "LED_OFF") digitalWrite(front_light, LOW);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}