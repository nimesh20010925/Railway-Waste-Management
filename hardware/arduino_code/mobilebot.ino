#include <Robojax_L298N_DC_motor.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>

// ==================== ESP-NOW SETUP ====================
uint8_t slaveMac[] = {0x24, 0x6F, 0x28, 0xXX, 0xXX, 0xXX}; // CHANGE TO YOUR ESP32-CAM MAC
String yoloResult = "";
bool yoloReady = false;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  char buf[10];
  memcpy(buf, data, len);
  buf[len] = '\0';
  yoloResult = String(buf);
  yoloReady = true;
  Serial.println("YOLO: " + yoloResult);
}

// ==================== YOUR ORIGINAL FLAGS ====================
typedef struct _vFlag {
  uint8_t BTFlag = 0;
  uint8_t L298NFlag = 0;
  uint8_t HCSR04Flag = 1;
  uint8_t LEDFlag = 1;
  uint8_t ServoFlag = 0;
  uint8_t initial_Flag = 0;
  uint8_t FunctionFlag = 0;
  uint8_t back_light_Flag = 0;
  uint8_t front_light_Flag = 0;
} vFlag;
vFlag flag;

// ==================== YOUR ORIGINAL HARDWARE ====================
BluetoothSerial SerialBT;
WiFiServer server(8080);
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASS";

Servo servoScan;
#define SERVO_SCAN_PIN 13

#define LED_BUILTIN 2
#define CHA 0
#define ENA 4
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define ENB 5
#define CHB 1
const int CCW = 2;
const int CW = 1;
Robojax_L298N_DC_motor motors(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB);

#define back_light 21
#define front_light 22
#define TRIGPIN_PIN 12
#define ECHO_PIN 14
long duration;
int distance = 60;
int distanceR = 0;
int distanceL = 0;

#define IR_LEFT 34
#define IR_RIGHT 35
#define IR_FRONT_LEFT 36
#define IR_FRONT_RIGHT 39

#define MAX_DISTANCE 200

TaskHandle_t huart;
TaskHandle_t hfunction;

// ==================== NEW: lookLeft & lookRight ====================
int lookRight() {
  servoScan.write(0);           // Look fully right
  delay(800);
  digitalWrite(TRIGPIN_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int d = (duration > 0) ? duration / 58 : MAX_DISTANCE;
  servoScan.write(90);
  delay(300);
  Serial.println("Look Right: " + String(d) + " cm");
  return d;
}

int lookLeft() {
  servoScan.write(180);         // Look fully left
  delay(800);
  digitalWrite(TRIGPIN_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int d = (duration > 0) ? duration / 58 : MAX_DISTANCE;
  servoScan.write(90);
  delay(300);
  Serial.println("Look Left: " + String(d) + " cm");
  return d;
}

// ==================== YOUR ORIGINAL MOTOR FUNCTIONS (UNCHANGED) ====================
void Forward()  { Serial.println("Moving Forward");  motors.rotate(1, 85, CW);  motors.rotate(2, 85, CW); }
void Reverse()  { Serial.println("Moving Backward"); motors.rotate(1, 85, CCW); motors.rotate(2, 85, CCW); }
void Left()     { Serial.println("Turning Left");    motors.rotate(1, 80, CW);  motors.rotate(2, 80, CCW); }
void Right()    { Serial.println("Turning Right");   motors.rotate(1, 80, CCW); motors.rotate(2, 80, CW); }
void Stop()     { Serial.println("Stopping");       motors.brake(1); motors.brake(2); }

// ==================== YOUR ORIGINAL SETUP (ONLY ESP-NOW ADDED) ====================
void setup() {
  Serial.begin(115200);
  Serial.println("Smart Waste Robot #1 (Navigation) Starting...");

  SerialBT.begin("WasteBot_BT");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(TRIGPIN_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(back_light, OUTPUT);
  pinMode(front_light, OUTPUT);
  digitalWrite(front_light, HIGH);

  pinMode(IR_LEFT, INPUT); pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FRONT_LEFT, INPUT); pinMode(IR_FRONT_RIGHT, INPUT);

  motors.begin();
  servoScan.attach(SERVO_SCAN_PIN, 500, 2400);
  servoScan.write(90);

  // ESP-NOW Init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    while(1);
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slaveMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  xTaskCreatePinnedToCore(vUARTTask, "UARTTask", 4096, NULL, 3, &huart, 0);
  xTaskCreatePinnedToCore(vFunctionTask, "FunctionTask", 4096, NULL, 1, &hfunction, 1);

  Serial.println("Navigation ESP32 Ready!");
}

// ==================== YOUR ORIGINAL TASKS (ONLY ADDED lookLeft/lookRight LOGIC) ====================
void vFunctionTask(void *pvParameters) {
  for (;;) {
    servoScan.write(90);
    delay(50);
    digitalWrite(TRIGPIN_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPIN_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH, 30000);
    distance = (duration == 0) ? 999 : duration / 58;
    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void vUARTTask(void *pvParameters) {
  for (;;) {
    if (SerialBT.available()) {
      String cmd = SerialBT.readStringUntil('\n');
      cmd.trim();
      Stop();
      if (cmd == "FS") Forward();
      else if (cmd == "BS") Reverse();
      else if (cmd == "LS") Left();
      else if (cmd == "RS") Right();
      else if (cmd == "S") Stop();
      else if (cmd == "PU") pickUpWaste();
    }
    vTaskDelay(10);
  }
}

void pickUpWaste() {
  Serial.println("Waste Pickup Sequence Triggered");
  Forward(); delay(600); Stop();
  Reverse(); delay(800); Stop();
}

// ==================== MAIN LOOP WITH lookLeft/lookRight (ONLY ADDED) ====================
void loop() {
  while (1) {
    // Edge Detection (Your original safety)
    if (digitalRead(IR_FRONT_LEFT) == LOW || digitalRead(IR_FRONT_RIGHT) == LOW) {
      Stop(); Reverse(); delay(600); Stop(); Left(); delay(400); continue;
    }
    if (digitalRead(IR_LEFT) == LOW) { Right(); delay(500); Stop(); continue; }
    if (digitalRead(IR_RIGHT) == LOW) { Left(); delay(500); Stop(); continue; }

    // Smart Obstacle Avoidance with lookLeft/lookRight
    if (distance <= 30) {
      Stop(); delay(200);
      Reverse(); delay(400); Stop();
      distanceR = lookRight();
      delay(400);
      distanceL = lookLeft();
      delay(400);

      if (distanceR > distanceL && distanceR > 30) {
        Right(); delay(600);
      } else if (distanceL > 30) {
        Left(); delay(600);
      } else {
        Reverse(); delay(800);
      }
      Stop();
    }
    // Waste Detection Range
    else if (distance <= 35) {
      Stop();
      Serial.println("Object in range! Triggering Camera...");
      esp_now_send(slaveMac, (uint8_t*)"TRIGGER", 7);
      yoloReady = false;
      int timeout = 8000;
      while (!yoloReady && timeout > 0) { delay(10); timeout -= 10; }

      if (yoloReady && yoloResult == "PICK") {
        pickUpWaste();
      }
      yoloReady = false;
      yoloResult = "";
      Forward();
    }
    else {
      Forward();
    }
    vTaskDelay(50);
  }
}

#include <WiFi.h>
#include "esp_camera.h"
#include <ESP32Servo.h>
#include <esp_now.h>

Servo armLeft, armRight, gripLeft, gripRight;

#define ARM_LEFT_PIN 12
#define ARM_RIGHT_PIN 13
#define GRIP_LEFT_PIN 14
#define GRIP_RIGHT_PIN 15

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  String cmd = "";
  for (int i = 0; i < len; i++) cmd += (char)data[i];

  if (cmd == "TRIGGER") {
    takePhotoAndDetect();
  }
}

void takePhotoAndDetect() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    esp_now_send(mac, (uint8_t*)"ERROR", 5);
    return;
  }

  // Simulate YOLO detection (replace with real YOLO later)
  // For now: 70% chance to detect waste
  bool isWaste = (random(0, 100) < 70);

  if (isWaste) {
    esp_now_send(mac, (uint8_t*)"PICK", 4);
    performPickup();
  } else {
    esp_now_send(mac, (uint8_t*)"IGNORE", 6);
  }
  esp_camera_fb_return(fb);
}

void performPickup() {
  armLeft.write(60); armRight.write(120);
  delay(800);
  gripLeft.write(100); gripRight.write(80);
  delay(600);
  armLeft.write(30); armRight.write(150);
  delay(1000);
  gripLeft.write(60); gripRight.write(120);
  delay(600);
  armLeft.write(90); armRight.write(90);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Camera config here (use standard AI-Thinker config)

  if (esp_now_init() != ESP_OK) while(1);

  armLeft.attach(ARM_LEFT_PIN);
  armRight.attach(ARM_RIGHT_PIN);
  gripLeft.attach(GRIP_LEFT_PIN);
  gripRight.attach(GRIP_RIGHT_PIN);

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Vision + Arm ESP32 Ready");
}

void loop() {
  delay(100);
}