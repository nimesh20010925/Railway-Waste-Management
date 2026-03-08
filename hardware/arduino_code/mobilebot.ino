Nimesh SLIIT Piliyandala, [3/8/2026 4:48 PM]
#include <WiFi.h> 
#include <ESP32Servo.h> 
#include <NewPing.h> 
 
// ========================================================== 
// WIFI 
// ========================================================== 
const char* ssid = "Dialog 4G"; 
const char* password = "0R9MGJ621H8"; 
WiFiServer server(80); 
 
// ========================================================== 
// MODES 
// ========================================================== 
enum NavigationMode { 
  HEAD_COUNT_MODE, 
  WASTE_COLLECTION_MODE 
}; 
 
NavigationMode currentMode = HEAD_COUNT_MODE; 
 
bool shouldMove = true; 
bool wasteCollectionActive = false; 
 
unsigned long modeStartTime = 0; 
const unsigned long WASTE_COLLECTION_TIMEOUT = 20000; 
 
// ========================================================== 
// MOTOR PINS 
// ========================================================== 
#define LM_F 17 
#define LM_B 16 
#define RM_F 18 
#define RM_B 19 
 
#define ENA 4 
#define ENB 21 
 
int speedSet = 120;   // Reduced speed 
 
// ========================================================== 
// ULTRASONIC 
// ========================================================== 
#define TRIG_FRONT 12 
#define ECHO_FRONT 14 
 
#define TRIG_FL 32 
#define ECHO_FL 33 
 
#define TRIG_FR 25 
#define ECHO_FR 26 
 
#define TRIG_BL 27 
#define ECHO_BL 2 
 
#define TRIG_BR 15 
#define ECHO_BR 5 
 
#define TRIG_BIN 22 
#define ECHO_BIN 23 
 
#define MAX_DISTANCE 250 
 
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE); 
NewPing sonarFL(TRIG_FL, ECHO_FL, MAX_DISTANCE); 
NewPing sonarFR(TRIG_FR, ECHO_FR, MAX_DISTANCE); 
NewPing sonarBL(TRIG_BL, ECHO_BL, MAX_DISTANCE); 
NewPing sonarBR(TRIG_BR, ECHO_BR, MAX_DISTANCE); 
NewPing sonarBin(TRIG_BIN, ECHO_BIN, MAX_DISTANCE); 
 
// ========================================================== 
// SERVO 
// ========================================================== 
#define SERVO_PIN 13 
Servo servoMotor; 
 
int centerPos = 90; 
 
// ========================================================== 
// THRESHOLDS 
// ========================================================== 
int safeDistance = 20; 
int edgeThreshold = 15; 
int binFullLevel = 10; 
 
// ========================================================== 
// SETUP 
// ========================================================== 
void setup() { 
 
  Serial.begin(115200); 
 
  pinMode(LM_F, OUTPUT); 
  pinMode(LM_B, OUTPUT); 
  pinMode(RM_F, OUTPUT); 
  pinMode(RM_B, OUTPUT); 
 
  // PWM (ESP32 CORE 3.x) 
  ledcAttach(ENA, 1000, 8); 
  ledcAttach(ENB, 1000, 8); 
 
  // Servo 
  ESP32PWM::allocateTimer(0); 
  servoMotor.setPeriodHertz(50); 
  servoMotor.attach(SERVO_PIN, 500, 2500); 
 
  servoMotor.write(centerPos); 
  delay(1000); 
 
  Serial.println("Robot Ready"); 
 
  // WiFi 
  Serial.print("Connecting WiFi"); 
 
  WiFi.begin(ssid, password); 
 
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500); 
    Serial.print("."); 
  } 
 
  Serial.println(); 
  Serial.print("Robot IP: "); 
  Serial.println(WiFi.localIP()); 
 
  server.begin(); 
} 
 
// ========================================================== 
// LOOP 
// ========================================================== 
void loop() { 
 
  if (currentMode == WASTE_COLLECTION_MODE && 
      millis() - modeStartTime > WASTE_COLLECTION_TIMEOUT) { 
 
    currentMode = HEAD_COUNT_MODE; 
    wasteCollectionActive = false; 
  } 
 
  WiFiClient client = server.available(); 
 
  if (client) { 
 
    String request = client.readStringUntil('\r'); 
    Serial.println(request); 
 
    if (request.indexOf("/move") != -1) shouldMove = true; 
    if (request.indexOf("/stop") != -1) shouldMove = false; 
 
    if (request.indexOf("/collection_start") != -1) { 
      currentMode = WASTE_COLLECTION_MODE; 
      wasteCollectionActive = true; 
      modeStartTime = millis(); 
    } 
 
    if (request.indexOf("/collection_end") != -1) { 
      currentMode = HEAD_COUNT_MODE; 
      wasteCollectionActive = false; 
    } 
 
    client.println("HTTP/1.1 200 OK"); 
    client.println("Content-Type: text/plain"); 
    client.println(); 
    client.println("OK");

Nimesh SLIIT Piliyandala, [3/8/2026 4:48 PM]
client.stop(); 
  } 
 
  navigate(); 
 
  delay(50); 
} 
 
// ========================================================== 
// NAVIGATION 
// ========================================================== 
void navigate() { 
 
  if (currentMode == WASTE_COLLECTION_MODE) { 
    moveStop(); 
    return; 
  } 
 
  if (!shouldMove) { 
    moveStop(); 
    return; 
  } 
 
  // BIN CHECK 
  int bin = readDistance(sonarBin); 
 
  if (bin <= binFullLevel) { 
    Serial.println("BIN FULL"); 
    moveStop(); 
    return; 
  } 
 
  // FLOOR SENSORS 
  int fl = readDistance(sonarFL); 
  int fr = readDistance(sonarFR); 
  int bl = readDistance(sonarBL); 
  int br = readDistance(sonarBR); 
 
  bool gFL = fl < edgeThreshold; 
  bool gFR = fr < edgeThreshold; 
  bool gBL = bl < edgeThreshold; 
  bool gBR = br < edgeThreshold; 
 
  // EDGE DETECTION 
  if (!gFL  !gFR  !gBL || !gBR) { 
 
    Serial.println("EDGE DETECTED"); 
 
    moveStop(); 
    delay(100); 
 
    if (!gFL && !gFR) { 
 
      Serial.println("FRONT EDGE"); 
 
      moveBackward(); 
      delay(1000); 
 
      turnRight(); 
      delay(1200); 
    } 
 
    else if (!gBL && !gBR) { 
 
      Serial.println("BACK EDGE"); 
 
      moveForward(); 
      delay(500); 
    } 
 
    else if (!gFL || !gBL) { 
 
      Serial.println("LEFT EDGE"); 
 
      turnRight(); 
      delay(500); 
    } 
 
    else if (!gFR || !gBR) { 
 
      Serial.println("RIGHT EDGE"); 
 
      turnLeft(); 
      delay(500); 
    } 
 
    moveStop(); 
    return; 
  } 
 
  // OBSTACLE DETECTION 
  int front = readDistance(sonarFront); 
 
  if (front <= safeDistance) { 
 
    Serial.println("Obstacle"); 
 
    moveStop(); 
    delay(100); 
 
    moveBackward(); 
    delay(800); 
 
    moveStop(); 
 
    int right = lookRight(); 
    int left = lookLeft(); 
 
    if (right > left) { 
 
      turnRight(); 
      delay(500); 
    } 
 
    else { 
 
      turnLeft(); 
      delay(500); 
    } 
 
    moveStop(); 
  } 
 
  else { 
 
    moveForward(); 
  } 
} 
 
// ========================================================== 
// SERVO SCAN 
// ========================================================== 
int lookRight() { 
 
  servoMotor.write(30); 
  delay(400); 
 
  int d = readDistance(sonarFront); 
 
  servoMotor.write(centerPos); 
  delay(200); 
 
  return d; 
} 
 
int lookLeft() { 
 
  servoMotor.write(150); 
  delay(400); 
 
  int d = readDistance(sonarFront); 
 
  servoMotor.write(centerPos); 
  delay(200); 
 
  return d; 
} 
 
// ========================================================== 
// DISTANCE 
// ========================================================== 
int readDistance(NewPing &sonar) { 
 
  delay(30); 
 
  int d = sonar.ping_cm(); 
 
  if (d == 0) return MAX_DISTANCE; 
 
  return d; 
} 
 
// ========================================================== 
// MOTOR CONTROL 
// ========================================================== 
void moveForward() { 
 
  digitalWrite(LM_F, HIGH); 
  digitalWrite(RM_F, HIGH); 
 
  digitalWrite(LM_B, LOW); 
  digitalWrite(RM_B, LOW); 
 
  ledcWrite(ENA, speedSet); 
  ledcWrite(ENB, speedSet); 
} 
 
void moveBackward() { 
 
  digitalWrite(LM_B, HIGH); 
  digitalWrite(RM_B, HIGH); 
 
  digitalWrite(LM_F, LOW); 
  digitalWrite(RM_F, LOW); 
 
  ledcWrite(ENA, speedSet); 
  ledcWrite(ENB, speedSet); 
} 
 
void turnRight() { 
 
  digitalWrite(LM_F, HIGH); 
  digitalWrite(RM_B, HIGH); 
 
  digitalWrite(LM_B, LOW); 
  digitalWrite(RM_F, LOW); 
 
  ledcWrite(ENA, speedSet); 
  ledcWrite(ENB, speedSet); 
} 
 
void turnLeft() { 
 
  digitalWrite(LM_B, HIGH); 
  digitalWrite(RM_F, HIGH); 
 
  digitalWrite(LM_F, LOW); 
  digitalWrite(RM_B, LOW); 
 
  ledcWrite(ENA, speedSet); 
  ledcWrite(ENB, speedSet); 
} 
 
void moveStop() { 
 
  digitalWrite(LM_F, LOW); 
  digitalWrite(RM_F, LOW); 
  digitalWrite(LM_B, LOW); 
  digitalWrite(RM_B, LOW); 
 
  ledcWrite(ENA, 0); 
  ledcWrite(ENB, 0); 
}