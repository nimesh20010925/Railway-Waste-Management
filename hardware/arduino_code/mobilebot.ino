#include <Robojax_L298N_DC_motor.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <WiFi.h>

// ----------- Flag structure ------------------------
typedef struct _vFlag {
  uint8_t BTFlag = 0;        // Bluetooth control flag
  uint8_t L298NFlag = 0;     // Motor driver status
  uint8_t HCSR04Flag = 1;    // Ultrasonic sensor enable
  uint8_t LEDFlag = 1;       // LED status indicator
  uint8_t ServoFlag = 0;     // Servo control flag
  uint8_t initial_Flag = 0;  // Initialization status
  uint8_t FunctionFlag = 0;  // General function flag
  uint8_t back_light_Flag = 0;  // Rear lights
  uint8_t front_light_Flag = 0; // Front lights
} vFlag;
vFlag flag;

// ----------- Bluetooth -----------------------------
BluetoothSerial SerialBT;

// ----------- Wi-Fi Server (for YOLO connection) ----
const char* ssid = "YOUR_WIFI_SSID";      // <-- Change to your WiFi
const char* password = "YOUR_WIFI_PASS";  // <-- Change to your password
WiFiServer server(8080);  // Server for receiving YOLO detection commands

// ----------- Servo Setup ---------------------------
Servo servoScan;      // Servo for rotating ultrasonic sensor
Servo armLeft;        // Left arm servo
Servo armRight;       // Right arm servo  
Servo gripLeft;       // Left gripper servo
Servo gripRight;      // Right gripper servo

// Servo pins - No conflicts
#define SERVO_SCAN_PIN 13   // Servo to scan with ultrasonic
#define ARM_LEFT_PIN 32     // Left arm lift servo
#define ARM_RIGHT_PIN 33    // Right arm lift servo
#define GRIP_LEFT_PIN 25    // Left gripper servo
#define GRIP_RIGHT_PIN 26   // Right gripper servo

// ----------- LED Indicators ------------------------
#define LED_BUILTIN 2       // ESP32 built-in LED

// ----------- L298N Motor Driver --------------------
#define CHA 0               // Channel A
#define ENA 4               // Enable pin motor A
#define IN1 16              // Motor A input 1
#define IN2 17              // Motor A input 2
#define IN3 18              // Motor B input 1
#define IN4 19              // Motor B input 2
#define ENB 5               // Enable pin motor B
#define CHB 1               // Channel B
const int CCW = 2;          // Counter-clockwise
const int CW  = 1;          // Clockwise
#define motor1 1            // Motor 1 identifier
#define motor2 2            // Motor 2 identifier
Robojax_L298N_DC_motor motors(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB);

// ----------- Lights --------------------------------
#define back_light 21       // Rear indicator light
#define front_light 22      // Front headlight

// ----------- Ultrasonic Sensor (Obstacle Detection) --
#define TRIGPIN_PIN 12      // Trigger pin
#define ECHO_PIN 14         // Echo pin
long duration;              // Time for sound wave return
int distance = 60;          // Measured distance in cm
int distanceR = 0;          // Right distance measurement
int distanceL = 0;          // Left distance measurement

// ----------- IR Edge Sensors (Cliff Detection) --------
#define IR_LEFT 34          // Left edge detection
#define IR_RIGHT 35         // Right edge detection  
#define IR_FRONT_LEFT 36    // Front-left edge detection
#define IR_FRONT_RIGHT 39   // Front-right edge detection

// ----------- Global Variables -----------------------
unsigned long currentMillis = 0;
#define MAX_DISTANCE 200    // Maximum detection range
#define MAX_SPEED 190       // Maximum motor speed
int speedSet = 0;           // Current speed setting

// ----------- Wi-Fi Camera Trigger -------------------
#define CAM_TRIGGER 23      // Pin to activate ESP32-CAM

// ----------- FreeRTOS Task Handles ------------------
TaskHandle_t huart;         // Bluetooth/UART task handle
TaskHandle_t hfunction;     // Sensor reading task handle

// ----------- Function Prototypes --------------------
void initial();
void vUARTTask(void *pvParameters);
void vFunctionTask(void *pvParameters);
void pickUpWaste();
void Forward();
void Reverse();
void Left();
void Right();
void Stop();
void liftArmUp();
void lowerArm();
void openGrip();
void closeGrip();

// ------------------------------------------------
// SETUP FUNCTION
// ------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("🚀 Smart Waste Collection Robot Initializing...");
  
  // Initialize Bluetooth
  SerialBT.begin("WasteBot_BT"); // Bluetooth device name
  
  // Initialize all pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Indicate power on
  
  // Ultrasonic sensor pins
  pinMode(TRIGPIN_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Motor control pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Light pins
  pinMode(back_light, OUTPUT); 
  pinMode(front_light, OUTPUT);
  digitalWrite(front_light, HIGH); // Turn on front light
  
  // IR sensor pins (for edge detection)
  pinMode(IR_LEFT, INPUT); 
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FRONT_LEFT, INPUT); 
  pinMode(IR_FRONT_RIGHT, INPUT);
  
  // Camera trigger pin
  pinMode(CAM_TRIGGER, OUTPUT); 
  digitalWrite(CAM_TRIGGER, LOW); // Start with camera off
  
  // Initialize motor driver
  motors.begin();
  Serial.println("✅ Motor driver initialized");

  // Attach all servos
  servoScan.attach(SERVO_SCAN_PIN, 500, 2400);
  armLeft.attach(ARM_LEFT_PIN);
  armRight.attach(ARM_RIGHT_PIN);
  gripLeft.attach(GRIP_LEFT_PIN);
  gripRight.attach(GRIP_RIGHT_PIN);
  
  // Set servos to initial positions
  servoScan.write(90);    // Center ultrasonic sensor
  armLeft.write(90);      // Neutral arm position
  armRight.write(90);     // Neutral arm position  
  gripLeft.write(90);     // Open gripper
  gripRight.write(90);    // Open gripper
  Serial.println("✅ Servos initialized and centered");

  // Connect to Wi-Fi for YOLO communication
  Serial.println("📡 Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  
  int wifiTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
    delay(500);
    Serial.print(".");
    wifiTimeout++;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink during connection
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Wi-Fi connected!");
    Serial.print("📱 ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    Serial.println("\n❌ Wi-Fi connection failed!");
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // Start TCP server for YOLO commands
  server.begin();
  Serial.println("✅ TCP server started on port 8080");

  // Create FreeRTOS tasks for parallel processing
  xTaskCreatePinnedToCore(vUARTTask, "UARTTask", 4096, NULL, 3, &huart, 0);
  xTaskCreatePinnedToCore(vFunctionTask, "FunctionTask", 4096, NULL, 1, &hfunction, 1);
  
  Serial.println("✅ FreeRTOS tasks created");
  Serial.println("🎯 Robot Ready! Starting autonomous operation...");
}

// ------------------------------------------------
// MOTOR CONTROL FUNCTIONS
// ------------------------------------------------
void Forward() {
  Serial.println("🔼 Moving Forward");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, 100);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, 100);
}

void Reverse() {
  Serial.println("🔽 Moving Backward");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 100);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 100);
}

void Left() {
  Serial.println("↩️ Turning Left");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, 100);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 100);
}

void Right() {
  Serial.println("↪️ Turning Right");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 100);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, 100);
}

void Stop() {
  Serial.println("🛑 Stopping");
  motors.brake(1);
  motors.brake(2);
}

// ------------------------------------------------
// ROBOT ARM CONTROL FUNCTIONS
// ------------------------------------------------
void liftArmUp() {
  Serial.println("🔼 Lifting arm up");
  armLeft.write(60);   // Adjust angle for up position
  armRight.write(60);
}

void lowerArm() {
  Serial.println("🔽 Lowering arm down");
  armLeft.write(120);  // Adjust angle for down position
  armRight.write(120);
}

void openGrip() {
  Serial.println("🟢 Opening gripper");
  gripLeft.write(60);  // Adjust angle for open position
  gripRight.write(60);
}

void closeGrip() {
  Serial.println("🔴 Closing gripper");
  gripLeft.write(110); // Adjust angle for closed position
  gripRight.write(110);
}

// ------------------------------------------------
// WASTE PICKUP SEQUENCE
// ------------------------------------------------
void pickUpWaste() {
  Serial.println("🗑️ Starting waste pickup sequence...");
  
  // Approach the waste
  Forward(); 
  delay(800); 
  Stop();
  
  // Lower arm to waste level
  lowerArm(); 
  delay(700);
  
  // Close gripper to grab waste
  closeGrip(); 
  delay(600);
  
  // Lift arm with waste
  liftArmUp(); 
  delay(800);
  
  // Back up to disposal area
  Reverse(); 
  delay(900); 
  Stop();
  
  // Release waste into bin
  openGrip(); 
  delay(600);
  
  // Reset arm position
  armLeft.write(90);
  armRight.write(90);
  
  Serial.println("✅ Waste successfully collected and disposed!");
}

// ------------------------------------------------
// MAIN AUTONOMOUS LOOP
// ------------------------------------------------
void loop() {
  while (1) {
    // 🔍 EDGE DETECTION SAFETY SYSTEM
    int leftIR = digitalRead(IR_LEFT);
    int rightIR = digitalRead(IR_RIGHT);
    int frontLeftIR = digitalRead(IR_FRONT_LEFT);
    int frontRightIR = digitalRead(IR_FRONT_RIGHT);

    // Front edge detection - immediate stop and retreat
    if (frontLeftIR == LOW || frontRightIR == LOW) {
      Serial.println("⚠️ 🚨 EDGE DETECTED AT FRONT! Emergency maneuver!");
      Stop(); 
      delay(200);
      Reverse(); 
      delay(400);
      Stop(); 
      delay(100);
      Left(); 
      delay(500);
      Stop(); 
      delay(200);
      continue; // Skip rest of loop and check sensors again
    }
    
    // Left edge detection
    if (leftIR == LOW) {
      Serial.println("⚠️ Edge detected on left!");
      Stop(); 
      delay(200);
      Right(); 
      delay(500);
      Stop(); 
      delay(200);
      continue;
    }
    
    // Right edge detection  
    if (rightIR == LOW) {
      Serial.println("⚠️ Edge detected on right!");
      Stop(); 
      delay(200);
      Left(); 
      delay(500);
      Stop(); 
      delay(200);
      continue;
    }

    // 🎯 OBJECT DETECTION & WASTE COLLECTION SYSTEM
    if (flag.HCSR04Flag == 1) {
      if (distance <= 35) { // Object within pickup range
        Stop(); 
        delay(200);
        Serial.println("🎯 Object detected within 35cm - Activating camera system");
        
        // Activate ESP32-CAM for object recognition
        digitalWrite(CAM_TRIGGER, HIGH);  
        Serial.println("📸 Camera activated - waiting for YOLO analysis...");
        delay(3000); // Wait for camera to initialize and process
        
        // Check for YOLO detection results over WiFi
        WiFiClient client = server.available();
        if (client) {
          Serial.println("🔗 Connected to YOLO system");
          String cmd = client.readStringUntil('\n');
          cmd.trim();
          Serial.print("📥 Received command: ");
          Serial.println(cmd);
          
          if (cmd == "PICK") {
            Serial.println("🔄 Starting automated pickup sequence");
            pickUpWaste();
          } else {
            Serial.println("❌ Object not identified as waste - continuing patrol");
          }
          client.stop();
        } else {
          Serial.println("❓ No classification received - continuing");
        }
        
        // Turn off camera to save power
        digitalWrite(CAM_TRIGGER, LOW);
        Serial.println("📸 Camera deactivated");
        
        // Continue moving after processing
        Forward(); 
        delay(200); 
        Stop();
      } else {
        // Normal patrol movement
        Forward(); 
        delay(100);
        Stop(); 
        delay(30);
      }
    }
    
    // Small delay to prevent watchdog timer issues
    vTaskDelay(1);
  }
}

// ------------------------------------------------
// FREE RTOS TASK 1: ULTRASONIC SENSOR READING
// ------------------------------------------------
void vFunctionTask(void *pvParameters) {
  Serial.println("📊 Ultrasonic sensor task started");
  
  for (;;) {
    if (flag.HCSR04Flag == 1) {
      // Ensure sensor is pointing forward
      servoScan.write(90);
      
      // Generate ultrasonic pulse
      digitalWrite(TRIGPIN_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIGPIN_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGPIN_PIN, LOW);
      
      // Measure echo response time
      duration = pulseIn(ECHO_PIN, HIGH);
      
      // Calculate distance in cm
      distance = duration / 29 / 2;
      
      // Validate distance reading
      if (distance > MAX_DISTANCE) {
        distance = MAX_DISTANCE;
      }
      
      Serial.print("📏 Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }
    
    // Read sensor every 200ms
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// ------------------------------------------------
// FREE RTOS TASK 2: BLUETOOTH MANUAL CONTROL
// ------------------------------------------------
void vUARTTask(void *pvParameters) {
  Serial.println("📱 Bluetooth control task started");
  
  for (;;) {
    // Check for Bluetooth commands
    while (SerialBT.available()) {
      String BTdata = SerialBT.readString();
      BTdata.trim();
      Stop(); // Always stop first for safety
      
      Serial.print("📲 Bluetooth command: ");
      Serial.println(BTdata);
      
      // Process control commands
      if (BTdata == "FS") {
        Serial.println("🎮 Manual: Forward");
        Forward();
      }
      else if (BTdata == "BS") {
        Serial.println("🎮 Manual: Backward");  
        Reverse();
      }
      else if (BTdata == "LS") {
        Serial.println("🎮 Manual: Left");
        Left();
      }
      else if (BTdata == "RS") {
        Serial.println("🎮 Manual: Right");
        Right();
      }
      else if (BTdata == "S") {
        Serial.println("🎮 Manual: Stop");
        Stop();
      }
      else if (BTdata == "PU") {
        Serial.println("🎮 Manual: Pick Up Waste");
        pickUpWaste();
      }
      else if (BTdata == "LED_ON") {
        Serial.println("💡 Manual: Lights ON");
        digitalWrite(front_light, HIGH);
      }
      else if (BTdata == "LED_OFF") {
        Serial.println("💡 Manual: Lights OFF");
        digitalWrite(front_light, LOW);
      }
      else {
        Serial.println("❓ Unknown Bluetooth command");
      }
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}