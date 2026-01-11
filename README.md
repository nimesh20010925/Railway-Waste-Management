# Railway-Waste-Management

## 📌 Project Overview
Railway stations experience large passenger volumes, especially during peak hours, making manual waste collection unsafe, inefficient, and labor-intensive. This research proposes an **intelligent, time-aware, multi-robot waste management system** designed to automate garbage collection, classification, and monitoring while minimizing human involvement.

The system integrates **CCTV-based crowd analysis**, a **mobile garbage-collecting robot with intelligent navigation**, a **stationary robotic waste classification arm**, and a **mobile notification application**. By avoiding peak crowd periods and automating waste handling, the solution improves hygiene, safety, and operational efficiency in railway station environments.


## 🏗 System Architecture
![System Architecture Diagram](assets/architecture_diagram.png)

## 🎯 Research Objectives
- Identify crowded and non-crowded time periods using CCTV-based headcount analysis
- Schedule garbage collection during less crowded periods
- Enable autonomous garbage collection using a mobile robot
- Detect and classify garbage using YOLOv8-based vision models
- Monitor categorized bin fill levels in real time
- Notify cleaning staff through a mobile application


## 🦾 System Components

### 1️⃣ Crowd Analysis Module
- Uses CCTV feeds for headcount estimation
- Prevents robot operation during peak crowd conditions

### 2️⃣ Mobile Garbage-Collecting Robot
- Obstacle avoidance and platform-edge safety
- YOLOv8-assisted garbage detection
- Internal bin fill-level monitoring

### 3️⃣ Stationary Robotic Sorting Arm
- ESP32-CAM-based vision system
- YOLOv8 waste classification
- Four-bin automated sorting mechanism

### 4️⃣ Smart Disposal Bins
- Ultrasonic-based bin fill monitoring
- Prevents overflow and delayed collection

### 5️⃣ Mobile Notification Application
- Real-time bin-full alerts
- Faster response from cleaning staff


## 🛠 Technologies & Tools Used

### Hardware
- ESP32 Microcontroller
- ESP32-CAM
- Mobile Robot Platform
- Servo Motors (6-DOF Robotic Arm)
- Ultrasonic Sensors
- IR Sensors
- Load Cell Sensors

### Machine Learning & Computer Vision
- YOLOv8 (custom-trained)
- ultralytics
- opencv-python
- cvzone

### Software & Development
- Arduino IDE
- Flutter
- firebase_core
- firebase_messaging
- flutter_test


## 📦 Dependencies & Requirements

### 🔹 Python
- Python 3.8+
- ultralytics
- opencv-python
- cvzone

### 🔹 Mobile Application
- Flutter SDK
- firebase_core
- firebase_messaging
- flutter_test

### 🔹 Embedded Systems
- Arduino IDE
- ESP32 board packages
- ESP32-CAM libraries


## 🚀 Research Novelty & Contribution
- Time-aware garbage collection based on real crowd conditions
- YOLOv8-based intelligence for detection and classification
- Safe autonomous navigation in public environments
- Combined mobile and stationary robotic architecture
- Event-driven waste management approach

## 📊 Expected Outcomes
- Improved station cleanliness
- Reduced manual labor
- Increased operational safety
- Scalable solution for public environments

## 🔗 GitHub Repository
https://github.com/nimesh20010925/Railway-Waste-Management.git