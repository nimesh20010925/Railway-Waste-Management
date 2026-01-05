# Railway-Waste-Management

## 📌 Project Overview
Railway stations experience large passenger volumes, especially during peak hours, making manual waste collection unsafe, inefficient, and labor-intensive. This research proposes an **intelligent, time-aware, multi-robot waste management system** designed to automate garbage collection, classification, and monitoring while minimizing human involvement.

The system integrates **CCTV-based crowd analysis**, a **mobile garbage-collecting robot with intelligent navigation**, a **stationary robotic waste classification arm**, and a **mobile notification application**. By avoiding peak crowd periods and automating waste handling, the solution improves hygiene, safety, and operational efficiency in railway station environments.

## 🎯 Research Objectives
- Identify crowded and non-crowded time periods using CCTV-based headcount analysis  
- Schedule garbage collection during less crowded periods to ensure safety  
- Enable autonomous garbage collection using a mobile robot  
- Detect and classify garbage using vision-based machine learning models (YOLOv8)  
- Monitor categorized bin fill levels in real time  
- Notify cleaning staff through a mobile application when bins are full  


## 🔁 End-to-End System Workflow
1. **Crowd Detection Phase**  
   CCTV camera feeds are analyzed to estimate passenger headcount. Crowded time windows are identified and excluded from robot operation.

2. **Collection Scheduling Phase**  
   Garbage collection tasks are scheduled during less crowded periods to avoid interference with passengers.

3. **Mobile Robot Navigation & Collection Phase**  
   The mobile robot navigates station environments using obstacle avoidance and efficient navigation logic. A custom-trained **YOLOv8 model** assists in identifying garbage items to be collected. Detected garbage is collected and stored in the robot’s internal bin.

4. **Garbage Transfer Phase**  
   When the internal bin reaches a threshold, the mobile robot moves to the stationary sorting station and unloads garbage beneath the robotic arm.

5. **Waste Identification & Classification Phase**  
   The stationary robotic arm captures images of individual garbage items using an ESP32-CAM. A **YOLOv8-based classification model** categorizes waste into:
   - Plastic  
   - Paper  
   - Glass  
   - Unidentified waste  

   The arm places items into **four corresponding disposal bins**.

6. **Bin Monitoring Phase**  
   Sensors continuously monitor fill levels of each disposal bin.

7. **Notification Phase**  
   When any bin reaches its threshold, a real-time notification is sent to cleaning staff via the mobile application.

## 🦾 System Components
### 1️⃣ Crowd Analysis Module
- Utilizes existing CCTV infrastructure  
- Estimates passenger density  
- Determines safe operational time windows  

### 2️⃣ Mobile Garbage-Collecting Robot
- Operates during low-crowd periods only  
- Uses obstacle avoidance and efficient navigation  
- YOLOv8-assisted garbage detection  
- Internal bin with fill-level monitoring  

### 3️⃣ Stationary Robotic Sorting Arm
- Fixed installation for safety and reliability  
- 6-DOF servo-based robotic arm  
- Vision-assisted garbage classification using YOLOv8  
- Four-bin sorting mechanism  

### 4️⃣ Smart Disposal Bins
- Separate bins for each waste category  
- Equipped with ultrasonic sensors  
- Prevent delayed collection and hygiene issues  

### 5️⃣ Mobile Notification Application
- Sends real-time bin-full alerts    
- Enables faster response from cleaning staff  


## 🛠 Technologies & Tools Used
### Hardware
- ESP32 Microcontroller  
- ESP32-CAM Module  
- Mobile Robot Platform  
- Servo Motors (6-DOF Robotic Arm)  
- Ultrasonic Sensors  
- IR Sensors  
- Load Cell Sensors  

### Machine Learning & Computer Vision
- **YOLOv8 (custom-trained)** for garbage detection and classification  
- Railway-station-like image dataset  
- Supervised training with bounding-box annotations  

### Robotics & Navigation
- Obstacle avoidance using sensor fusion  
- Efficient navigation logic for mobile robot movement  
- Safe operation strategies for public environments  

### Software & Development
- Arduino IDE (ESP32 firmware)  
- Flutter (Mobile Application)  
- Firebase / Cloud REST services    

## 🌍 Application Domain
- Smart Transportation Systems  
- Smart Cities  
- Public Infrastructure Automation  
- Environmental Monitoring  

## ♻️ Sustainable Development Goals (SDGs)
- **SDG 11:** Sustainable Cities and Communities  
- **SDG 12:** Responsible Consumption and Production  

## 🚀 Research Novelty & Contribution
- Time-aware garbage collection based on real crowd conditions  
- Integration of **YOLOv8-based vision intelligence** for both collection and classification  
- Safe and efficient mobile robot navigation in public spaces  
- Combined mobile collection and stationary robotic sorting architecture  
- Event-driven notification system replacing fixed schedules  
- Modular and scalable research framework  

## 📊 Expected Outcomes
- Improved cleanliness in railway stations  
- Reduced manual labor and operational delays  
- Increased safety during peak passenger hours  
- Scalable solution for other public environments  

## 🔮 Future Enhancements
- Advanced route planning algorithms  
- Predictive waste generation analytics  
- Multi-station centralized monitoring  
- Energy-efficient and solar-powered deployment  

## 📜 Project Status
- ✔ Crowd analysis logic designed  
- ✔ Mobile robot navigation workflow defined  
- ✔ YOLOv8-based detection and classification planned  
- ✔ Robotic arm mechanism implemented  
- ✔ Mobile app UI and notification flow designed  



