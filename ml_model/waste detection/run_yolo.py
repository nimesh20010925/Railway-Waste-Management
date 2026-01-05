from ultralytics import YOLO
import cv2
import numpy as np
import requests
from threading import Thread
import time

# -------------------------------
# Load your trained YOLOv8 model
# -------------------------------
model = YOLO("/Users/pereraw.b.n/waste detection/best.pt")

# -------------------------------
# ESP32-CAM Configuration
# -------------------------------
ESP32_CAM_IP = "192.168.8.103"
STREAM_URL = f"http://{ESP32_CAM_IP}:81/stream"
CAPTURE_URL = f"http://{ESP32_CAM_IP}/capture"

# Choose method 1 or 2 below:

# ====================================================================
# METHOD 1: Using OpenCV with MJPEG Stream (Recommended)
# ====================================================================
def method1_opencv_stream():
    """Connect directly to ESP32-CAM MJPEG stream"""
    stream_url = f"http://{ESP32_CAM_IP}:81/stream"
    
    print(f"Connecting to ESP32-CAM stream: {stream_url}")
    cap = cv2.VideoCapture(stream_url)
    
    if not cap.isOpened():
        print("Failed to connect to ESP32-CAM stream")
        return
    
    fps = 0
    frame_count = 0
    start_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Reconnecting...")
            cap.release()
            cap = cv2.VideoCapture(stream_url)
            time.sleep(1)
            continue
        
        frame_count += 1
        if frame_count % 30 == 0:
            fps = frame_count / (time.time() - start_time)
            print(f"FPS: {fps:.2f}")
        
        # Run YOLO detection
        results = model.predict(source=frame, save=False, show=False, conf=0.5)
        
        # Display results
        annotated_frame = results[0].plot()
        cv2.putText(annotated_frame, f"ESP32-CAM | FPS: {fps:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("ESP32-CAM Waste Detection", annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

# ====================================================================
# METHOD 2: HTTP Requests for Single Frames (More Reliable)
# ====================================================================
def method2_http_capture():
    """Fetch single images via HTTP capture endpoint"""
    print(f"Connecting to ESP32-CAM: {ESP32_CAM_IP}")
    
    while True:
        try:
            # Fetch image from ESP32-CAM
            response = requests.get(CAPTURE_URL, timeout=2)
            img_array = np.array(bytearray(response.content), dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            if frame is None:
                print("Failed to decode image")
                time.sleep(0.1)
                continue
            
            # Run YOLO detection
            results = model.predict(source=frame, save=False, show=False, conf=0.5)
            
            # Display results
            annotated_frame = results[0].plot()
            
            # Add statistics
            detections = len(results[0].boxes)
            cv2.putText(annotated_frame, f"Detections: {detections}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow("ESP32-CAM Waste Detection", annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        except requests.exceptions.RequestException as e:
            print(f"Connection error: {e}. Retrying...")
            time.sleep(1)
    
    cv2.destroyAllWindows()

# ====================================================================
# METHOD 3: Fastest - Using threading for requests
# ====================================================================
class ESP32CamStream:
    def __init__(self, ip):
        self.ip = ip
        self.frame = None
        self.stopped = False
        self.capture_url = f"http://{ip}/capture"
    
    def start(self):
        Thread(target=self.update, args=()).start()
        return self
    
    def update(self):
        while not self.stopped:
            try:
                response = requests.get(self.capture_url, timeout=1)
                img_array = np.array(bytearray(response.content), dtype=np.uint8)
                self.frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            except:
                self.frame = None
                time.sleep(0.1)
    
    def read(self):
        return self.frame
    
    def stop(self):
        self.stopped = True

def method3_threaded_stream():
    """Threaded approach for faster frame rate"""
    print("Starting threaded ESP32-CAM stream...")
    
    stream = ESP32CamStream(ESP32_CAM_IP).start()
    time.sleep(1)  # Allow camera to initialize
    
    while True:
        frame = stream.read()
        if frame is None:
            continue
        
        # Run YOLO detection
        results = model.predict(source=frame, save=False, show=False, conf=0.5)
        
        # Display
        annotated_frame = results[0].plot()
        cv2.imshow("ESP32-CAM Threaded", annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    stream.stop()
    cv2.destroyAllWindows()

# ====================================================================
# METHOD 4: Web Dashboard with Flask (Most Professional)
# ====================================================================
from flask import Flask, Response, render_template

app = Flask(__name__)

def generate_frames():
    """Video streaming generator function."""
    stream_url = f"http://{ESP32_CAM_IP}:81/stream"
    cap = cv2.VideoCapture(stream_url)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Run detection
        results = model.predict(source=frame, save=False, show=False)
        annotated_frame = results[0].plot()
        
        # Encode frame
        ret, buffer = cv2.imencode('.jpg', annotated_frame)
        frame = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def method4_flask_dashboard():
    """Start Flask web server with detection"""
    print("Starting Flask server at http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

# ====================================================================
# MAIN EXECUTION
# ====================================================================
if __name__ == "__main__":
    print("ESP32-CAM Waste Detection System")
    print(f"Camera IP: {ESP32_CAM_IP}")
    print("\nChoose method:")
    print("1. OpenCV Stream (fastest)")
    print("2. HTTP Capture (reliable)")
    print("3. Threaded HTTP (balanced)")
    print("4. Flask Web Dashboard (browser)")
    
    choice = input("\nEnter choice (1-4): ").strip()
    
    if choice == "1":
        method1_opencv_stream()
    elif choice == "2":
        method2_http_capture()
    elif choice == "3":
        method3_threaded_stream()
    elif choice == "4":
        method4_flask_dashboard()
    else:
        print("Defaulting to Method 1...")
        method1_opencv_stream()