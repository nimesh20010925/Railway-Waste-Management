from ultralytics import YOLO
import cv2
import requests
import time

# ================================
# CONFIG
# ================================
YOLO_MODEL_PATH = "/Users/pereraw.b.n/waste detection/best.pt"

ESP32_CAM_IP = "192.168.8.103"
ESP32_ROBOT_IP = "192.168.8.50"

STREAM_URL = f"http://{ESP32_CAM_IP}:81/stream"
GRAB_URL = f"http://{ESP32_ROBOT_IP}/grab"
RESET_URL = f"http://{ESP32_ROBOT_IP}/reset"

CONFIDENCE_THRESHOLD = 0.6
DETECTION_CONFIRM_FRAMES = 5   # frames required to confirm object
COOLDOWN_TIME = 6              # seconds after grab

# ================================
# LOAD MODEL
# ================================
print("Loading YOLO model...")
model = YOLO(YOLO_MODEL_PATH)

# ================================
# CONNECT TO CAMERA
# ================================
print("Connecting to ESP32-CAM...")
cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    print("❌ Failed to connect to ESP32-CAM")
    exit()

print("✅ Camera connected")

# ================================
# HELPER: SEND COMMAND
# ================================
def send_command(url):
    try:
        requests.get(url, timeout=1)
        print(f"➡️ Sent command: {url}")
    except requests.exceptions.RequestException as e:
        print(f"⚠️ Command failed: {e}")

# ================================
# MAIN LOOP
# ================================
detection_counter = 0
last_action_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Frame lost, reconnecting...")
        cap.release()
        time.sleep(1)
        cap = cv2.VideoCapture(STREAM_URL)
        continue

    # YOLO detection
    results = model.predict(
        source=frame,
        conf=CONFIDENCE_THRESHOLD,
        save=False,
        show=False
    )

    boxes = results[0].boxes
    annotated_frame = results[0].plot()

    # ================================
    # DETECTION LOGIC
    # ================================
    if boxes is not None and len(boxes) > 0:
        detection_counter += 1
        cv2.putText(
            annotated_frame,
            f"Detecting... {detection_counter}/{DETECTION_CONFIRM_FRAMES}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )
    else:
        detection_counter = 0

    # ================================
    # CONFIRMED OBJECT
    # ================================
    current_time = time.time()
    if (
        detection_counter >= DETECTION_CONFIRM_FRAMES
        and current_time - last_action_time > COOLDOWN_TIME
    ):
        print("✅ Waste detected → GRAB")
        send_command(GRAB_URL)
        last_action_time = current_time
        detection_counter = 0

    # ================================
    # DISPLAY
    # ================================
    cv2.imshow("ESP32-CAM Waste Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# ================================
# CLEANUP
# ================================
cap.release()
cv2.destroyAllWindows()
