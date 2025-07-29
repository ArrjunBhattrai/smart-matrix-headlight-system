import cv2
import time
import serial
import threading
from ultralytics import YOLO

#Serial communication setup
try:
    ser = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)
    print(f" Serial connected: {ser.is_open}")
except Exception as e:
    print(f" Serial connection failed: {e}")
    exit()

# Load YOLO model
model = YOLO("yolov8n.pt")

# Camera settings
FOV = 60
FRAME_WIDTH = 720
KNOWN_WIDTH = 0.075  #meters
FOCAL_LENGTH = 440  #Focal length calibrated using FOCAL_LENGTH = (obj_width_px_at_known_distance * known_distance) / known_width

# Global variables for threading
letter_to_send = "X"
last_sent_time = 0
running = True

# Open video feed
cap = cv2.VideoCapture(0)
cap.set(3, FRAME_WIDTH)
cap.set(4, 640)

if not cap.isOpened():
    print("Camera not accessible")
    exit()
else:
    print("Camera feed started")

# Function to determine which letter to send
def get_letter_to_send(distance, angle):
    distance_cm = distance * 100

    if distance_cm > 20:
        return 'X'

    if distance_cm <= 10:
        if -5 <= angle <= 5:
            return 'A'
        elif -10 <= angle < -5:
            return 'B'
        elif angle < -10:
            return 'C'
        elif 5 < angle <= 10:
            return 'D'
        elif angle > 10:
            return 'E'

    elif 10 < distance_cm <= 15:
        if -5 <= angle <= 5:
            return 'F'
        elif -10 <= angle < -5:
            return 'G'
        elif angle < -10:
            return 'H'
        elif 5 < angle <= 10:
            return 'I'
        elif angle > 10:
            return 'J'

    elif 15 < distance_cm <= 20:
        if -5 <= angle <= 5:
            return 'K'
        elif -10 <= angle < -5:
            return 'L'
        elif angle < -10:
            return 'M'
        elif 5 < angle <= 10:
            return 'N'
        elif angle > 10:
            return 'O'

    return 'X'

# Background thread for sending data periodically
def send_data():
    global letter_to_send, last_sent_time
    while running:
        current_time = time.time()
        if ser.is_open and (current_time - last_sent_time > 0.2):
            try:
                ser.write(letter_to_send.encode())
                print(f"[Thread] Sent to Arduino: {letter_to_send}")
                last_sent_time = current_time
            except Exception as e:
                print(f"[Thread] Serial communication error: {e}")
        time.sleep(0.05)

# Start thread
thread = threading.Thread(target=send_data, daemon=True)
thread.start()
print(f"Thread started: {thread.is_alive()}")

# Main loop
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    results = model(frame)
    temp_letter = "X"

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0])
            obj_name = model.names[cls]

            if obj_name == "car":
                x1, _, x2, _ = map(int, box.xyxy[0])
                obj_width_px = x2 - x1

                if obj_width_px > 0:
                    distance = (KNOWN_WIDTH * FOCAL_LENGTH) / obj_width_px
                else:
                    distance = 0

                x_center = (x1 + x2) / 2
                X_mid = FRAME_WIDTH / 2
                angle = ((x_center - X_mid) / X_mid) * (FOV / 2)

                temp_letter = get_letter_to_send(distance, angle)


                # Draw on frame
                cv2.rectangle(frame, (x1, 50), (x2, 100), (0, 255, 0), 2)
                cv2.putText(frame, f"{temp_letter} | D: {distance*100:.1f}cm, A: {angle:.1f}°",
                            (x1, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # Update global letter
    letter_to_send = temp_letter

    cv2.imshow("Car Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
ser.close()
thread.join()
print("Clean exit. Program terminated.")
