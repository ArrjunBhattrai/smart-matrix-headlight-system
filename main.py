import cv2
import time
import serial
import threading
from ultralytics import YOLO

# Set up the serial connection (Change 'COM10' to your Arduino's port)
ser = serial.Serial('COM9', 9600, timeout=1)
time.sleep(2)  # Allow time for serial connection to establish

# Load YOLOv8 model
model = YOLO("yolov8n.pt")  # Ensure you have yolov8n.pt

# Define camera parameters
FOV = 60  # Camera's field of view in degrees
FRAME_WIDTH = 640  # Camera frame width
KNOWN_WIDTH = 0.075  # Real width of object in meters
FOCAL_LENGTH = 820  # Calibrated focal length (adjust if needed)

cap = cv2.VideoCapture(0)  # Open camera
cap.set(3, FRAME_WIDTH)
cap.set(4, 480)

running = True  # Flag for thread execution
last_sent_time = 0  # Avoid sending redundant data
object_detected = False  # Track if any car is detected


# Thread to send data to Arduino
def send_data():
    global object_detected, last_sent_time
    while running:
        current_time = time.time()

        if object_detected:
            message = object_detected  # Message is either "Ncar" or "Acar"
        else:
            message = "Nnone\n"  # No car detected → Both LEDs OFF

        if ser.is_open:
            try:
                if current_time - last_sent_time > 0.05:  # 50ms interval
                    ser.write(message.encode())
                    print(f"Sent to Arduino: {message.strip()}")
                    last_sent_time = current_time
            except:
                print("Serial communication error")

        time.sleep(0.05)  # Send data every 50ms


# Start serial communication thread
thread = threading.Thread(target=send_data, daemon=True)
thread.start()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame")
        break

    results = model(frame)  # Run YOLOv8
    detected_distance = 0
    detected_angle = 0
    car_detected = False
    message_to_send = "Nnone\n"  # Default to "Nnone" if no valid car detected

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0])  # Get class index
            obj_name = model.names[cls]  # Get object name

            if obj_name == "car":  # Only process car detection
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box
                obj_width_px = x2 - x1  # Width of detected car in pixels

                # Calculate Distance
                if obj_width_px > 0:
                    detected_distance = (KNOWN_WIDTH * FOCAL_LENGTH) / obj_width_px
                else:
                    detected_distance = 0

                # Calculate Angle
                x_center = (x1 + x2) / 2
                X_mid = FRAME_WIDTH / 2
                detected_angle = ((x_center - X_mid) / X_mid) * (FOV / 2)

                # Decide LED control based on Angle
                if -10 <= detected_angle <= 10:
                    message_to_send = "Ncar\n"  # LED1 ON (Pin 12)
                elif detected_angle < -10 or detected_angle > 10:
                    message_to_send = "Acar\n"  # LED2 ON (Pin 13)

                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Car: {detected_distance:.2f}m, {detected_angle:.2f}°",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255), 2)

                car_detected = True  # A car was detected

    # Send Data to Arduino
    object_detected = message_to_send if car_detected else "Nnone\n"  # Update for serial thread
    print(
        f"Detected Car: {car_detected}, Distance: {detected_distance:.2f}m, Angle: {detected_angle:.2f}°, Sent: {object_detected.strip()}")

    # Show camera output
    cv2.imshow("YOLOv8 Car Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
ser.close()
thread.join()