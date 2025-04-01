import cv2
import time
import math
import serial
import threading
from ultralytics import YOLO

# Set up serial communication (change 'COM3' to match your Arduino port)
ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Allow time for serial connection to establish

# Load YOLO model
model = YOLO("yolov8n.pt")  # Ensure you have yolov8n.pt

# Camera Parameters
FOV = 60  # Camera Field of View in degrees
FRAME_WIDTH = 720  # Camera frame width in pixels
KNOWN_WIDTH = 0.075  # Actual width of the object (car) in meters
FOCAL_LENGTH = 440  # Calibrated focal length

# LED Setup
NUM_LEDS = 10  # Total LEDs (5 left + 5 right)
LED_SPACING = 0.015  # 1.5 cm = 0.015m between each LED
LED_CENTER = 5  # Middle LEDs (between 5th and 6th)

cap = cv2.VideoCapture(0)  # Open the camera
cap.set(3, FRAME_WIDTH)
cap.set(4, 640)

running = True  # Flag for thread execution
last_sent_time = 0  # Avoid sending redundant data
object_detected = False  # Track if any car is detected


# Thread to send data to Arduino
def send_data():
    global object_detected, last_sent_time
    while running:
        current_time = time.time()

        if object_detected:
            message = object_detected  # Example: "OFF:3,4,5"
        else:
            message = "OFF:none\n"  # No car detected → All LEDs ON

        if ser.is_open:
            try:
                if current_time - last_sent_time > 0.2:  # 50ms interval
                    ser.write(message.encode())
                    print(f"Sent to Arduino: {message.strip()}")
                    last_sent_time = current_time
            except:
                print("Serial communication error")

        time.sleep(0.05)  # Send data every 50ms


# Start the serial communication thread
thread = threading.Thread(target=send_data, daemon=True)
thread.start()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame")
        break

    results = model(frame)  # Run YOLO detection
    detected_distance = 0
    detected_angle = 0
    car_detected = False
    message_to_send = "OFF:none\n"  # Default to no LEDs OFF

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0])  # Get class index
            obj_name = model.names[cls]  # Get object name

            if obj_name == "car":  # Only process car detection
                x1, _, x2, _ = map(int, box.xyxy[0])  # Bounding box
                obj_width_px = x2 - x1  # Width of car in pixels

                # Calculate Distance
                if obj_width_px > 0:
                    detected_distance = (KNOWN_WIDTH * FOCAL_LENGTH) / obj_width_px
                else:
                    detected_distance = 0

                # Calculate Angle
                x_center = (x1 + x2) / 2
                X_mid = FRAME_WIDTH / 2
                detected_angle = ((x_center - X_mid) / X_mid) * (FOV / 2)

                # Calculate Lateral Displacement
                lateral_displacement = detected_distance * math.tan(math.radians(detected_angle))

                # Determine LED in front of the car
                led_index = round((lateral_displacement / LED_SPACING)) + LED_CENTER
                led_index = max(1, min(NUM_LEDS, led_index))  # Keep it within 1-10 range

                # Determine how many LEDs to turn off based on distance
                if detected_distance < 0.05:  # If car is too close, turn off 7 LEDs
                    led_range = 7
                elif detected_distance < 0.15:  # If car is near (0.05m to 0.15m), turn off 5 LEDs
                    led_range = 5
                else:  # If car is far (>0.15m), turn off only 2 LEDs
                    led_range = 2

                # Get the affected LED range
                left_led_index = max(1, led_index - (led_range // 2))
                right_led_index = min(NUM_LEDS, led_index + (led_range // 2))

                # Generate LED OFF message
                leds_to_turn_off = [i for i in range(left_led_index, right_led_index + 1)]
                message_to_send = f"OFF:{','.join(map(str, leds_to_turn_off))}\n"

                # Draw bounding box
                cv2.rectangle(frame, (x1, 50), (x2, 100), (0, 255, 0), 2)
                cv2.putText(frame, f"Dist: {detected_distance:.2f}m, Angle: {detected_angle:.2f}°",
                            (x1, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                car_detected = True

    # Send Data to Arduino
    object_detected = message_to_send if car_detected else "OFF:none\n"  # Update for serial thread
    print(
        f"Car Detected: {car_detected}, Distance: {detected_distance:.2f}m, Angle: {detected_angle:.2f}°, LEDs OFF: {object_detected.strip()}")

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
