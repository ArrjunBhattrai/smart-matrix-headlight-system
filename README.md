# 🚗 Smart Matrix Headlight System

An intelligent vehicle headlight system that dynamically dims specific LED zones to reduce glare for oncoming traffic, using real-time object detection (YOLO) and hardware actuation via Arduino.

---

## 🎯 Objective

Traditional high-beam headlights often blind oncoming drivers, especially on highways and curved roads. This project addresses that problem by using computer vision to detect oncoming vehicles and adjust LED zones of the headlight in real time to reduce glare while maintaining visibility.

---

## ⚙️ System Architecture

Camera Feed → YOLO Detection → Position Mapping → Serial Command → Arduino → LED Matrix

- **YOLOv5 (Python)** detects vehicles and their screen positions.
- Detected bounding boxes are mapped to headlight **LED matrix zones**.
- Commands are sent via **USB Serial** to **Arduino** to turn ON/OFF specific zones.
- **Arduino** controls a physical **LED matrix or strip** simulating the car's headlight.

---

## 🧰 Technologies Used

| Component        | Technology |
|------------------|------------|
| Object Detection | YOLOv5 / OpenCV |
| Microcontroller  | Arduino Uno / Mega |
| LED Control      | 8x8 LED Matrix / LED strips |
| Communication    | Serial over USB |
| Programming Lang | Python, C++ (Arduino) |

---

## 🧠 Key Features

- 🎥 Real-time vehicle detection using YOLO
- 💡 Dynamic LED zone dimming based on object position
- 🔁 Serial communication protocol between Python and Arduino
- ⚡ Low-power LED matrix simulation of headlight
- 📐 Mapping logic from screen space to headlight zones

---

## 🪛 Hardware Components

- Arduino Uno / Mega
- 8×8 LED Matrix (or separate LEDs/strips for zones)
- USB camera (or video feed)
- Power supply for LEDs
- Jumper wires, resistors, breadboard

---