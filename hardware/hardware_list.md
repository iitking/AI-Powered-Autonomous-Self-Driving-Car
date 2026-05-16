# 🛠️ Autonomous Self-Driving Car - Bill of Materials (BOM)

This document specifies the complete hardware components, technical specifications, and recommended parts for constructing the physical 1/10th or 1/16th scale autonomous prototype.

---

## 1. Primary Computing & AI Processing Unit

| Component | Model Recommended | Primary Role | Interface | Operating Voltage |
| :--- | :--- | :--- | :--- | :--- |
| **Edge AI Computer (Option A)** | **NVIDIA Jetson Orin Nano / Nano 4GB** | YOLO Object Detection, CNN models, ROS 2 | CSI-2, USB 3.0, UART | 5V / 9-19V DC |
| **Single Board Computer (Option B)** | **Raspberry Pi 5 (4GB / 8GB)** | Computer Vision, OpenCV, PID Decision Stack | CSI, USB 3.0, UART | 5V / 5A (USB-PD) |

---

## 2. Low-Level Embedded Controller

| Component | Model Recommended | Function | Pinout / Ports |
| :--- | :--- | :--- | :--- |
| **Microcontroller** | **Arduino Uno R3 / Mega 2560** OR **ESP32 DevKit V1** | Hardware PWM generation, encoder ISRs, ultrasonic timing, safety watchdog | USB Serial / UART, GPIOs |

---

## 3. Sensors & Perception Devices

| Sensor | Recommended Part | Purpose | Specs / Range |
| :--- | :--- | :--- | :--- |
| **Wide-Angle Camera** | Sony IMX219 (Raspberry Pi Cam v2 / Jetson Cam) OR 1080p USB Webcam (Logitech C920) | Lane detection, traffic lights, signs, YOLO object detection | 1080p @ 30fps, 120°–160° FoV |
| **Ultrasonic Sensors (x3)** | HC-SR04 / HC-SR04P (3.3V compatible) | Short-range collision avoidance (Front, Left, Right) | 2 cm – 400 cm range, 15° cone |
| **IMU (Inertial Unit)** | MPU-6050 (6-DOF) or BNO055 (9-DOF Absolute Orientation) | Heading angle (yaw), roll/pitch stability, acceleration | I2C (Address 0x68), 100 Hz |
| **Wheel Encoders (x2)** | Optical IR Speed Sensor (LM393 with 20-slot disc) or Hall Effect | Odometry, speed measurement, dead-reckoning | 20 pulses/rev, digital pulse |
| **2D LiDAR (Optional)** | RPLIDAR A1M8 / LD19 | 360° laser scanning, SLAM mapping | 12m range, 5.5–10 Hz scan rate, UART |

---

## 4. Actuation & Drive Mechanism

| Component | Model / Specs | Purpose | Quantity |
| :--- | :--- | :--- | :--- |
| **Chassis** | 1:10 or 1:16 scale RC Car Chassis with Ackermann steering | Physical frame, suspension, steering linkage | 1 |
| **Steering Servo** | MG996R / MG90S High-Torque Metal Gear Servo | Front wheel steering angle (-30° to +30°) | 1 |
| **Drive Motors** | High-Torque 6V–12V DC Gearmotors (e.g. TT motor or 370 motor with gearbox) | Rear-wheel drive propulsion | 1 or 2 |
| **Motor Driver** | TB6612FNG (Dual MOSFET, high efficiency) or L298N Dual H-Bridge | Motor speed (PWM) and directional control (H-Bridge) | 1 |

---

## 5. Power Distribution & Regulation

| Component | Rating / Specs | Purpose |
| :--- | :--- | :--- |
| **Main Battery** | 2S (7.4V) or 3S (11.1V) LiPo Battery 2200mAh – 3000mAh 25C | System power source |
| **Logic Step-Down Converter** | LM2596 / XL4015 DC-DC Buck Converter (Set to 5.1V, 3A–5A) | Clean isolated power for Jetson / Raspberry Pi & Arduino |
| **Servo Step-Down (Optional)**| LM2596 DC-DC Buck Converter (Set to 6.0V, 3A) | High current power for steering servo (prevents brownouts) |
| **Power Switch & Fuse** | Heavy-duty toggle switch + 10A inline fuse | Master emergency power cutoff |
| **Logic Level Converter** | 4-channel 5V <-> 3.3V Bi-directional converter | Arduino (5V) to Raspberry Pi / ESP32 (3.3V) protection |

---

## 6. Miscellaneous Hardware & Cables

- Solid-core jumper wires & Dupont connectors (Male-to-Male, Male-to-Female)
- Micro-USB / USB-C to USB-A data communication cable
- M3 brass standoffs, screws, and nylon nuts
- Breadboard or custom soldered protoboard
- 3D-printed camera mount (pitched downwards by ~15°)
