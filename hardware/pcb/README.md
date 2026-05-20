# 🖨️ Custom PCB Design & Expansion Shield

This folder contains guidelines and resources for fabricating a custom integration PCB / Hat for the autonomous vehicle prototype.

---

## 1. PCB Overview

The expansion board combines:
- Socket headers for Arduino Nano or ESP32 DevKit module
- Dual TB6612FNG motor driver socket
- Dedicated 3-pin headers for Steering Servo (Signal, 6V, GND)
- 4-pin headers for HC-SR04 Ultrasonic Sensors (Front, Left, Right)
- I2C header for MPU-6050 IMU (3.3V, GND, SDA, SCL)
- Screw terminals for 2S/3S LiPo battery input and DC motor leads
- Dual DC-DC step-down buck footprint (5V for logic, 6V for servo)
- Battery voltage divider resistors (10k / 4.7k) and power indicator LEDs

---

## 2. Recommended PCB Layout Rules

1. **Trace Width for Power:** Use minimum **40–60 mil (1.0–1.5mm)** copper trace width or dedicated polygon pour for motor and battery lines to carry up to 3A without resistive heating.
2. **Ground Plane:** Use a full continuous ground plane on the bottom layer.
3. **Decoupling Capacitors:** Place a **0.1 µF ceramic capacitor** directly adjacent to the VCC pin of every active IC, and a **470–1000 µF electrolytic capacitor** across motor driver power terminals.
