# 🔌 Hardware Wiring Diagram & Pin Connections

This document details the complete electrical interconnections between the host computer (Jetson / Raspberry Pi), microcontroller (Arduino / ESP32), motor driver, steering servo, and sensor array.

---

## 1. System Interconnection Architecture

```text
 ┌──────────────────────────────────────────────────────────────┐
 │                      LiPo Battery (7.4V - 11.1V)             │
 └──────┬─────────────────────────────────────────────────┬─────┘
        │                                                 │
        ↓                                                 ↓
 ┌───────────────┐                                 ┌─────────────┐
 │ 5V/3A Buck    │                                 │ Motor Driver│
 │ Regulator     │                                 │ (TB6612FNG) │
 └──────┬────────┘                                 └──────┬──────┘
        ├──────────────────────┬─────────────┐            │
        ↓                      ↓             ↓            ↓
 ┌──────────────┐       ┌─────────────┐ ┌─────────┐ ┌────────────┐
 │ Raspberry Pi │<=====>│ Arduino/    │ │ Steering│ │ Drive DC   │
 │ / Jetson     │  USB  │ ESP32       │ │ Servo   │ │ Motors     │
 └──────┬───────┘ Serial└──────┬──────┘ └─────────┘ └────────────┘
        │                      │
        ↓                      ├──────────────► Ultrasonic HC-SR04
     Camera                    ├──────────────► Optical Encoders
                               └──────────────► MPU-6050 IMU (I2C)
```

---

## 2. Detailed Pinout Tables

### A. Arduino Uno R3 Connection Table

| Peripheral | Peripheral Pin | Arduino Pin | Signal Type | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Steering Servo** | Signal (White/Orange) | **Pin 9** | Hardware PWM (Timer 1) | Servo angle control |
| | VCC (Red) | +5V or +6V Buck | DC Power | Do NOT power directly from Arduino pin |
| | GND (Black/Brown) | Common GND | Ground | Shared ground |
| **Motor Driver (TB6612)** | PWM (PWMA) | **Pin 5** | PWM (0-255) | Speed throttle |
| | AIN1 | **Pin 7** | Digital Out | Direction forward |
| | AIN2 | **Pin 8** | Digital Out | Direction reverse |
| | STBY (Standby) | **Pin 6** | Digital Out | High = Active, Low = Coast |
| | VM (Motor Power) | Battery (+) | DC Power | 7.4V – 12V |
| | VCC (Logic Power) | +5V | Logic Power | From Arduino 5V pin |
| | GND | Common GND | Ground | Both motor and logic GND |
| **Ultrasonic (HC-SR04)** | VCC | +5V | DC Power | 5V supply |
| | GND | Common GND | Ground | |
| | Trig | **Pin 4** | Digital Out | 10µs trigger pulse |
| | Echo | **Pin 2** | Interrupt (INT0) | High pulse duration = distance |
| **Optical Wheel Encoder** | VCC | +5V | DC Power | |
| | GND | Common GND | Ground | |
| | Digital Out (DO) | **Pin 3** | Interrupt (INT1) | Pulses per rotation |
| **MPU-6050 IMU** | VCC | +5V / +3.3V | DC Power | |
| | GND | Common GND | Ground | |
| | SDA | **Pin A4** | I2C Data | Heading and acceleration |
| | SCL | **Pin A5** | I2C Clock | |
| **Host SBC (Pi/Jetson)**| USB Port | USB Port | USB Serial | Bidirectional telemetry @ 115200 baud |

---

### B. ESP32 DevKit V1 Connection Table

| Peripheral | Peripheral Pin | ESP32 GPIO | Signal / Mode |
| :--- | :--- | :--- | :--- |
| **Steering Servo** | Signal | **GPIO 18** | LEDC Channel 0 (50 Hz) |
| **Motor Driver** | PWMA (Speed) | **GPIO 19** | LEDC Channel 1 (20 kHz silent PWM) |
| | PWMB (Direction) | **GPIO 21** | LEDC Channel 2 (20 kHz silent PWM) |
| **Ultrasonic** | Trig | **GPIO 22** | GPIO Output |
| | Echo | **GPIO 23** | GPIO Input (Use voltage divider if 5V) |
| **Encoder** | Signal | **GPIO 34** | Input Only (Interrupt RISING) |
| **IMU (MPU-6050)** | SDA | **GPIO 21** / SDA | I2C Data |
| | SCL | **GPIO 22** / SCL | I2C Clock |

---

## 3. High-Current Steering Servo Brownout Protection

High torque servos (such as the MG996R) can draw instantaneous peak stall currents up to **1.5A–2.5A** during rapid steering maneuvers.
Connecting the servo's power wire directly to the 5V regulator of an Arduino or Raspberry Pi will cause voltage dips that reset the microcontroller.

**Correct Wiring Practice:**
1. Power the steering servo using an independent dedicated DC-DC buck converter adjusted to **6.0V**.
2. Connect all ground wires (Battery Ground, Buck Converter Ground, Arduino Ground, Raspberry Pi Ground) together to establish a single common ground plane.
3. Install a **1000 µF / 16V electrolytic capacitor** across the servo's VCC and GND terminals to buffer transient current spikes.
