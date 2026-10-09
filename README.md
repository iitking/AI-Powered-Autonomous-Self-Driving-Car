# 🚗 AI-Powered Autonomous Self-Driving Car

[![Python Version](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Tests Passing](https://img.shields.io/badge/tests-29%2F29%20passing-brightgreen.svg)]()
[![Platform: Web Cockpit](https://img.shields.io/badge/Dashboard-Web%20Cockpit%2020Hz-orange.svg)]()

A software-and-hardware-in-the-loop (SIL/HIL) autonomous self-driving car system featuring real-time computer vision perception, behavioral decision state machines, Pure Pursuit + PID trajectory tracking, Extended Kalman Filter sensor fusion, proactive obstacle overtaking, dynamic lead vehicle Adaptive Cruise Control (ACC), pedestrian crosswalk yielding, and an interactive 2D web cockpit dashboard.

---

## 🌟 Key Capabilities

- **Lookahead Pure Pursuit & PID Lateral Steering:**
  - Blends geometric Pure Pursuit curve tracking with closed-loop PID cross-track error feedback.
  - Slew-rate-limited front wheel steering to prevent physical servo jerk.
  - Completed multi-lap tracking test with a maximum cross-track error of only **3.6 cm**.

- **Proactive Obstacle Avoidance & Corridor Overtaking:**
  - Scans up to 2.5 meters ahead for static obstacles, road cones, and stalled vehicles.
  - Dynamically shifts into a $\pm 0.32\text{ m}$ bypass corridor and latches evasion trajectory to prevent limit-cycle chattering.
  - Overtakes smoothly at cruising speed ($0.8\text{--}1.1\text{ m/s}$) without freezing or unnecessary emergency braking.

- **Dynamic Lead Traffic & Adaptive Cruise Control (ACC):**
  - Moving NPC traffic vehicles driving along the track.
  - Ego vehicle maintains safe time headway and speed-matches the lead vehicle ($0.85\text{ m/s}$).
  - Automatically identifies stalled lead vehicles and dynamically overtakes them.

- **Pedestrian Crosswalk Yielding:**
  - Detects active pedestrians crossing designated zebra crosswalks.
  - Smoothly decelerates and yields before the stop line, automatically resuming once the crosswalk is clear.

- **Computer Vision & Sensor Fusion:**
  - Perspective-transformed lane detection with polynomial curvature estimation.
  - HSV color-space traffic light signal classification (Red, Yellow, Green).
  - YOLO bounding box object projection.
  - Extended Kalman Filter (EKF) combining wheel encoders and 6-axis IMU yaw rate.

- **Interactive Real-Time Web Cockpit (20 Hz):**
  - Zero-external-dependency HTML5 Canvas web cockpit.
  - Birds-eye 2D track visualizer with real-time 16-beam LiDAR raycasting.
  - First-person simulated windshield camera view with HUD overlays.
  - Live controls to cycle traffic lights, drop obstacles, spawn lead traffic, stall traffic, and trigger pedestrian crossings.

---

## 🏗️ System Architecture

```
                  ┌───────────────────────────────┐
                  │    Sensors & Input Stream     │
                  │ Camera • Ultrasonic • IMU/Enc │
                  └───────────────┬───────────────┘
                                  │
                                  ▼
                  ┌───────────────────────────────┐
                  │      Perception Pipeline      │
                  │ Lane Detector • Traffic Light │
                  │     YOLO Object Detection     │
                  └───────────────┬───────────────┘
                                  │
                                  ▼
                  ┌───────────────────────────────┐
                  │   Behavior Decision Planner   │
                  │ State Machine • Corridor Latch│
                  │  ACC Following • Ped Yielding │
                  └───────────────┬───────────────┘
                                  │
                                  ▼
                  ┌───────────────────────────────┐
                  │    Motion Control & Actuation │
                  │ Pure Pursuit + PID Steering   │
                  │     Speed PID Controller      │
                  └───────────────┬───────────────┘
                                  │
                                  ▼
                  ┌───────────────────────────────┐
                  │  Physical Model / Hardware    │
                  │ Kinematic Bicycle Model / ESP32│
                  └───────────────────────────────┘
```

---

## 🚀 Quick Start

### 1. Prerequisites & Setup
Clone the repository and install required packages:
```bash
git clone https://github.com/niveshkr149/autonomous-self-driving-car.git
cd autonomous-self-driving-car

# Optional virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

*(Note: The core simulation and web cockpit run with zero external dependencies using Python's standard library!)*

### 2. Launch the Web Cockpit Simulator
```bash
python3 scripts/run_simulation.py
```
Open **[http://localhost:8080](http://localhost:8080)** in your browser to access the cockpit.

---

## 🎮 Interactive Controls in Dashboard

| Control Button | Action |
| :--- | :--- |
| **`Switch to Manual`** | Take manual keyboard control (`W/S` for Throttle/Brake, `A/D` for Steer). |
| **`🚦 Cycle Light`** | Cycle traffic signal between Green, Yellow, and Red. |
| **`🚧 Drop Obstacle`** | Drop a stationary obstacle directly ahead to watch live evasive overtaking. |
| **`🚗 Lead Traffic Car`** | Toggle moving NPC lead vehicle for Adaptive Cruise Control following. |
| **`🛑 Stall Traffic`** | Stall the lead traffic vehicle so ego car overtakes it from the side. |
| **`🚶 Pedestrian Cross`**| Trigger a pedestrian walking across the zebra crossing. |
| **`Clear All`** | Clear all obstacles, traffic cars, and active pedestrians. |
| **`🔄 Reset`** | Reset vehicle to starting line. |

---

## 🧪 Testing & Verification

Run the full automated unit test suite (29 tests covering kinematics, PID tuning, perception, obstacle evasion, ACC traffic, and pedestrian yielding):
```bash
python3 -m unittest discover tests
```
Output:
```
.............................
Ran 29 tests in 0.063s - OK
```

To run performance benchmarks:
```bash
python3 scripts/benchmark_performance.py
```

---

## 📂 Repository Structure

```
├── .gitignore               # Ignored build and environment files
├── README.md                # Project documentation and guide
├── pyproject.toml           # Project packaging and metadata
├── requirements.txt         # Python package dependencies
├── datasets/                # Sample track dataset and annotations
├── embedded/                # Low-level micro-controller firmware
│   ├── arduino/             # Arduino motor and servo PWM driver
│   └── esp32/               # ESP32 high-speed telemetry bridge
├── hardware/                # Schematics, PCB layouts & BOM list
├── notebooks/               # Jupyter research & analysis notebooks
├── scripts/                 # Simulation and benchmark runners
├── src/                     # Core autonomy stack
│   ├── communication/       # Serial protocol & hardware bridge
│   ├── config.py            # Global vehicle geometry & tuning configs
│   ├── control/             # Steering & longitudinal PID controllers
│   ├── dashboard/           # Web cockpit server & interactive frontend
│   ├── decision/            # State machine & behavior planner
│   ├── navigation/          # Localizer, path planner & avoidance
│   ├── perception/          # Lane, traffic light & YOLO detectors
│   ├── sensors/             # Sensor fusion (EKF), ultrasonic, IMU
│   ├── simulation/          # Kinematic bicycle model & 2D world
│   └── utils/               # Math, angle normalization & transforms
└── tests/                   # 29 automated unit tests
```

---

## 👨‍💻 Author

<div align="center">

<a href="https://github.com/iitking">
  <img src="https://github.com/iitking.png" width="110" height="110" style="border-radius:50%" alt="Nivesh Kumar Meena" />
</a>

### **Nivesh Kumar Meena**

**AI Architect · MLOps Engineer** | B.Tech Electrical Engineering, **IIT Roorkee**

*Building agentic AI systems, RAG pipelines and production-ready ML.*

<a href="https://www.linkedin.com/in/nivesh-kumar-meena-a31465221/"><img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white" alt="LinkedIn" /></a>
<a href="https://github.com/iitking"><img src="https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white" alt="GitHub" /></a>
<a href="mailto:niveshkr149@gmail.com"><img src="https://img.shields.io/badge/Email-D14836?style=for-the-badge&logo=gmail&logoColor=white" alt="Email" /></a>

<br/><br/>

⭐ **If you found this project useful, please give it a star!** ⭐

<sub>Open to AI/ML engineering opportunities and collaborations.</sub>

</div>

---

<div align="center">
  <sub>Made with ❤️ by <a href="https://github.com/iitking">Nivesh Kumar Meena</a> · © 2026 · MIT License</sub>
</div>
