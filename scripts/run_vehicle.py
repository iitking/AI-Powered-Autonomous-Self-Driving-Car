#!/usr/bin/env python3
"""
Physical Vehicle Runtime Loop for Raspberry Pi / NVIDIA Jetson.
Connects camera, runs perception + decision + PID controllers,
and streams actuation packets to the Arduino/ESP32 motor controller.

Usage:
    python3 scripts/run_vehicle.py [--port /dev/ttyUSB0] [--camera 0]
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.config import DEFAULT_CONFIG
from src.communication.serial_bridge import SerialBridge, VehicleActuationCommand
from src.control.speed_controller import SpeedController
from src.control.steering_controller import SteeringController
from src.decision.behavior_planner import BehaviorPlanner
from src.perception.lane_detector import LaneDetector
from src.perception.object_detector import ObjectDetector
from src.perception.traffic_light_detector import TrafficLightDetector, TrafficLightState

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def main():
    parser = argparse.ArgumentParser(description="Autonomous Physical Vehicle Runtime")
    parser.add_argument("--port", type=str, default=DEFAULT_CONFIG.serial.port, help="Serial port to MCU")
    parser.add_argument("--baud", type=int, default=DEFAULT_CONFIG.serial.baudrate, help="Serial baudrate")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index")
    parser.add_argument("--mock", action="store_true", help="Run with mock serial hardware")
    args = parser.parse_args()

    print("=" * 60)
    print("🚘 AUTONOMOUS VEHICLE PHYSICAL RUNTIME")
    print(f"Connecting to MCU at {args.port} @ {args.baud} baud...")
    print("=" * 60)

    serial_bridge = SerialBridge(port=args.port, baudrate=args.baud, mock=args.mock)
    if not serial_bridge.is_connected:
        print(f"[Warning] Could not connect to {args.port}. Operating in Mock mode.")

    lane_detector = LaneDetector(DEFAULT_CONFIG.camera)
    object_detector = ObjectDetector(config=DEFAULT_CONFIG.camera)
    behavior_planner = BehaviorPlanner(DEFAULT_CONFIG)
    steering_ctrl = SteeringController(DEFAULT_CONFIG.vehicle, DEFAULT_CONFIG.pid)
    speed_ctrl = SpeedController(DEFAULT_CONFIG.vehicle, DEFAULT_CONFIG.pid, DEFAULT_CONFIG.safety)

    cap = None
    if HAS_CV2:
        cap = cv2.VideoCapture(args.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_CONFIG.camera.image_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_CONFIG.camera.image_height)
        if not cap.isOpened():
            print(f"[Warning] Camera index {args.camera} failed to open. Falling back to synthetic frames.")
            cap = None

    print("[System] Autonomy pipeline initialized. Starting control loop (30 Hz)...")
    loop_dt = 1.0 / 30.0

    try:
        while True:
            start_t = time.time()

            # 1. Capture Camera Frame
            frame = None
            if cap is not None:
                ret, frame = cap.read()

            # 2. Perception
            lane_res = lane_detector.process_frame(frame)
            objects = object_detector.detect(frame)

            # 3. Decision
            decision = behavior_planner.plan(
                lane=lane_res,
                objects=objects,
                traffic_light=TrafficLightState.GREEN,
                min_sensor_distance_m=2.5,
            )

            # 4. Control
            steer_deg, _ = steering_ctrl.compute_steering(
                cross_track_error_m=lane_res.cross_track_error_m + decision.lateral_offset_bias_m,
                heading_error_rad=lane_res.heading_error_rad,
                current_speed_mps=1.0,
                dt=loop_dt,
            )

            throttle, brake, _ = speed_ctrl.update(
                target_speed_mps=decision.target_speed_mps,
                current_speed_mps=1.0,
                dt=loop_dt,
            )

            # 5. Transmit actuation packet to Microcontroller
            cmd = VehicleActuationCommand(
                steering_deg=steer_deg,
                throttle=throttle,
                brake=brake,
                emergency_stop=decision.emergency_brake,
            )
            serial_bridge.send_command(cmd)

            # Sleep remainder of loop
            elapsed = time.time() - start_t
            sleep_time = max(0.001, loop_dt - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[Shutdown] Stopping vehicle safely...")
        # Send zero-throttle brake command
        stop_cmd = VehicleActuationCommand(steering_deg=0.0, throttle=0.0, brake=1.0, emergency_stop=True)
        serial_bridge.send_command(stop_cmd)
        serial_bridge.close()
        if cap is not None:
            cap.release()
        print("[Shutdown] Motors cut. System safe.")


if __name__ == "__main__":
    main()
