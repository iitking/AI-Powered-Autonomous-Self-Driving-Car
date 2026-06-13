"""
Longitudinal Speed and Throttle/Braking Controller.
Implements curvature speed limiting, Adaptive Cruise Control (ACC) headway,
and closed-loop PID velocity tracking.
"""

import math
from typing import Optional, Tuple
from src.config import PIDGains, VehicleGeometry, SafetyConfig
from src.control.pid_controller import PIDController, PIDTelemetry
from src.utils.math_utils import clamp


class SpeedController:
    """
    Computes motor throttle and braking commands from target velocity,
    road curvature, and obstacle distances.
    """

    def __init__(
        self,
        vehicle_geom: Optional[VehicleGeometry] = None,
        pid_gains: Optional[PIDGains] = None,
        safety_config: Optional[SafetyConfig] = None,
    ):
        self.geom = vehicle_geom or VehicleGeometry()
        self.gains = pid_gains or PIDGains()
        self.safety = safety_config or SafetyConfig()

        # Speed PID controller (Error = target_speed - current_speed)
        # Output range: [-1.0, 1.0] where > 0 is throttle, < 0 is brake
        self.pid = PIDController(
            kp=self.gains.kp_speed,
            ki=self.gains.ki_speed,
            kd=self.gains.kd_speed,
            output_limits=(-1.0, 1.0),
            integral_limit=self.gains.speed_anti_windup,
        )

        self.current_throttle = 0.0
        self.current_brake = 0.0

    def compute_safe_speed(
        self,
        nominal_speed_mps: float,
        road_curvature_radius_m: float,
        closest_obstacle_dist_m: float,
    ) -> float:
        """
        Determines the safe ceiling speed:
        1. Curvature speed limit: v <= sqrt(mu * g * R)
        2. Obstacle distance headway: slow down proportionally as obstacle approaches
        """
        # 1. Curvature limit (max lateral acceleration = 0.4g)
        max_lat_accel = 0.40 * 9.81
        if road_curvature_radius_m < 100.0 and road_curvature_radius_m > 0.05:
            curve_speed_limit = math.sqrt(max_lat_accel * road_curvature_radius_m)
        else:
            curve_speed_limit = self.geom.max_speed

        speed = min(nominal_speed_mps, curve_speed_limit, self.geom.max_speed)

        # 2. Obstacle distance adaptive deceleration
        if closest_obstacle_dist_m <= self.safety.critical_stop_distance:
            return 0.0
        elif closest_obstacle_dist_m < self.safety.slowdown_distance:
            # Linear ramp between critical_stop and slowdown
            fraction = (closest_obstacle_dist_m - self.safety.critical_stop_distance) / (
                self.safety.slowdown_distance - self.safety.critical_stop_distance
            )
            speed = min(speed, self.geom.min_speed + fraction * (self.geom.max_speed - self.geom.min_speed))

        return max(0.0, speed)

    def update(
        self,
        target_speed_mps: float,
        current_speed_mps: float,
        dt: float,
    ) -> Tuple[float, float, PIDTelemetry]:
        """
        Returns (throttle [0.0, 1.0], brake [0.0, 1.0], telemetry).
        """
        if target_speed_mps <= 0.01 and current_speed_mps < 0.05:
            # Vehicle stopped
            self.pid.reset()
            return 0.0, 1.0, PIDTelemetry(target=0.0, current=current_speed_mps)

        effort = self.pid.update(target=target_speed_mps, current=current_speed_mps, dt=dt)

        if effort >= 0.0:
            throttle = effort
            brake = 0.0
        else:
            throttle = 0.0
            brake = -effort

        self.current_throttle = throttle
        self.current_brake = brake
        return throttle, brake, self.pid.last_telemetry

    def reset(self):
        self.pid.reset()
        self.current_throttle = 0.0
        self.current_brake = 0.0
