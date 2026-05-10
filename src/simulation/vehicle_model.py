"""
Kinematic Bicycle Model for Autonomous Vehicle Simulation.
Simulates Ackermann steering, throttle acceleration, aerodynamic drag,
rolling resistance, and steering servo delay.
"""

import math
from dataclasses import dataclass
from typing import Tuple
from src.config import VehicleGeometry
from src.utils.math_utils import clamp, normalize_angle


@dataclass
class VehicleKinematics:
    x: float = 0.0          # Global X coordinate (meters)
    y: float = 0.0          # Global Y coordinate (meters)
    yaw: float = 0.0        # Vehicle heading (radians, 0 = facing +X)
    velocity: float = 0.0   # Linear forward velocity (m/s)
    steer_angle: float = 0.0# Current front wheel angle (radians)
    accel: float = 0.0      # Current linear acceleration (m/s^2)


class KinematicBicycleModel:
    """
    Standard Ackermann kinematic bicycle model:
    dx/dt = v * cos(yaw)
    dy/dt = v * sin(yaw)
    dyaw/dt = (v / L) * tan(delta)
    dv/dt = a - drag
    """

    def __init__(self, geometry: VehicleGeometry):
        self.geom = geometry
        self.state = VehicleKinematics()

        # Physical coefficients
        self.drag_coeff = 0.15     # Aerodynamic drag factor
        self.rolling_friction = 0.10 # Rolling resistance
        self.steer_delay_rate = 6.0 # Servo responsiveness rad/s

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, v: float = 0.0):
        """Resets vehicle pose and dynamics."""
        self.state = VehicleKinematics(x=x, y=y, yaw=yaw, velocity=v, steer_angle=0.0, accel=0.0)

    def step(self, target_steer_deg: float, throttle: float, brake: float, dt: float) -> VehicleKinematics:
        """
        Advances vehicle state by dt seconds.
        target_steer_deg: commanded steering angle in degrees
        throttle: [0.0, 1.0] forward engine effort
        brake: [0.0, 1.0] braking effort
        """
        target_steer_rad = math.radians(clamp(target_steer_deg, -self.geom.max_steering_angle, self.geom.max_steering_angle))

        # Dynamic servo steering lag
        steer_diff = target_steer_rad - self.state.steer_angle
        max_steer_step = self.steer_delay_rate * dt
        self.state.steer_angle += clamp(steer_diff, -max_steer_step, max_steer_step)

        # Net acceleration calculation
        motor_thrust = throttle * self.geom.max_accel
        brake_decel = brake * self.geom.max_decel
        drag = self.drag_coeff * (self.state.velocity ** 2) + self.rolling_friction * (1.0 if self.state.velocity > 0.01 else 0.0)

        net_accel = motor_thrust - brake_decel - drag
        self.state.accel = net_accel

        # Velocity update
        new_v = self.state.velocity + net_accel * dt
        self.state.velocity = clamp(new_v, 0.0, self.geom.max_speed)

        # Kinematic bicycle pose integration
        v = self.state.velocity
        delta = self.state.steer_angle
        yaw = self.state.yaw
        L = self.geom.wheelbase

        # Yaw rate omega = (v / L) * tan(delta)
        yaw_rate = (v / L) * math.tan(delta)

        # Midpoint integration
        mid_yaw = yaw + (yaw_rate * dt) / 2.0
        self.state.x += v * math.cos(mid_yaw) * dt
        self.state.y += v * math.sin(mid_yaw) * dt
        self.state.yaw = normalize_angle(yaw + yaw_rate * dt)

        return self.state
