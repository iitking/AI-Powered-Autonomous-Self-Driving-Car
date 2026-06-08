"""
Sensor Fusion using Extended Kalman Filter (EKF).
Fuses wheel odometry, IMU gyro angular rate, and accelerometer data
to generate optimal, noise-filtered 2D vehicle state estimates:
[pos_x, pos_y, heading_yaw, velocity].
"""

import math
from dataclasses import dataclass
from typing import Tuple
from src.utils.math_utils import normalize_angle


@dataclass
class FusedState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0


class SensorFusionEKF:
    """
    4-State Extended Kalman Filter:
    State: x = [pos_x, pos_y, yaw, v]^T
    Inputs: u = [accel_x, gyro_yaw_rate]^T
    Measurements: z = [encoder_v, imu_yaw]^T
    """

    def __init__(self, initial_x: float = 0.0, initial_y: float = 0.0, initial_yaw: float = 0.0):
        # State vector
        self.x = initial_x
        self.y = initial_y
        self.yaw = initial_yaw
        self.v = 0.0

        # State covariance matrix P (4x4 diagonal elements)
        self.p_pos = 0.1
        self.p_yaw = 0.05
        self.p_v = 0.1

        # Process noise variance Q
        self.q_pos = 0.01
        self.q_yaw = 0.005
        self.q_v = 0.05

        # Measurement noise variance R
        self.r_enc_v = 0.04
        self.r_imu_yaw = 0.02

    def predict(self, accel_x: float, gyro_gz: float, dt: float):
        """
        EKF Prediction step using kinematic motion model:
        x_{k+1} = x_k + v_k * cos(yaw_k) * dt
        y_{k+1} = y_k + v_k * sin(yaw_k) * dt
        yaw_{k+1} = yaw_k + gyro_gz * dt
        v_{k+1} = v_k + accel_x * dt
        """
        # State propagation
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = normalize_angle(self.yaw + gyro_gz * dt)
        self.v = max(0.0, self.v + accel_x * dt)

        # Covariance propagation (simplified diagonal approximation for embedded speed)
        self.p_pos += (self.v * dt) ** 2 * self.p_yaw + self.q_pos * dt
        self.p_yaw += self.q_yaw * dt
        self.p_v += self.q_v * dt

    def update_encoder(self, encoder_speed_mps: float):
        """Measurement update with wheel encoder speed."""
        # Kalman gain for velocity
        k_v = self.p_v / (self.p_v + self.r_enc_v)
        v_residual = encoder_speed_mps - self.v
        self.v += k_v * v_residual
        self.p_v *= (1.0 - k_v)

    def update_imu_heading(self, measured_yaw_rad: float):
        """Measurement update with absolute heading (e.g. from magnetometer or landmark)."""
        k_yaw = self.p_yaw / (self.p_yaw + self.r_imu_yaw)
        yaw_residual = normalize_angle(measured_yaw_rad - self.yaw)
        self.yaw = normalize_angle(self.yaw + k_yaw * yaw_residual)
        self.p_yaw *= (1.0 - k_yaw)

    def get_state(self) -> FusedState:
        """Returns current optimal state."""
        return FusedState(x=self.x, y=self.y, yaw=self.yaw, v=self.v)

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0.0
        self.p_pos = 0.1
        self.p_yaw = 0.05
        self.p_v = 0.1
