"""
Inertial Measurement Unit (IMU) Processing (MPU6050 / BNO055).
Processes 3-axis accelerometer and 3-axis gyroscope data to compute
heading (yaw), roll, pitch, and linear accelerations.
"""

import math
from dataclasses import dataclass
from src.utils.math_utils import normalize_angle


@dataclass
class IMUData:
    ax: float = 0.0  # Linear acceleration x (forward) m/s^2
    ay: float = 0.0  # Linear acceleration y (lateral) m/s^2
    az: float = 9.81 # Linear acceleration z (vertical) m/s^2
    gx: float = 0.0  # Angular rate roll rad/s
    gy: float = 0.0  # Angular rate pitch rad/s
    gz: float = 0.0  # Angular rate yaw rad/s


class IMUProcessor:
    """
    Fuses gyroscope integration and accelerometer tilt with gyro bias cancellation.
    """

    def __init__(self, gyro_bias_z: float = 0.0):
        self.gyro_bias_z = gyro_bias_z
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

    def update(self, data: IMUData, dt: float) -> float:
        """
        Integrates yaw gyro rate with bias subtraction.
        Returns estimated vehicle heading (yaw) in radians.
        """
        # Pitch and roll from gravity vector
        denom = math.sqrt(data.ay ** 2 + data.az ** 2)
        if denom > 1e-4:
            self.pitch = math.atan2(data.ax, denom)
        if abs(data.az) > 1e-4:
            self.roll = math.atan2(-data.ay, data.az)

        # Unbiased yaw rate
        unbiased_gz = data.gz - self.gyro_bias_z
        self.yaw = normalize_angle(self.yaw + unbiased_gz * dt)

        return self.yaw

    def calibrate_bias(self, sample_gz_list: list):
        """Calculates zero-rate gyroscope bias from stationary samples."""
        if sample_gz_list:
            self.gyro_bias_z = sum(sample_gz_list) / len(sample_gz_list)

    def reset(self, initial_yaw: float = 0.0):
        self.yaw = initial_yaw
        self.pitch = 0.0
        self.roll = 0.0
