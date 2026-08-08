"""
Vehicle Localization and Dead-Reckoning Odometry.
Tracks 2D pose (x, y, heading) relative to origin using wheel encoder
speed and IMU angular rate.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from src.utils.math_utils import normalize_angle


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw_rad: float = 0.0

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw_rad)


class OdometryLocalizer:
    """
    Integrates wheel odometry and gyroscope heading into 2D global pose.
    """

    def __init__(self, initial_pose: Optional[Pose2D] = None):
        self.pose = initial_pose or Pose2D()
        self.trajectory: List[Tuple[float, float]] = [(self.pose.x, self.pose.y)]
        self.total_distance = 0.0

    def update(self, linear_velocity: float, yaw_rate_rad_s: float, dt: float) -> Pose2D:
        """
        Updates pose via kinematic dead reckoning:
        theta_{t+1} = theta_t + w * dt
        x_{t+1} = x_t + v * cos(theta_mid) * dt
        y_{t+1} = y_t + v * sin(theta_mid) * dt
        """
        delta_dist = linear_velocity * dt
        self.total_distance += abs(delta_dist)

        # Midpoint integration for higher accuracy
        theta_mid = self.pose.yaw_rad + (yaw_rate_rad_s * dt) / 2.0
        self.pose.x += delta_dist * math.cos(theta_mid)
        self.pose.y += delta_dist * math.sin(theta_mid)
        self.pose.yaw_rad = normalize_angle(self.pose.yaw_rad + yaw_rate_rad_s * dt)

        # Record trajectory history (subsampled to save memory)
        last_x, last_y = self.trajectory[-1]
        if math.hypot(self.pose.x - last_x, self.pose.y - last_y) >= 0.05:
            self.trajectory.append((self.pose.x, self.pose.y))
            if len(self.trajectory) > 500:
                self.trajectory.pop(0)

        return self.pose

    def reset(self, new_pose: Optional[Pose2D] = None):
        self.pose = new_pose or Pose2D()
        self.trajectory = [(self.pose.x, self.pose.y)]
        self.total_distance = 0.0
