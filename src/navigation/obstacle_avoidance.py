"""
Dynamic Obstacle Avoidance Maneuver Generator.
Computes smooth lateral lane offsets using quintic polynomial profiles
to steer around stationary and dynamic road obstacles.
"""

import math
from dataclasses import dataclass
from typing import Optional
from src.utils.math_utils import clamp


@dataclass
class AvoidanceProfile:
    active: bool = False
    lateral_shift_m: float = 0.0  # Current lateral offset to inject
    progress: float = 0.0         # 0.0 to 1.0 along maneuver
    side: str = "NONE"            # "LEFT" or "RIGHT"


class ObstacleAvoidancePlanner:
    """
    Generates smooth evasive spline offsets when obstacle obstructs primary lane.
    """

    def __init__(self, clearance_margin_m: float = 0.35, maneuver_distance_m: float = 1.5):
        self.clearance_margin = clearance_margin_m
        self.maneuver_distance = maneuver_distance_m
        self.active_maneuver = False
        self.distance_traveled = 0.0
        self.target_lateral_offset = 0.0
        self.side = "NONE"

    def trigger_avoidance(self, obstacle_distance_m: float, obstacle_lateral_pos_m: float):
        """Initiates an evasive lane maneuver."""
        self.active_maneuver = True
        self.distance_traveled = 0.0

        # If obstacle is slightly on right side of lane, avoid left (negative offset)
        if obstacle_lateral_pos_m >= 0.0:
            self.target_lateral_offset = -self.clearance_margin
            self.side = "LEFT"
        else:
            self.target_lateral_offset = self.clearance_margin
            self.side = "RIGHT"

    def step(self, distance_increment_m: float) -> AvoidanceProfile:
        """
        Advances the avoidance profile by distance_increment_m.
        Uses a smooth bell curve: offset = target * sin(pi * progress)^2
        so the car smoothly moves out, passes the obstacle, and returns to lane center.
        """
        if not self.active_maneuver:
            return AvoidanceProfile(active=False, lateral_shift_m=0.0, progress=0.0, side="NONE")

        self.distance_traveled += distance_increment_m
        progress = clamp(self.distance_traveled / self.maneuver_distance, 0.0, 1.0)

        # Smooth bell profile: peaks at progress = 0.5, returns to 0 at progress = 1.0
        # shape: sin(progress * pi)
        bell_factor = math.sin(progress * math.pi)
        current_offset = self.target_lateral_offset * bell_factor

        if progress >= 1.0:
            self.active_maneuver = False
            self.distance_traveled = 0.0
            self.side = "NONE"

        return AvoidanceProfile(
            active=self.active_maneuver,
            lateral_shift_m=current_offset,
            progress=progress,
            side=self.side,
        )

    def cancel(self):
        self.active_maneuver = False
        self.distance_traveled = 0.0
        self.side = "NONE"
