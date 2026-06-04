"""
Wheel Speed Encoder Interface.
Computes wheel rotational speed, linear distance, and linear velocity
from quadrature or optical interrupt pulses.
"""

import math
from typing import Optional


class WheelEncoder:
    """
    Tracks wheel pulses and calculates velocity and traveled distance.
    """

    def __init__(self, pulses_per_rev: int = 20, wheel_radius_m: float = 0.033):
        self.ppr = pulses_per_rev
        self.radius = wheel_radius_m
        self.circumference = 2.0 * math.pi * self.radius

        self.total_pulses = 0
        self.current_velocity_mps = 0.0
        self.total_distance_m = 0.0

    def update_ticks(self, delta_ticks: int, dt: float) -> float:
        """
        Updates encoder state with count of pulses since last cycle.
        Returns linear vehicle speed in m/s.
        """
        self.total_pulses += delta_ticks
        distance_step = (delta_ticks / self.ppr) * self.circumference
        self.total_distance_m += distance_step

        if dt > 1e-5:
            self.current_velocity_mps = distance_step / dt
        else:
            self.current_velocity_mps = 0.0

        return self.current_velocity_mps

    def reset(self):
        self.total_pulses = 0
        self.current_velocity_mps = 0.0
        self.total_distance_m = 0.0
