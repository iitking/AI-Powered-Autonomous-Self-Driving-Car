"""
Ultrasonic Rangefinder Sensor Interface and Processing.
Implements spike rejection and median filtering for HC-SR04 sensors.
"""

from typing import List, Optional
from src.utils.math_utils import clamp


class UltrasonicSensor:
    """
    HC-SR04 ultrasonic rangefinder model with outlier filtering.
    """

    def __init__(
        self,
        sensor_id: str = "front",
        min_range_m: float = 0.02,
        max_range_m: float = 3.50,
        filter_window_size: int = 5,
    ):
        self.sensor_id = sensor_id
        self.min_range = min_range_m
        self.max_range = max_range_m
        self.window_size = filter_window_size
        self.readings_history: List[float] = []
        self.filtered_distance = max_range_m

    def update(self, raw_distance_m: float) -> float:
        """
        Ingests a new raw distance measurement, filters out noise,
        and returns the robust filtered distance in meters.
        """
        # Validate sanity
        if raw_distance_m <= 0.0 or raw_distance_m > self.max_range * 1.5:
            # Drop invalid readings
            return self.filtered_distance

        clamped = clamp(raw_distance_m, self.min_range, self.max_range)
        self.readings_history.append(clamped)
        if len(self.readings_history) > self.window_size:
            self.readings_history.pop(0)

        # Apply median filter
        sorted_window = sorted(self.readings_history)
        mid = len(sorted_window) // 2
        if len(sorted_window) % 2 == 1:
            self.filtered_distance = sorted_window[mid]
        else:
            self.filtered_distance = (sorted_window[mid - 1] + sorted_window[mid]) / 2.0

        return self.filtered_distance

    def get_distance(self) -> float:
        """Returns the current filtered distance in meters."""
        return self.filtered_distance

    def reset(self):
        self.readings_history.clear()
        self.filtered_distance = self.max_range

# Outlier filtering with sliding window median

# Distance getter property for SIL integration
