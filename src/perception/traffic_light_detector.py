"""
Traffic Light Perception and State Classification.
Classifies state (RED, YELLOW, GREEN, NONE) from ROI crops or bounding boxes.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import cv2
    import numpy as np
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    cv2 = None
    np = None


class TrafficLightState(str, Enum):
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"
    UNKNOWN = "UNKNOWN"


@dataclass
class TrafficLightDetection:
    state: TrafficLightState
    confidence: float
    distance_m: float
    stop_required: bool


class TrafficLightDetector:
    """Detects and classifies traffic light signals."""

    def __init__(self):
        self.last_state = TrafficLightState.UNKNOWN

    def classify_crop(self, crop_image) -> TrafficLightState:
        """
        Classifies color state of a cropped traffic light image using HSV color space.
        """
        if not HAS_OPENCV or crop_image is None or crop_image.size == 0:
            return TrafficLightState.UNKNOWN

        hsv = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)

        # Red mask (wraps around 0 and 180 in HSV)
        red_mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Yellow mask
        yellow_mask = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([35, 255, 255]))

        # Green mask
        green_mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([90, 255, 255]))

        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)

        max_count = max(red_count, yellow_count, green_count)
        if max_count < 20:
            return TrafficLightState.UNKNOWN

        if max_count == red_count:
            return TrafficLightState.RED
        elif max_count == yellow_count:
            return TrafficLightState.YELLOW
        else:
            return TrafficLightState.GREEN

    def evaluate(self, state: TrafficLightState, distance_m: float) -> TrafficLightDetection:
        """Evaluates whether stopping is required based on state and distance."""
        stop_req = (state == TrafficLightState.RED) or (state == TrafficLightState.YELLOW and distance_m < 1.0)
        return TrafficLightDetection(
            state=state,
            confidence=0.95 if state != TrafficLightState.UNKNOWN else 0.0,
            distance_m=distance_m,
            stop_required=stop_req,
        )
