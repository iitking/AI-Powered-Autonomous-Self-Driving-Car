"""
Traffic Sign Recognition and Classification.
Identifies regulatory, warning, and priority signs.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Optional


class TrafficSignType(str, Enum):
    STOP = "STOP"
    YIELD = "YIELD"
    SPEED_LIMIT_30 = "SPEED_LIMIT_30"
    SPEED_LIMIT_50 = "SPEED_LIMIT_50"
    PEDESTRIAN_CROSSING = "PEDESTRIAN_CROSSING"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    UNKNOWN = "UNKNOWN"


@dataclass
class TrafficSignDetection:
    sign_type: TrafficSignType
    confidence: float
    distance_m: float
    action_suggested: str


class TrafficSignDetector:
    """Classifies detected sign crops or YOLO detections."""

    def __init__(self):
        pass

    def evaluate_sign(self, sign_type: TrafficSignType, distance_m: float) -> TrafficSignDetection:
        """Determines vehicle action associated with sign."""
        action = "CONTINUE"
        if sign_type == TrafficSignType.STOP:
            action = "STOP_AT_LINE"
        elif sign_type == TrafficSignType.YIELD:
            action = "SLOW_AND_CHECK"
        elif sign_type == TrafficSignType.SPEED_LIMIT_30:
            action = "SET_SPEED_0.8"
        elif sign_type == TrafficSignType.SPEED_LIMIT_50:
            action = "SET_SPEED_1.4"
        elif sign_type == TrafficSignType.PEDESTRIAN_CROSSING:
            action = "CAUTION_PEDESTRIAN"

        return TrafficSignDetection(
            sign_type=sign_type,
            confidence=0.92 if sign_type != TrafficSignType.UNKNOWN else 0.0,
            distance_m=distance_m,
            action_suggested=action,
        )
