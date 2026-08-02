"""
Hierarchical Finite State Machine for Autonomous Vehicle Behaviors.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional
import time


class DrivingMode(str, Enum):
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"


class DrivingState(str, Enum):
    INITIALIZING = "INITIALIZING"
    MANUAL_DRIVE = "MANUAL_DRIVE"
    LANE_FOLLOWING = "LANE_FOLLOWING"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    APPROACHING_TRAFFIC_LIGHT = "APPROACHING_TRAFFIC_LIGHT"
    STOPPED_AT_LIGHT = "STOPPED_AT_LIGHT"
    STOPPED_AT_SIGN = "STOPPED_AT_SIGN"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    RECOVERY = "RECOVERY"


@dataclass
class StateTransitionLog:
    timestamp: float
    from_state: DrivingState
    to_state: DrivingState
    reason: str


class BehaviorStateMachine:
    """
    State machine orchestrating the driving states and safety boundaries.
    """

    def __init__(self, initial_state: DrivingState = DrivingState.LANE_FOLLOWING):
        self.current_state = initial_state
        self.previous_state = initial_state
        self.state_entry_time = time.time()
        self.transition_history: List[StateTransitionLog] = []
        self.mode = DrivingMode.AUTONOMOUS

    @property
    def time_in_state(self) -> float:
        """Returns elapsed time in current state (seconds)."""
        return time.time() - self.state_entry_time

    def transition_to(self, new_state: DrivingState, reason: str = ""):
        """Executes state transition if valid."""
        if new_state == self.current_state:
            return

        now = time.time()
        log = StateTransitionLog(
            timestamp=now,
            from_state=self.current_state,
            to_state=new_state,
            reason=reason,
        )
        self.transition_history.append(log)
        # Keep log size bounded
        if len(self.transition_history) > 100:
            self.transition_history.pop(0)

        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_entry_time = now

    def set_mode(self, mode: DrivingMode):
        """Switches between manual and autonomous modes."""
        self.mode = mode
        if mode == DrivingMode.MANUAL:
            self.transition_to(DrivingState.MANUAL_DRIVE, "User switched to manual mode")
        elif self.current_state == DrivingState.MANUAL_DRIVE:
            self.transition_to(DrivingState.LANE_FOLLOWING, "User enabled autonomous mode")

    def reset(self):
        self.current_state = DrivingState.LANE_FOLLOWING
        self.previous_state = DrivingState.LANE_FOLLOWING
        self.state_entry_time = time.time()
        self.transition_history.clear()
