"""
Behavior Decision Planner for Autonomous Self-Driving Car.
Supports:
- Proactive obstacle overtaking & bypass corridor selection (no freezing!)
- Adaptive Cruise Control (ACC) following lead traffic vehicles
- Pedestrian crosswalk yielding & automatic continuation
- Traffic light compliance (Red stop, Yellow slow, Green go)
- Direct forward-bumper emergency braking with auto-resume
- Off-track recovery to road centerline
"""

from dataclasses import dataclass
from typing import List, Optional

from src.config import Config, DEFAULT_CONFIG
from src.decision.state_machine import BehaviorStateMachine, DrivingMode, DrivingState
from src.perception.lane_detector import LaneDetectionResult
from src.perception.object_detector import DetectedObject
from src.perception.traffic_light_detector import TrafficLightState


@dataclass
class BehaviorDecision:
    driving_state: DrivingState
    target_speed_mps: float
    lateral_offset_bias_m: float  # Lateral shift added to steer around obstacles
    maneuver: str
    reason: str
    emergency_brake: bool = False


class BehaviorPlanner:
    """
    High-level decision engine orchestrating autonomous driving behaviors.
    """

    def __init__(self, config: Optional[Config] = None):
        self.config = config or DEFAULT_CONFIG
        self.fsm = BehaviorStateMachine()

        # Active avoidance latch so maneuver completes smoothly
        self.active_avoid_bias: float = 0.0
        self.avoid_target_id: Optional[str] = None
        self.avoid_steps_remaining: int = 0

    def plan(
        self,
        lane: LaneDetectionResult,
        objects: List[DetectedObject],
        traffic_light: TrafficLightState,
        direct_forward_collision_dist_m: float = 999.0,
        manual_throttle: float = 0.0,
        manual_steer: float = 0.0,
        min_sensor_distance_m: Optional[float] = None,
    ) -> BehaviorDecision:
        if min_sensor_distance_m is not None and direct_forward_collision_dist_m == 999.0:
            direct_forward_collision_dist_m = min_sensor_distance_m
        """
        Decision hierarchy:
        1. Manual Override
        2. Direct Forward Collision Watchdog (Unavoidable bumper emergency stop)
        3. Traffic Light Stop (Red / Yellow)
        4. Pedestrian Crosswalk Yielding
        5. Active Obstacle Overtaking & Corridor Bypass
        6. Adaptive Cruise Control (Following Lead Traffic Car)
        7. Off-Track Recovery
        8. Normal Lane Cruising
        """
        cfg_safety = self.config.safety
        cfg_veh = self.config.vehicle

        # 1. Manual Override
        if self.fsm.mode == DrivingMode.MANUAL:
            return BehaviorDecision(
                driving_state=DrivingState.MANUAL_DRIVE,
                target_speed_mps=manual_throttle * cfg_veh.max_speed,
                lateral_offset_bias_m=0.0,
                maneuver="MANUAL_CONTROL",
                reason="Manual driver control active",
            )

        # 2. Direct Forward Collision Failsafe (Only when obstacle is directly in bumper path < 0.20m)
        if direct_forward_collision_dist_m <= 0.20:
            self.fsm.transition_to(
                DrivingState.EMERGENCY_STOP,
                f"Direct head-on obstacle ({direct_forward_collision_dist_m:.2f}m)"
            )
            return BehaviorDecision(
                driving_state=DrivingState.EMERGENCY_STOP,
                target_speed_mps=0.0,
                lateral_offset_bias_m=self.active_avoid_bias,
                maneuver="EMERGENCY_BRAKE",
                reason=f"Imminent impact: obstacle at {direct_forward_collision_dist_m:.2f}m",
                emergency_brake=True,
            )

        # 3. Traffic Light Compliance
        if traffic_light == TrafficLightState.RED:
            self.fsm.transition_to(DrivingState.STOPPED_AT_LIGHT, "Red traffic light")
            return BehaviorDecision(
                driving_state=DrivingState.STOPPED_AT_LIGHT,
                target_speed_mps=0.0,
                lateral_offset_bias_m=0.0,
                maneuver="STOP_AT_LIGHT",
                reason="Traffic light RED: stopped at stop line",
                emergency_brake=False,
            )
        elif traffic_light == TrafficLightState.YELLOW:
            self.fsm.transition_to(DrivingState.APPROACHING_TRAFFIC_LIGHT, "Yellow light")
            return BehaviorDecision(
                driving_state=DrivingState.APPROACHING_TRAFFIC_LIGHT,
                target_speed_mps=cfg_veh.min_speed,
                lateral_offset_bias_m=0.0,
                maneuver="SLOW_FOR_LIGHT",
                reason="Traffic light YELLOW: slowing down",
            )

        # 4. Pedestrian Crosswalk Yielding
        pedestrians_in_path = [o for o in objects if o.class_name == "pedestrian" and o.is_in_vehicle_path]
        if pedestrians_in_path:
            closest_ped = min(pedestrians_in_path, key=lambda p: p.distance_m)
            if closest_ped.distance_m <= 0.50:
                # Full stop before crosswalk
                return BehaviorDecision(
                    driving_state=DrivingState.STOPPED_AT_SIGN,
                    target_speed_mps=0.0,
                    lateral_offset_bias_m=0.0,
                    maneuver="YIELD_PEDESTRIAN",
                    reason=f"Pedestrian in crosswalk at {closest_ped.distance_m:.2f}m: waiting",
                )
            elif closest_ped.distance_m <= 1.80:
                # Smooth slowdown
                return BehaviorDecision(
                    driving_state=DrivingState.APPROACHING_TRAFFIC_LIGHT,
                    target_speed_mps=cfg_veh.min_speed * 0.8,
                    lateral_offset_bias_m=0.0,
                    maneuver="SLOW_FOR_PEDESTRIAN",
                    reason=f"Pedestrian ahead at {closest_ped.distance_m:.2f}m: slowing down",
                )

        # 5. Active Obstacle Overtaking & Bypass
        # Stationary obstacles (cones, boxes, obstacles, stalled cars) that block ego path
        stationary_obstacles = [
            o for o in objects
            if o.class_name in ["cone", "box", "obstacle", "stalled_car"] and o.is_in_vehicle_path
        ]

        if stationary_obstacles:
            closest_obs = min(stationary_obstacles, key=lambda o: o.distance_m)
            d = closest_obs.distance_m

            # Begin evasion well in advance (up to 2.5m ahead)
            if 0.15 <= d <= 2.5:
                # If not already committed to an avoidance side, latch corridor direction:
                # (lateral_offset >= 0 -> shift left -0.32m; lateral_offset < 0 -> shift right +0.32m)
                if self.avoid_steps_remaining == 0:
                    if closest_obs.lateral_offset_m >= 0.0:
                        self.active_avoid_bias = -0.32  # Shift left
                    else:
                        self.active_avoid_bias = 0.32   # Shift right

                # Refresh latch window while obstacle remains ahead
                self.avoid_steps_remaining = 45

                self.fsm.transition_to(DrivingState.AVOIDING_OBSTACLE, f"Overtaking {closest_obs.class_name}")
                return BehaviorDecision(
                    driving_state=DrivingState.AVOIDING_OBSTACLE,
                    target_speed_mps=min(cfg_veh.max_speed * 0.75, 1.05),
                    lateral_offset_bias_m=self.active_avoid_bias,
                    maneuver=f"OVERTAKE_{closest_obs.class_name.upper()}",
                    reason=f"Bypassing {closest_obs.class_name} at {d:.2f}m ({'LEFT' if self.active_avoid_bias < 0 else 'RIGHT'})",
                )

        # Continue existing avoidance maneuver if clearing obstacle body
        if self.avoid_steps_remaining > 0:
            self.avoid_steps_remaining -= 1
            # Smoothly ramp bias back down on final steps
            decay_factor = min(1.0, self.avoid_steps_remaining / 15.0)
            current_bias = self.active_avoid_bias * decay_factor
            return BehaviorDecision(
                driving_state=DrivingState.AVOIDING_OBSTACLE,
                target_speed_mps=min(cfg_veh.max_speed * 0.75, 1.1),
                lateral_offset_bias_m=current_bias,
                maneuver="CLEARING_OBSTACLE",
                reason="Clearing obstacle corridor and returning to centerline",
            )
        else:
            self.active_avoid_bias = 0.0

        # 6. Adaptive Cruise Control (ACC) Following Moving Lead Car
        lead_cars = [o for o in objects if o.class_name == "car" and o.is_in_vehicle_path]
        if lead_cars:
            closest_car = min(lead_cars, key=lambda c: c.distance_m)
            d = closest_car.distance_m

            if d < 0.60:
                # Too close to lead car: brake smoothly
                target_speed = 0.3
            elif d < 1.40:
                # Match lead vehicle speed (safe headway)
                target_speed = 0.85
            else:
                # Closing in smoothly
                target_speed = min(cfg_veh.max_speed, 1.2)

            return BehaviorDecision(
                driving_state=DrivingState.LANE_FOLLOWING,
                target_speed_mps=target_speed,
                lateral_offset_bias_m=0.0,
                maneuver="ACC_FOLLOW_LEAD_CAR",
                reason=f"Adaptive Cruise Control: lead vehicle at {d:.2f}m",
            )

        # 7. Off-Track Recovery Maneuver
        if abs(lane.cross_track_error_m) > 0.40:
            self.fsm.transition_to(DrivingState.RECOVERY, f"Off-track deviation ({lane.cross_track_error_m:+.2f}m)")
            return BehaviorDecision(
                driving_state=DrivingState.RECOVERY,
                target_speed_mps=max(cfg_veh.min_speed, 0.45),
                lateral_offset_bias_m=0.0,
                maneuver="RECOVERY_ALIGN",
                reason=f"Off-track recovery active (CTE={lane.cross_track_error_m:+.2f}m)",
            )

        # 8. Normal Lane Cruising
        self.fsm.transition_to(DrivingState.LANE_FOLLOWING, "Path clear")
        target_speed = cfg_veh.max_speed

        if lane.curvature_radius_m < 3.0:
            target_speed = max(cfg_veh.min_speed, target_speed * 0.75)

        return BehaviorDecision(
            driving_state=DrivingState.LANE_FOLLOWING,
            target_speed_mps=target_speed,
            lateral_offset_bias_m=0.0,
            maneuver="CRUISE",
            reason="Cruising along lane",
        )
