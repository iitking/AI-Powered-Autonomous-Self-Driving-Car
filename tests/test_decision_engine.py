"""Unit tests for Behavior Planner and State Machine."""

import unittest
from src.config import DEFAULT_CONFIG
from src.decision.behavior_planner import BehaviorPlanner
from src.decision.state_machine import DrivingMode, DrivingState
from src.perception.lane_detector import LaneDetectionResult
from src.perception.object_detector import DetectedObject, BoundingBox
from src.perception.traffic_light_detector import TrafficLightState


class TestDecisionEngine(unittest.TestCase):

    def setUp(self):
        self.planner = BehaviorPlanner(DEFAULT_CONFIG)
        self.nominal_lane = LaneDetectionResult(detected=True, cross_track_error_m=0.0)

    def test_emergency_stop_on_critical_obstacle(self):
        # Obstacle at 0.15m (< critical_stop_distance of 0.30m)
        decision = self.planner.plan(
            lane=self.nominal_lane,
            objects=[],
            traffic_light=TrafficLightState.GREEN,
            min_sensor_distance_m=0.15,
        )
        self.assertEqual(decision.driving_state, DrivingState.EMERGENCY_STOP)
        self.assertEqual(decision.target_speed_mps, 0.0)
        self.assertTrue(decision.emergency_brake)

    def test_red_traffic_light_stop(self):
        decision = self.planner.plan(
            lane=self.nominal_lane,
            objects=[],
            traffic_light=TrafficLightState.RED,
            min_sensor_distance_m=2.0,
        )
        self.assertEqual(decision.driving_state, DrivingState.STOPPED_AT_LIGHT)
        self.assertEqual(decision.target_speed_mps, 0.0)

    def test_obstacle_avoidance_steering_bias(self):
        # Object in path at 0.60m (between critical and slowdown)
        obj = DetectedObject(
            class_id=1,
            class_name="box",
            confidence=0.9,
            bbox=BoundingBox(0, 0, 10, 10),
            distance_m=0.60,
            lateral_offset_m=0.05, # slightly right of center
            is_in_vehicle_path=True,
        )
        decision = self.planner.plan(
            lane=self.nominal_lane,
            objects=[obj],
            traffic_light=TrafficLightState.GREEN,
            min_sensor_distance_m=0.60,
        )
        self.assertEqual(decision.driving_state, DrivingState.AVOIDING_OBSTACLE)
        # Avoids left (negative bias) when obstacle is on right
        self.assertLess(decision.lateral_offset_bias_m, 0.0)

    def test_nominal_lane_cruising(self):
        decision = self.planner.plan(
            lane=self.nominal_lane,
            objects=[],
            traffic_light=TrafficLightState.GREEN,
            min_sensor_distance_m=2.5,
        )
        self.assertEqual(decision.driving_state, DrivingState.LANE_FOLLOWING)
        self.assertGreater(decision.target_speed_mps, 0.5)


if __name__ == "__main__":
    unittest.main()
