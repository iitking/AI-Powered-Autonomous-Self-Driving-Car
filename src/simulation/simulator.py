"""
Master Simulator for Autonomous Self-Driving Car.
Integrates Perception, Behavior Planning, PID Control, Kinematic Physics,
Sensor Fusion, and World Simulation into an interactive loop.
"""

import math
import time
from typing import Any, Dict, List, Optional

from src.config import Config, DEFAULT_CONFIG
from src.control.pid_controller import PIDTelemetry
from src.control.speed_controller import SpeedController
from src.control.steering_controller import SteeringController
from src.decision.behavior_planner import BehaviorDecision, BehaviorPlanner
from src.decision.state_machine import DrivingMode, DrivingState
from src.navigation.localization import OdometryLocalizer, Pose2D
from src.navigation.obstacle_avoidance import ObstacleAvoidancePlanner
from src.perception.lane_detector import LaneDetectionResult, LaneDetector
from src.perception.object_detector import DetectedObject
from src.perception.traffic_light_detector import TrafficLightState
from src.sensors.sensor_fusion import SensorFusionEKF
from src.sensors.ultrasonic import UltrasonicSensor
from src.simulation.vehicle_model import KinematicBicycleModel
from src.simulation.world import SimulationWorld, WorldObstacle


class AutonomousCarSimulator:
    """
    Complete software-in-the-loop (SIL) autonomous vehicle simulator.
    """

    def __init__(self, config: Optional[Config] = None):
        self.config = config or DEFAULT_CONFIG

        # Physics and World
        self.world = SimulationWorld(
            track_radius=2.2,
            straight_length=4.0,
            lane_width=self.config.camera.lane_width_meters * 1.5,
        )
        self.vehicle = KinematicBicycleModel(self.config.vehicle)

        # Controllers and Autonomy Stack
        self.lane_detector = LaneDetector(self.config.camera)
        self.behavior_planner = BehaviorPlanner(self.config)
        self.steering_ctrl = SteeringController(self.config.vehicle, self.config.pid)
        self.speed_ctrl = SpeedController(self.config.vehicle, self.config.pid, self.config.safety)
        self.avoidance_planner = ObstacleAvoidancePlanner()

        # State Estimation & Sensors
        self.ekf = SensorFusionEKF()
        self.localizer = OdometryLocalizer()
        self.front_ultrasonic = UltrasonicSensor("front")

        # Telemetry & States
        self.latest_decision = BehaviorDecision(
            driving_state=DrivingState.INITIALIZING,
            target_speed_mps=0.0,
            lateral_offset_bias_m=0.0,
            maneuver="INIT",
            reason="System starting",
        )
        self.latest_steer_telemetry = PIDTelemetry()
        self.latest_speed_telemetry = PIDTelemetry()
        self.latest_lane = LaneDetectionResult()
        self.latest_objects: List[DetectedObject] = []
        self.latest_traffic_light = TrafficLightState.GREEN
        self.latest_lidar_beams: list = []
        self.latest_target_point: Optional[Tuple[float, float]] = None

        # Manual controls
        self.manual_steer_deg = 0.0
        self.manual_throttle = 0.0

        # Performance & Stats
        self.step_count = 0
        self.sim_time = 0.0
        self.lap_count = 0
        self.prev_track_index = 0

        # Start position
        self.reset()

    def reset(self):
        """Resets car to starting line."""
        start_x = -self.world.straight / 2.0
        start_y = self.world.r
        start_yaw = 0.0  # Heading east (+X)

        self.vehicle.reset(x=start_x, y=start_y, yaw=start_yaw, v=0.0)
        self.steering_ctrl.reset()
        self.speed_ctrl.reset()
        self.avoidance_planner.cancel()
        self.behavior_planner.fsm.reset()
        self.ekf.reset(start_x, start_y, start_yaw)
        self.localizer.reset(Pose2D(start_x, start_y, start_yaw))
        self.sim_time = 0.0
        self.step_count = 0

    def set_driving_mode(self, mode: DrivingMode):
        """Toggles between Manual and Autonomous driving."""
        self.behavior_planner.fsm.set_mode(mode)

    def set_manual_inputs(self, steer_deg: float, throttle: float):
        """Sets user input for manual driving."""
        self.manual_steer_deg = steer_deg
        self.manual_throttle = throttle

    def add_obstacle(self, x: float, y: float, class_name: str = "box"):
        """Dynamically inserts obstacle into the world."""
        obs_id = f"obs_{len(self.world.obstacles) + 1}"
        self.world.obstacles.append(WorldObstacle(id=obs_id, x=x, y=y, radius=0.20, class_name=class_name))

    def clear_obstacles(self):
        """Removes all obstacles."""
        self.world.obstacles.clear()

    def toggle_traffic_light(self):
        """Cycles traffic light color."""
        if self.world.traffic_lights:
            tl = self.world.traffic_lights[0]
            if tl.state == TrafficLightState.GREEN:
                tl.state = TrafficLightState.YELLOW
            elif tl.state == TrafficLightState.YELLOW:
                tl.state = TrafficLightState.RED
            else:
                tl.state = TrafficLightState.GREEN

    def toggle_traffic_car(self):
        """Toggles the moving lead traffic car on/off."""
        if self.world.traffic_cars:
            self.world.traffic_cars.clear()
        else:
            from src.simulation.world import DynamicTrafficCar
            _, _, _, _, idx = self.world.find_nearest_track_point(self.vehicle.state.x, self.vehicle.state.y)
            lead_idx = (idx + 35) % len(self.world.center_line)
            prog = (lead_idx / float(len(self.world.center_line))) * self.world.track_length_m
            self.world.traffic_cars.append(
                DynamicTrafficCar(id="car_npc_1", track_progress=prog, speed_mps=0.85)
            )

    def stall_traffic_car(self):
        """Toggles stall on lead vehicle so ego car overtakes it!"""
        if self.world.traffic_cars:
            car = self.world.traffic_cars[0]
            car.is_stalled = not car.is_stalled
            car.speed_mps = 0.0 if car.is_stalled else 0.85

    def trigger_pedestrian(self):
        """Triggers a pedestrian crossing the road."""
        if self.world.pedestrians:
            ped = self.world.pedestrians[0]
            ped.is_active = not ped.is_active
            ped.walk_progress = -1.0
            ped.walk_direction = 1.0

    def clear_all_traffic(self):
        """Clears all obstacles, traffic cars, and active pedestrians."""
        self.world.obstacles.clear()
        self.world.traffic_cars.clear()
        for p in self.world.pedestrians:
            p.is_active = False

    def step(self, dt: float = 0.05):
        """
        Advances the simulation loop by dt:
        1. World Update (traffic lights, dynamic obstacles)
        2. Perception (Lane, YOLO objects, Traffic Light, LiDAR)
        3. Decision Planning (State Machine arbitration)
        4. Control Execution (Steering & Speed PID)
        5. Vehicle Kinematics Integration
        6. Sensor Fusion & Odometry Update
        """
        self.step_count += 1
        self.sim_time += dt

        # 1. Update World
        self.world.step(dt)

        car_x = self.vehicle.state.x
        car_y = self.vehicle.state.y
        car_yaw = self.vehicle.state.yaw
        current_v = self.vehicle.state.velocity

        # 2. Perception Pipeline
        # Lane Detection
        self.latest_lane = self.world.get_lane_perception(car_x, car_y, car_yaw)
        # YOLO Object Detection
        self.latest_objects = self.world.get_object_detections(car_x, car_y, car_yaw)
        # Traffic Light Detection
        tl_state, tl_dist = self.world.get_traffic_light_state(car_x, car_y, car_yaw)
        self.latest_traffic_light = tl_state
        # LiDAR Raycasting
        self.latest_lidar_beams = self.world.raycast_lidar(
            car_x, car_y, car_yaw,
            num_beams=self.config.simulation.lidar_num_beams,
            max_range=self.config.simulation.lidar_max_range,
        )
        # Forward collision check specifically along bumper path
        forward_collision_dist = self.world.get_forward_collision_distance(car_x, car_y, car_yaw)
        self.front_ultrasonic.update(forward_collision_dist)
        filtered_ultrasonic = self.front_ultrasonic.get_distance()

        # 3. Decision Engine / Behavior Planning
        self.latest_decision = self.behavior_planner.plan(
            lane=self.latest_lane,
            objects=self.latest_objects,
            traffic_light=self.latest_traffic_light,
            direct_forward_collision_dist_m=forward_collision_dist,
            manual_throttle=self.manual_throttle,
            manual_steer=self.manual_steer_deg,
        )

        # 4. Control Execution
        if self.behavior_planner.fsm.mode == DrivingMode.MANUAL:
            commanded_steer_deg = self.manual_steer_deg
            throttle = max(0.0, self.manual_throttle)
            brake = max(0.0, -self.manual_throttle)
        else:
            # Lateral Control (Steering)
            # Combine lane CTE with behavior avoidance offset bias
            # (negative bias shifts vehicle left, positive bias shifts vehicle right)
            effective_cte = self.latest_lane.cross_track_error_m - self.latest_decision.lateral_offset_bias_m

            # Lookahead target along track centerline
            lookahead = self.world.get_lookahead_target(car_x, car_y, self.config.pid.lookahead_dist_m)
            tx, ty, th = lookahead
            bias = self.latest_decision.lateral_offset_bias_m
            shifted_lookahead = (
                tx + bias * math.sin(th),
                ty - bias * math.cos(th),
                th,
            )
            self.latest_target_point = (shifted_lookahead[0], shifted_lookahead[1])

            if self.latest_decision.driving_state == DrivingState.RECOVERY:
                # Direct aggressive heading towards nearest road centerline
                nearest_cx, nearest_cy, _, _, _ = self.world.find_nearest_track_point(car_x, car_y)
                target_yaw = math.atan2(nearest_cy - car_y, nearest_cx - car_x)
                heading_diff = normalize_angle(target_yaw - car_yaw)
                commanded_steer_deg = clamp(
                    math.degrees(heading_diff) * 1.5,
                    -self.config.vehicle.max_steering_angle,
                    self.config.vehicle.max_steering_angle,
                )
                self.latest_steer_telemetry = PIDTelemetry(
                    target=0.0,
                    current=effective_cte,
                    error=effective_cte,
                    p_term=round(commanded_steer_deg, 2),
                    output=round(commanded_steer_deg, 2),
                )
            else:
                commanded_steer_deg, self.latest_steer_telemetry = self.steering_ctrl.compute_steering(
                    cross_track_error_m=effective_cte,
                    heading_error_rad=self.latest_lane.heading_error_rad,
                    current_speed_mps=current_v,
                    dt=dt,
                    lookahead_point=shifted_lookahead,
                    car_pose=(car_x, car_y, car_yaw),
                )

            # Longitudinal Control (Speed)
            target_v = self.latest_decision.target_speed_mps
            # When actively overtaking/avoiding an obstacle, allow vehicle to bypass without false headway braking
            obs_dist_for_speed = filtered_ultrasonic if self.latest_decision.driving_state != DrivingState.AVOIDING_OBSTACLE else 999.0
            safe_v = self.speed_ctrl.compute_safe_speed(
                nominal_speed_mps=target_v,
                road_curvature_radius_m=self.latest_lane.curvature_radius_m,
                closest_obstacle_dist_m=obs_dist_for_speed,
            )

            if self.latest_decision.emergency_brake:
                throttle = 0.0
                brake = 1.0
            else:
                throttle, brake, self.latest_speed_telemetry = self.speed_ctrl.update(
                    target_speed_mps=safe_v,
                    current_speed_mps=current_v,
                    dt=dt,
                )

        # 5. Physics Step (Kinematic Bicycle Model)
        self.vehicle.step(
            target_steer_deg=commanded_steer_deg,
            throttle=throttle,
            brake=brake,
            dt=dt,
        )

        # 6. Sensor Fusion & Odometry
        # Gyro yaw rate: omega = (v / L) * tan(delta)
        steer_rad = self.vehicle.state.steer_angle
        gyro_rate = (self.vehicle.state.velocity / self.config.vehicle.wheelbase) * math.tan(steer_rad)

        self.ekf.predict(accel_x=self.vehicle.state.accel, gyro_gz=gyro_rate, dt=dt)
        self.ekf.update_encoder(self.vehicle.state.velocity)
        self.localizer.update(self.vehicle.state.velocity, gyro_rate, dt)

        # 7. Lap Completion Tracking
        _, _, _, _, current_idx = self.world.find_nearest_track_point(self.vehicle.state.x, self.vehicle.state.y)
        track_size = len(self.world.center_line)
        if self.prev_track_index > track_size - 30 and current_idx < 30:
            self.lap_count += 1
        self.prev_track_index = current_idx

    def get_full_state(self) -> Dict[str, Any]:
        """Returns comprehensive JSON-serializable state dictionary for dashboard/telemetry."""
        pos = self.vehicle.state

        return {
            "timestamp": time.time(),
            "sim_time": round(self.sim_time, 2),
            "step": self.step_count,
            "lap": self.lap_count,
            "vehicle": {
                "x": round(pos.x, 3),
                "y": round(pos.y, 3),
                "yaw_rad": round(pos.yaw, 3),
                "yaw_deg": round(math.degrees(pos.yaw), 1),
                "velocity_mps": round(pos.velocity, 2),
                "velocity_kmh": round(pos.velocity * 3.6, 1),
                "steer_angle_deg": round(math.degrees(pos.steer_angle), 1),
                "accel": round(pos.accel, 2),
            },
            "control": {
                "mode": self.behavior_planner.fsm.mode.value,
                "state": self.latest_decision.driving_state.value,
                "maneuver": self.latest_decision.maneuver,
                "reason": self.latest_decision.reason,
                "emergency_brake": self.latest_decision.emergency_brake,
                "pid_steer": {
                    "p": round(self.latest_steer_telemetry.p_term, 3),
                    "i": round(self.latest_steer_telemetry.i_term, 3),
                    "d": round(self.latest_steer_telemetry.d_term, 3),
                    "error": round(self.latest_steer_telemetry.error, 3),
                    "output": round(self.latest_steer_telemetry.output, 2),
                },
                "pid_speed": {
                    "p": round(self.latest_speed_telemetry.p_term, 3),
                    "i": round(self.latest_speed_telemetry.i_term, 3),
                    "d": round(self.latest_speed_telemetry.d_term, 3),
                    "target": round(self.latest_speed_telemetry.target, 2),
                    "current": round(self.latest_speed_telemetry.current, 2),
                },
            },
            "perception": {
                "cte_m": round(self.latest_lane.cross_track_error_m, 3),
                "heading_err_deg": round(math.degrees(self.latest_lane.heading_error_rad), 1),
                "curvature_radius_m": round(self.latest_lane.curvature_radius_m, 2) if self.latest_lane.curvature_radius_m < 1000 else 999.0,
                "detected_objects": [
                    {
                        "class": obj.class_name,
                        "distance_m": round(obj.distance_m, 2),
                        "lateral_m": round(obj.lateral_offset_m, 2),
                        "in_path": obj.is_in_vehicle_path,
                    }
                    for obj in self.latest_objects
                ],
                "traffic_light": {
                    "state": self.latest_traffic_light.value,
                },
            },
            "world": {
                "track_radius": self.world.r,
                "straight_length": self.world.straight,
                "lane_width": self.world.lane_width,
                "obstacles": [
                    {"id": obs.id, "x": round(obs.x, 2), "y": round(obs.y, 2), "radius": obs.radius, "class": obs.class_name}
                    for obs in self.world.obstacles
                ],
                "traffic_cars": [
                    {
                        "id": tc.id,
                        "x": round(tc.x, 2),
                        "y": round(tc.y, 2),
                        "yaw": round(tc.yaw, 2),
                        "speed_mps": round(tc.speed_mps, 2),
                        "is_stalled": tc.is_stalled,
                    }
                    for tc in self.world.traffic_cars
                ],
                "pedestrians": [
                    {
                        "id": ped.id,
                        "x": round(ped.x, 2),
                        "y": round(ped.y, 2),
                        "is_active": ped.is_active,
                        "walk_progress": round(ped.walk_progress, 2),
                    }
                    for ped in self.world.pedestrians
                ],
                "traffic_lights": [
                    {"id": tl.id, "x": round(tl.x, 2), "y": round(tl.y, 2), "state": tl.state.value if hasattr(tl.state, 'value') else str(tl.state)}
                    for tl in self.world.traffic_lights
                ],
                "lidar_rays": [
                    [round(b[0], 2), round(b[1], 2), round(b[2], 2), round(b[3], 2)]
                    for b in self.latest_lidar_beams
                ],
                "target_point": {
                    "x": round(self.latest_target_point[0], 2) if self.latest_target_point else 0.0,
                    "y": round(self.latest_target_point[1], 2) if self.latest_target_point else 0.0,
                },
            },
        }
