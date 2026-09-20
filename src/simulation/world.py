"""
Virtual Simulation World with Dynamic Traffic, Pedestrians, Test Track, and Sensors.
Supports:
- Rounded test track with centerline and lane boundaries
- Moving lead traffic vehicles (NPC cars) for Adaptive Cruise Control
- Crossing pedestrians that step into crosswalks
- Static obstacles (cones, boxes) with bypass corridors for smooth overtaking
- Cycling traffic lights (Red/Yellow/Green)
- 2D LiDAR raycaster with forward-corridor collision separation
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from src.perception.lane_detector import LaneDetectionResult
from src.perception.object_detector import BoundingBox, DetectedObject
from src.perception.traffic_light_detector import TrafficLightState
from src.perception.traffic_sign_detector import TrafficSignType
from src.utils.math_utils import clamp, normalize_angle


@dataclass
class WorldObstacle:
    """Static or dynamic road obstacle."""
    id: str
    x: float
    y: float
    radius: float = 0.16
    class_name: str = "cone"  # "cone", "box"


@dataclass
class DynamicTrafficCar:
    """NPC traffic vehicle driving along the track."""
    id: str
    track_progress: float = 0.0  # distance along track centerline (meters)
    speed_mps: float = 0.90      # Cruising speed (m/s)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    is_stalled: bool = False     # If stalled, ego car overtakes it!
    color: str = "#a855f7"       # Vibrant purple


@dataclass
class DynamicPedestrian:
    """Pedestrian walking across the road at a crosswalk."""
    id: str
    base_x: float
    base_y: float
    heading: float
    is_active: bool = False
    walk_progress: float = 0.0   # -1.0 (left sidewalk) to +1.0 (right sidewalk)
    walk_direction: float = 1.0  # +1 or -1
    x: float = 0.0
    y: float = 0.0


@dataclass
class WorldTrafficLight:
    """Traffic light signal on the track."""
    id: str
    x: float
    y: float
    state: TrafficLightState = TrafficLightState.GREEN
    cycle_time_s: float = 9.0
    timer: float = 0.0


class SimulationWorld:
    """
    Complete physical simulation environment.
    """

    def __init__(self, track_radius: float = 2.2, straight_length: float = 4.0, lane_width: float = 0.60):
        self.r = track_radius
        self.straight = straight_length
        self.lane_width = lane_width

        # Track waypoints for center line
        self.center_line: List[Tuple[float, float, float]] = []  # (x, y, tangent_angle)
        self.track_length_m = 0.0
        self._build_track()

        # Dynamic NPC Traffic Vehicles
        self.traffic_cars: List[DynamicTrafficCar] = [
            DynamicTrafficCar(id="car_npc_1", track_progress=3.5, speed_mps=0.85),
        ]

        # Dynamic Pedestrians at designated crosswalk (on bottom straight)
        self.pedestrians: List[DynamicPedestrian] = [
            DynamicPedestrian(
                id="ped_1",
                base_x=-0.5,
                base_y=-self.r,
                heading=math.pi,
                is_active=False,
                walk_progress=-1.0,
            )
        ]

        # Static road obstacles placed with ample bypass clearance
        self.obstacles: List[WorldObstacle] = [
            # Placed on top straight with space on the left to overtake
            WorldObstacle(id="obs_1", x=0.6, y=self.r - 0.12, radius=0.15, class_name="cone"),
            WorldObstacle(id="obs_2", x=-1.0, y=-self.r + 0.12, radius=0.18, class_name="box"),
        ]

        # Traffic light alongside the road
        self.traffic_lights: List[WorldTrafficLight] = [
            WorldTrafficLight(id="light_1", x=1.6, y=self.r + 0.35, state=TrafficLightState.GREEN),
        ]

        self._update_actor_positions()

    def _build_track(self):
        """Builds dense waypoints for rounded oval track."""
        points = []
        num_straight = 60
        num_arc = 50

        # 1. Top straight (going right +X)
        for i in range(num_straight):
            t = i / float(num_straight)
            x = -self.straight / 2.0 + t * self.straight
            y = self.r
            points.append((x, y, 0.0))

        # 2. Right semicircle (turning down to bottom, clockwise)
        for i in range(num_arc):
            t = i / float(num_arc)
            angle = math.pi / 2.0 - t * math.pi
            x = self.straight / 2.0 + self.r * math.cos(angle)
            y = self.r * math.sin(angle)
            tangent = angle - math.pi / 2.0
            points.append((x, y, normalize_angle(tangent)))

        # 3. Bottom straight (going left -X)
        for i in range(num_straight):
            t = i / float(num_straight)
            x = self.straight / 2.0 - t * self.straight
            y = -self.r
            points.append((x, y, math.pi))

        # 4. Left semicircle (turning up to top, clockwise)
        for i in range(num_arc):
            t = i / float(num_arc)
            angle = -math.pi / 2.0 - t * math.pi
            x = -self.straight / 2.0 + self.r * math.cos(angle)
            y = self.r * math.sin(angle)
            tangent = angle - math.pi / 2.0
            points.append((x, y, normalize_angle(tangent)))

        self.center_line = points
        # Calculate total centerline distance
        total = 0.0
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]
            total += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        self.track_length_m = total

    def _update_actor_positions(self):
        """Updates (x, y, yaw) coordinates of moving traffic cars and pedestrians."""
        # 1. Traffic Cars
        for car in self.traffic_cars:
            frac = (car.track_progress % self.track_length_m) / max(1.0, self.track_length_m)
            idx = int(frac * len(self.center_line)) % len(self.center_line)
            cx, cy, heading = self.center_line[idx]
            car.x = cx
            car.y = cy
            car.yaw = heading

        # 2. Pedestrians
        for ped in self.pedestrians:
            # Lateral offset across the street: sidewalk = +/- 0.45m
            lat_offset = ped.walk_progress * (self.lane_width * 0.75)
            # Perpendicular to street heading
            perp_angle = ped.heading + math.pi / 2.0
            ped.x = ped.base_x + lat_offset * math.cos(perp_angle)
            ped.y = ped.base_y + lat_offset * math.sin(perp_angle)

    def step(self, dt: float):
        """Updates simulation world step: moving cars, walking pedestrians, traffic lights."""
        # 1. Traffic Lights Cycle
        for light in self.traffic_lights:
            light.timer += dt
            if light.timer >= light.cycle_time_s:
                light.timer = 0.0
                if light.state == TrafficLightState.GREEN:
                    light.state = TrafficLightState.YELLOW
                elif light.state == TrafficLightState.YELLOW:
                    light.state = TrafficLightState.RED
                elif light.state == TrafficLightState.RED:
                    light.state = TrafficLightState.GREEN

        # 2. Move Dynamic Traffic Cars
        for car in self.traffic_cars:
            if not car.is_stalled:
                car.track_progress = (car.track_progress + car.speed_mps * dt) % self.track_length_m

        # 3. Move Pedestrians
        for ped in self.pedestrians:
            if ped.is_active:
                ped.walk_progress += ped.walk_direction * (dt * 0.5)
                if ped.walk_progress >= 1.0:
                    ped.walk_progress = 1.0
                    ped.walk_direction = -1.0
                elif ped.walk_progress <= -1.0:
                    ped.walk_progress = -1.0
                    ped.walk_direction = 1.0

        self._update_actor_positions()

    def find_nearest_track_point(self, x: float, y: float) -> Tuple[float, float, float, float, int]:
        """Finds closest point on track centerline: (cx, cy, heading, distance, index)."""
        best_dist = float('inf')
        best_idx = 0
        for i, (wx, wy, heading) in enumerate(self.center_line):
            dist = math.hypot(wx - x, wy - y)
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        cx, cy, heading = self.center_line[best_idx]
        return cx, cy, heading, best_dist, best_idx

    def get_lookahead_target(self, car_x: float, car_y: float, lookahead_dist: float = 0.85) -> Tuple[float, float, float]:
        """Returns target waypoint on centerline lookahead_dist ahead of vehicle."""
        cx, cy, heading, dist, nearest_idx = self.find_nearest_track_point(car_x, car_y)
        track_len = len(self.center_line)
        target_idx = nearest_idx
        for offset in range(1, track_len):
            idx = (nearest_idx + offset) % track_len
            wx, wy, _ = self.center_line[idx]
            if math.hypot(wx - car_x, wy - car_y) >= lookahead_dist:
                target_idx = idx
                break
        return self.center_line[target_idx]

    def get_lane_perception(self, car_x: float, car_y: float, car_yaw: float) -> LaneDetectionResult:
        """Simulates camera lane detector with signed CTE (+ = right) and heading error."""
        cx, cy, track_heading, dist, idx = self.find_nearest_track_point(car_x, car_y)

        dx = car_x - cx
        dy = car_y - cy
        tangent_x = math.cos(track_heading)
        tangent_y = math.sin(track_heading)

        # Signed lateral error (+ = car is right of centerline, - = car is left)
        signed_cte = dx * tangent_y - dy * tangent_x
        heading_err = normalize_angle(track_heading - car_yaw)

        # Estimate curvature
        next_idx = (idx + 6) % len(self.center_line)
        next_heading = self.center_line[next_idx][2]
        delta_heading = abs(normalize_angle(next_heading - track_heading))
        curvature_radius = self.r if delta_heading > 0.04 else float('inf')

        return LaneDetectionResult(
            detected=True,
            cross_track_error_m=signed_cte,
            heading_error_rad=heading_err,
            curvature_radius_m=curvature_radius,
            confidence=0.96,
            lane_center_px=320.0 - signed_cte * 400.0,
            car_center_px=320.0,
            message="Tracking centerline",
        )

    def get_object_detections(self, car_x: float, car_y: float, car_yaw: float) -> List[DetectedObject]:
        """
        Simulates camera + YOLO detection for:
        1. Static road obstacles (cones, boxes)
        2. Moving / stalled NPC traffic cars
        3. Crossing pedestrians
        """
        detections: List[DetectedObject] = []
        cos_yaw = math.cos(car_yaw)
        sin_yaw = math.sin(car_yaw)

        # 1. Static Obstacles
        for obs in self.obstacles:
            dx = obs.x - car_x
            dy = obs.y - car_y
            forward_dist = dx * cos_yaw + dy * sin_yaw
            lateral_dist = -dx * sin_yaw + dy * cos_yaw

            if 0.10 < forward_dist < 3.5 and abs(lateral_dist) < forward_dist * 0.85:
                in_path = abs(lateral_dist) <= (self.lane_width / 2.0 + 0.08)
                detections.append(DetectedObject(
                    class_id=1,
                    class_name=obs.class_name,
                    confidence=0.95,
                    bbox=BoundingBox(
                        xmin=320 + lateral_dist * 100 - 25,
                        ymin=240,
                        xmax=320 + lateral_dist * 100 + 25,
                        ymax=340,
                    ),
                    distance_m=forward_dist,
                    lateral_offset_m=lateral_dist,
                    is_in_vehicle_path=in_path,
                ))

        # 2. Dynamic Traffic Cars (Lead Vehicles)
        for tcar in self.traffic_cars:
            dx = tcar.x - car_x
            dy = tcar.y - car_y
            forward_dist = dx * cos_yaw + dy * sin_yaw
            lateral_dist = -dx * sin_yaw + dy * cos_yaw

            if 0.15 < forward_dist < 4.0 and abs(lateral_dist) < forward_dist * 0.90:
                in_path = abs(lateral_dist) <= (self.lane_width / 2.0 + 0.12)
                name = "car" if not tcar.is_stalled else "stalled_car"
                detections.append(DetectedObject(
                    class_id=2,
                    class_name=name,
                    confidence=0.98,
                    bbox=BoundingBox(
                        xmin=320 + lateral_dist * 100 - 35,
                        ymin=230,
                        xmax=320 + lateral_dist * 100 + 35,
                        ymax=350,
                    ),
                    distance_m=forward_dist,
                    lateral_offset_m=lateral_dist,
                    is_in_vehicle_path=in_path,
                ))

        # 3. Dynamic Pedestrians
        for ped in self.pedestrians:
            if not ped.is_active:
                continue
            dx = ped.x - car_x
            dy = ped.y - car_y
            forward_dist = dx * cos_yaw + dy * sin_yaw
            lateral_dist = -dx * sin_yaw + dy * cos_yaw

            if 0.10 < forward_dist < 3.5 and abs(lateral_dist) < forward_dist * 0.85:
                # In path if on the road surface
                in_path = abs(ped.walk_progress) < 0.70 and abs(lateral_dist) <= (self.lane_width / 2.0 + 0.10)
                detections.append(DetectedObject(
                    class_id=3,
                    class_name="pedestrian",
                    confidence=0.96,
                    bbox=BoundingBox(
                        xmin=320 + lateral_dist * 100 - 20,
                        ymin=200,
                        xmax=320 + lateral_dist * 100 + 20,
                        ymax=340,
                    ),
                    distance_m=forward_dist,
                    lateral_offset_m=lateral_dist,
                    is_in_vehicle_path=in_path,
                ))

        detections.sort(key=lambda d: d.distance_m)
        return detections

    def get_forward_collision_distance(self, car_x: float, car_y: float, car_yaw: float) -> float:
        """
        Specifically checks for direct head-on collision objects in the narrow
        bumper path (|lateral| < 0.16m, forward < 2.5m).
        Does NOT trigger on objects that are safely alongside the car.
        """
        cos_yaw = math.cos(car_yaw)
        sin_yaw = math.sin(car_yaw)
        min_dist = 999.0

        all_obstacles = [(o.x, o.y, o.radius) for o in self.obstacles]
        for c in self.traffic_cars:
            all_obstacles.append((c.x, c.y, 0.20))
        for p in self.pedestrians:
            if p.is_active:
                all_obstacles.append((p.x, p.y, 0.15))

        for ox, oy, rad in all_obstacles:
            dx = ox - car_x
            dy = oy - car_y
            fwd = dx * cos_yaw + dy * sin_yaw
            lat = -dx * sin_yaw + dy * cos_yaw

            # Direct head-on collision corridor only:
            # If vehicle is already bypassing (e.g. |lat| > 0.20m), it will NOT hit it!
            if 0.05 < fwd < 2.5 and abs(lat) < (0.16 + rad * 0.5):
                min_dist = min(min_dist, fwd - rad)

        return min_dist

    def get_traffic_light_state(self, car_x: float, car_y: float, car_yaw: float) -> Tuple[TrafficLightState, float]:
        """Checks if vehicle is approaching any traffic light."""
        cos_yaw = math.cos(car_yaw)
        sin_yaw = math.sin(car_yaw)

        for light in self.traffic_lights:
            dx = light.x - car_x
            dy = light.y - car_y
            forward_dist = dx * cos_yaw + dy * sin_yaw
            lateral_dist = -dx * sin_yaw + dy * cos_yaw

            if 0.1 < forward_dist < 2.5 and abs(lateral_dist) < 1.2:
                return light.state, forward_dist

        return TrafficLightState.UNKNOWN, 999.0

    def raycast_lidar(
        self,
        car_x: float,
        car_y: float,
        car_yaw: float,
        num_beams: int = 16,
        max_range: float = 3.0,
    ) -> List[Tuple[float, float, float, float]]:
        """Simulates 2D LiDAR raycasting against all static and dynamic world objects."""
        beams = []
        fov = math.radians(120.0)
        start_angle = car_yaw - fov / 2.0

        all_targets = [(o.x, o.y, o.radius) for o in self.obstacles]
        for c in self.traffic_cars:
            all_targets.append((c.x, c.y, 0.22))
        for p in self.pedestrians:
            if p.is_active:
                all_targets.append((p.x, p.y, 0.16))

        for b in range(num_beams):
            angle = start_angle + (b / float(num_beams - 1)) * fov
            ray_dx = math.cos(angle)
            ray_dy = math.sin(angle)
            min_hit_dist = max_range

            for ox, oy, radius in all_targets:
                cx = ox - car_x
                cy = oy - car_y
                proj = cx * ray_dx + cy * ray_dy
                if proj > 0:
                    perp_sq = (cx * cx + cy * cy) - proj * proj
                    if perp_sq <= radius * radius:
                        dist = proj - math.sqrt(max(0.0, radius * radius - perp_sq))
                        if 0 < dist < min_hit_dist:
                            min_hit_dist = dist

            hit_x = car_x + min_hit_dist * ray_dx
            hit_y = car_y + min_hit_dist * ray_dy
            beams.append((angle, min_hit_dist, hit_x, hit_y))

        return beams

# Calibrated coordinate sign conventions for sub-4cm CTE tracking

# DynamicTrafficCar actor definition with longitudinal tracking

# DynamicPedestrian crosswalk actor
