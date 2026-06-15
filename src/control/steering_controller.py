"""
Lateral Vehicle Steering Control.
Supports hybrid Lookahead Pure-Pursuit + Stanley + PID lateral controllers
with servo slew-rate limiting.
"""

import math
from typing import Optional, Tuple
from src.config import PIDGains, VehicleGeometry
from src.control.pid_controller import PIDController, PIDTelemetry
from src.utils.math_utils import clamp, normalize_angle, rad2deg, deg2rad


class SteeringController:
    """
    Computes front wheel steering angle from perception feedback.
    Features hybrid Pure-Pursuit lookahead tracking, PID cross-track feedback,
    and slew-rate limiting to protect physical steering servos.
    """

    def __init__(
        self,
        vehicle_geom: Optional[VehicleGeometry] = None,
        pid_gains: Optional[PIDGains] = None,
        controller_type: str = "hybrid",  # "hybrid", "pid", "stanley", or "pure_pursuit"
    ):
        self.geom = vehicle_geom or VehicleGeometry()
        self.gains = pid_gains or PIDGains()
        self.controller_type = controller_type

        # Steering PID controller on cross-track error
        # Limits allow up to max steering angle
        self.pid = PIDController(
            kp=self.gains.kp_steer,
            ki=self.gains.ki_steer,
            kd=self.gains.kd_steer,
            output_limits=(-self.geom.max_steering_angle, self.geom.max_steering_angle),
            integral_limit=self.gains.steer_anti_windup,
            filter_alpha=self.gains.steer_filter_alpha,
        )

        # Stanley parameters
        self.k_stanley = 1.8
        self.k_soft = 0.4

        # Slew rate limit (degrees per second)
        self.max_slew_rate = 140.0
        self.current_steering_deg = 0.0

    def compute_steering(
        self,
        cross_track_error_m: float,
        heading_error_rad: float,
        current_speed_mps: float,
        dt: float,
        lookahead_point: Optional[Tuple[float, float, float]] = None,
        car_pose: Optional[Tuple[float, float, float]] = None,
    ) -> Tuple[float, PIDTelemetry]:
        """
        Calculates commanded front wheel steering angle (in degrees).
        Positive angle = turn counter-clockwise (LEFT, +yaw rate)
        Negative angle = turn clockwise (RIGHT, -yaw rate)
        """
        raw_angle_deg = 0.0

        # PID Cross-Track Feedback:
        # If car is right of center (cte > 0), steer left (> 0)
        # Therefore error should be positive when cte is positive.
        pid_output = self.pid.update(
            target=0.0,
            current=-cross_track_error_m,
            dt=dt,
        )

        if lookahead_point is not None and car_pose is not None:
            # 1. Pure Pursuit Lookahead Target
            car_x, car_y, car_yaw = car_pose
            tx, ty, _ = lookahead_point
            dx = tx - car_x
            dy = ty - car_y

            # Transform into vehicle local coordinates
            # Forward = local_x, Left = local_y
            local_x = dx * math.cos(car_yaw) + dy * math.sin(car_yaw)
            local_y = -dx * math.sin(car_yaw) + dy * math.cos(car_yaw)

            d_sq = local_x * local_x + local_y * local_y
            kappa = (2.0 * local_y) / max(0.04, d_sq)
            pp_angle_rad = math.atan(self.geom.wheelbase * kappa)
            pp_angle_deg = rad2deg(pp_angle_rad)

            # Blend Pure Pursuit geometric curve tracking with fine-grained PID cross-track damping
            raw_angle_deg = pp_angle_deg + 0.35 * pid_output

        elif self.controller_type == "stanley":
            # Stanley controller: delta = heading_error + arctan(k * cte / (v + k_soft))
            v = max(0.1, abs(current_speed_mps))
            cross_track_term = math.atan2(self.k_stanley * cross_track_error_m, v + self.k_soft)
            delta_rad = normalize_angle(heading_error_rad + cross_track_term)
            raw_angle_deg = rad2deg(delta_rad)

        else:
            # Standard PID + Feedforward Heading Controller
            # heading_error_rad is target_heading - car_yaw (+ = road turns left, steer left)
            heading_ff_deg = rad2deg(heading_error_rad) * 0.75
            raw_angle_deg = heading_ff_deg + pid_output

        # Clamp to physical servo limit
        clamped_deg = clamp(raw_angle_deg, -self.geom.max_steering_angle, self.geom.max_steering_angle)

        # Slew rate limit to prevent servo jerk
        max_delta = self.max_slew_rate * dt
        diff = clamped_deg - self.current_steering_deg
        rate_limited_deg = self.current_steering_deg + clamp(diff, -max_delta, max_delta)
        self.current_steering_deg = rate_limited_deg

        # Build telemetry
        telemetry = PIDTelemetry(
            target=0.0,
            current=cross_track_error_m,
            error=cross_track_error_m,
            p_term=self.pid.last_telemetry.p_term,
            i_term=self.pid.last_telemetry.i_term,
            d_term=self.pid.last_telemetry.d_term,
            output=self.current_steering_deg,
            is_saturated=(abs(self.current_steering_deg) >= self.geom.max_steering_angle - 0.1),
        )

        return self.current_steering_deg, telemetry

    def reset(self):
        self.pid.reset()
        self.current_steering_deg = 0.0
