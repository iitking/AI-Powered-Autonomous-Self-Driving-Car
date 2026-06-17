"""
Model Predictive Control (MPC) for Autonomous Trajectory Tracking.
Formulates a receding horizon kinematic bicycle model optimization.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple
from src.config import VehicleGeometry
from src.utils.math_utils import clamp, normalize_angle


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0


class MPCController:
    """
    Receding horizon controller for kinematic bicycle model:
    x_{k+1} = x_k + v_k * cos(yaw_k) * dt
    y_{k+1} = y_k + v_k * sin(yaw_k) * dt
    yaw_{k+1} = yaw_k + (v_k / L) * tan(delta_k) * dt
    v_{k+1} = v_k + a_k * dt
    """

    def __init__(self, vehicle_geom: VehicleGeometry, horizon_steps: int = 10, dt: float = 0.05):
        self.geom = vehicle_geom
        self.N = horizon_steps
        self.dt = dt

        # Weights
        self.w_cte = 2.0
        self.w_heading = 1.5
        self.w_speed = 1.0
        self.w_steer_effort = 0.1
        self.w_steer_rate = 0.5

    def predict_state(self, state: VehicleState, steer_rad: float, accel: float) -> VehicleState:
        """Forward simulates vehicle kinematic bicycle model by dt."""
        nx = state.x + state.v * math.cos(state.yaw) * self.dt
        ny = state.y + state.v * math.sin(state.yaw) * self.dt
        nyaw = state.yaw + (state.v / self.geom.wheelbase) * math.tan(steer_rad) * self.dt
        nv = clamp(state.v + accel * self.dt, 0.0, self.geom.max_speed)
        return VehicleState(x=nx, y=ny, yaw=normalize_angle(nyaw), v=nv)

    def optimize(
        self,
        current_state: VehicleState,
        target_waypoints: List[Tuple[float, float, float]],  # (x, y, v_target)
    ) -> Tuple[float, float]:
        """
        Evaluates candidate steering commands and selects the trajectory minimizing cost.
        Returns: (optimal_steering_deg, optimal_accel)
        """
        if not target_waypoints:
            return 0.0, 0.0

        best_cost = float('inf')
        best_steer_deg = 0.0
        best_accel = 0.0

        # Discretized candidate action space
        candidate_steers = [-25.0, -15.0, -8.0, -3.0, 0.0, 3.0, 8.0, 15.0, 25.0]
        candidate_accels = [-1.0, -0.5, 0.0, 0.5, 1.0]

        for steer_deg in candidate_steers:
            steer_rad = math.radians(steer_deg)
            for accel in candidate_accels:
                cost = 0.0
                sim_state = VehicleState(
                    x=current_state.x,
                    y=current_state.y,
                    yaw=current_state.yaw,
                    v=current_state.v,
                )

                for step in range(min(self.N, len(target_waypoints))):
                    sim_state = self.predict_state(sim_state, steer_rad, accel)
                    tx, ty, tv = target_waypoints[step]

                    # Tracking errors
                    dx = sim_state.x - tx
                    dy = sim_state.y - ty
                    pos_err_sq = dx * dx + dy * dy
                    speed_err_sq = (sim_state.v - tv) ** 2

                    cost += self.w_cte * pos_err_sq + self.w_speed * speed_err_sq
                    cost += self.w_steer_effort * (steer_rad ** 2)

                if cost < best_cost:
                    best_cost = cost
                    best_steer_deg = steer_deg
                    best_accel = accel

        return best_steer_deg, best_accel
