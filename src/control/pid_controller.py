"""
Robust PID Controller with Anti-Windup and Derivative Low-Pass Filter.
Applicable for both steering angle and motor velocity control.
"""

from dataclasses import dataclass
from typing import Optional
from src.utils.math_utils import clamp


@dataclass
class PIDTelemetry:
    """Detailed state information for controller debugging and telemetry."""
    target: float = 0.0
    current: float = 0.0
    error: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    output: float = 0.0
    is_saturated: bool = False


class PIDController:
    """
    Standard PID Controller with:
    - Proportional, Integral, Derivative control
    - Integrator anti-windup clamping
    - First-order derivative low-pass filter (noise suppression)
    - Output saturation limits
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limits: tuple = (-1.0, 1.0),
        integral_limit: Optional[float] = None,
        filter_alpha: float = 0.25,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.min_out, self.max_out = output_limits
        self.integral_limit = integral_limit or (self.max_out - self.min_out) * 0.5
        self.filter_alpha = clamp(filter_alpha, 0.01, 1.0)

        # State variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.last_telemetry = PIDTelemetry()

    def update(self, target: float, current: float, dt: float) -> float:
        """
        Calculates control effort given setpoint and current process value.
        dt must be positive in seconds.
        """
        if dt <= 1e-6:
            return self.last_telemetry.output

        error = target - current

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)
        i_term = self.ki * self.integral

        # Derivative term with low-pass filter
        raw_derivative = (error - self.prev_error) / dt
        self.filtered_derivative = (
            self.filter_alpha * raw_derivative + (1.0 - self.filter_alpha) * self.filtered_derivative
        )
        d_term = self.kd * self.filtered_derivative

        # Total control effort
        raw_output = p_term + i_term + d_term
        output = clamp(raw_output, self.min_out, self.max_out)
        is_saturated = (raw_output != output)

        # Save state
        self.prev_error = error
        self.last_telemetry = PIDTelemetry(
            target=target,
            current=current,
            error=error,
            p_term=p_term,
            i_term=i_term,
            d_term=d_term,
            output=output,
            is_saturated=is_saturated,
        )

        return output

    def reset(self):
        """Clears accumulated integral error and previous derivative history."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.last_telemetry = PIDTelemetry()

    def set_gains(self, kp: Optional[float] = None, ki: Optional[float] = None, kd: Optional[float] = None):
        """Dynamically tune gains during runtime."""
        if kp is not None:
            self.kp = float(kp)
        if ki is not None:
            self.ki = float(ki)
        if kd is not None:
            self.kd = float(kd)

# Anti-windup threshold tuned for steering servo
