"""
Math and Geometry Utilities for Autonomous Vehicle Systems.
Provides Vector2D, Matrix operations, coordinate transforms, polynomial fitting,
and angle normalization with pure-Python fallbacks.
"""

import math
from typing import List, Tuple, Union

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    np = None


def clamp(val: float, min_val: float, max_val: float) -> float:
    """Clamps a value within [min_val, max_val]."""
    return max(min_val, min(max_val, val))


def normalize_angle(angle_rad: float) -> float:
    """Normalizes an angle to the range [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def deg2rad(deg: float) -> float:
    """Converts degrees to radians."""
    return deg * (math.pi / 180.0)


def rad2deg(rad: float) -> float:
    """Converts radians to degrees."""
    return rad * (180.0 / math.pi)


class Vector2D:
    """2D Vector representation with common vector operations."""
    __slots__ = ('x', 'y')

    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = float(x)
        self.y = float(y)

    def __repr__(self) -> str:
        return f"Vector2D(x={self.x:.3f}, y={self.y:.3f})"

    def __add__(self, other: 'Vector2D') -> 'Vector2D':
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector2D') -> 'Vector2D':
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vector2D':
        return Vector2D(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> 'Vector2D':
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> 'Vector2D':
        if abs(scalar) < 1e-9:
            raise ZeroDivisionError("Division by near-zero scalar")
        return Vector2D(self.x / scalar, self.y / scalar)

    def dot(self, other: 'Vector2D') -> float:
        """Returns the dot product of two vectors."""
        return self.x * other.x + self.y * other.y

    def magnitude(self) -> float:
        """Returns Euclidean length (norm) of vector."""
        return math.hypot(self.x, self.y)

    def magnitude_squared(self) -> float:
        """Returns squared magnitude (avoids square root)."""
        return self.x * self.x + self.y * self.y

    def normalized(self) -> 'Vector2D':
        """Returns unit vector in the same direction."""
        mag = self.magnitude()
        if mag < 1e-9:
            return Vector2D(0.0, 0.0)
        return self / mag

    def distance_to(self, other: 'Vector2D') -> float:
        """Euclidean distance between two vectors."""
        return math.hypot(self.x - other.x, self.y - other.y)

    def heading(self) -> float:
        """Returns the orientation angle in radians [-pi, pi]."""
        return math.atan2(self.y, self.x)

    def rotate(self, angle_rad: float) -> 'Vector2D':
        """Rotates the vector by angle_rad counter-clockwise."""
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return Vector2D(self.x * c - self.y * s, self.x * s + self.y * c)

    def to_tuple(self) -> Tuple[float, float]:
        """Returns (x, y) tuple."""
        return (self.x, self.y)


def fit_quadratic(x_coords: List[float], y_coords: List[float]) -> Tuple[float, float, float]:
    """
    Fits a quadratic curve x = a*y^2 + b*y + c using Least Squares.
    Returns (a, b, c).
    Supports pure Python (Gaussian elimination) or NumPy if available.
    """
    if len(x_coords) < 3 or len(y_coords) < 3:
        # Default straight line along y
        return (0.0, 0.0, sum(x_coords) / max(len(x_coords), 1))

    if HAS_NUMPY:
        try:
            poly = np.polyfit(y_coords, x_coords, 2)
            return (float(poly[0]), float(poly[1]), float(poly[2]))
        except Exception:
            pass

    # Pure Python least-squares solution for 2nd order polynomial
    # Normal equations: M * [a, b, c]^T = R
    n = len(y_coords)
    sy4 = sum(y ** 4 for y in y_coords)
    sy3 = sum(y ** 3 for y in y_coords)
    sy2 = sum(y ** 2 for y in y_coords)
    sy1 = sum(y for y in y_coords)
    sy0 = float(n)

    s_xy2 = sum(x * (y ** 2) for x, y in zip(x_coords, y_coords))
    s_xy1 = sum(x * y for x, y in zip(x_coords, y_coords))
    s_xy0 = sum(x_coords)

    # 3x3 linear system:
    # [ sy4  sy3  sy2 ] [ a ]   [ s_xy2 ]
    # [ sy3  sy2  sy1 ] [ b ] = [ s_xy1 ]
    # [ sy2  sy1  sy0 ] [ c ]   [ s_xy0 ]

    A = [
        [sy4, sy3, sy2],
        [sy3, sy2, sy1],
        [sy2, sy1, sy0]
    ]
    B = [s_xy2, s_xy1, s_xy0]

    # Cramer's Rule or Gaussian elimination for 3x3:
    def det3(m: List[List[float]]) -> float:
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    det_A = det3(A)
    if abs(det_A) < 1e-12:
        return (0.0, 0.0, s_xy0 / max(sy0, 1.0))

    # Replace column 0 with B
    A0 = [[B[r], A[r][1], A[r][2]] for r in range(3)]
    # Replace column 1 with B
    A1 = [[A[r][0], B[r], A[r][2]] for r in range(3)]
    # Replace column 2 with B
    A2 = [[A[r][0], A[r][1], B[r]] for r in range(3)]

    a = det3(A0) / det_A
    b = det3(A1) / det_A
    c = det3(A2) / det_A
    return (a, b, c)


def calculate_curvature(a: float, b: float, y_eval: float) -> float:
    """
    Calculates radius of curvature for curve x = a*y^2 + b*y + c at y_eval:
    R = [1 + (dx/dy)^2]^(3/2) / |d2x/dy2|
    where dx/dy = 2*a*y + b and d2x/dy2 = 2*a.
    Returns radius in meters/pixels. Returns float('inf') for straight lines.
    """
    dx_dy = 2.0 * a * y_eval + b
    d2x_dy2 = 2.0 * a

    if abs(d2x_dy2) < 1e-6:
        return float('inf')

    radius = ((1.0 + dx_dy ** 2) ** 1.5) / abs(d2x_dy2)
    return radius


class LowPassFilter:
    """First-order discrete low-pass filter."""

    def __init__(self, alpha: float = 0.2, initial_value: float = 0.0):
        self.alpha = clamp(alpha, 0.0, 1.0)
        self.value = initial_value

    def update(self, measurement: float) -> float:
        self.value = self.alpha * measurement + (1.0 - self.alpha) * self.value
        return self.value

    def reset(self, value: float = 0.0):
        self.value = value
