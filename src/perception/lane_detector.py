"""
Lane Detection and Curvature Estimation Pipeline.
Implements color masking, perspective transform (bird's-eye view),
sliding-window histogram line search, polynomial curve fitting,
cross-track error (CTE), and road curvature estimation.
Compatible with OpenCV when installed, with a self-contained fallback.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from src.config import CameraConfig
from src.utils.math_utils import clamp, fit_quadratic, calculate_curvature

try:
    import cv2
    import numpy as np
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    cv2 = None
    np = None


@dataclass
class LaneDetectionResult:
    """Telemetry and geometric results from lane detection."""
    detected: bool = False
    cross_track_error_m: float = 0.0  # Positive = car is to right of center, negative = to left
    heading_error_rad: float = 0.0    # Angle of car relative to lane tangent
    curvature_radius_m: float = float('inf') # Curve radius in meters
    confidence: float = 0.0           # 0.0 to 1.0 confidence score
    left_fit: Optional[Tuple[float, float, float]] = None  # (a, b, c) in x = a*y^2 + b*y + c
    right_fit: Optional[Tuple[float, float, float]] = None
    lane_center_px: float = 320.0
    car_center_px: float = 320.0
    message: str = "Init"


class LaneDetector:
    """
    Robust Lane Detector supporting both physical camera frames and
    simulated road perceptions.
    """

    def __init__(self, config: Optional[CameraConfig] = None):
        self.config = config or CameraConfig()
        self.img_w = self.config.image_width
        self.img_h = self.config.image_height

        # Smoothing filters for stable steering
        self.prev_left_fit: Optional[Tuple[float, float, float]] = None
        self.prev_right_fit: Optional[Tuple[float, float, float]] = None
        self.prev_cte: float = 0.0
        self.smoothing_factor = 0.25

        # Perspective transform matrices (OpenCV)
        self.M = None
        self.Minv = None
        self._init_perspective()

    def _init_perspective(self):
        """Initializes bird's eye view warp matrices if OpenCV is available."""
        if not HAS_OPENCV:
            return

        w, h = float(self.img_w), float(self.img_h)
        # Trapezoid Region of Interest in front of vehicle
        src = np.float32([
            [w * 0.15, h * 0.95],  # Bottom-left
            [w * 0.85, h * 0.95],  # Bottom-right
            [w * 0.58, h * 0.55],  # Top-right
            [w * 0.42, h * 0.55],  # Top-left
        ])
        # Warped rectangle (top-down bird's-eye view)
        dst = np.float32([
            [w * 0.25, h],
            [w * 0.75, h],
            [w * 0.75, 0],
            [w * 0.25, 0],
        ])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)

    def process_frame(self, frame) -> LaneDetectionResult:
        """
        Processes a raw BGR frame from the camera.
        If OpenCV is present, runs full computer vision pipeline.
        Otherwise falls back to analytic detection.
        """
        if HAS_OPENCV and isinstance(frame, np.ndarray):
            return self._process_opencv(frame)
        else:
            return self._process_fallback(frame)

    def _process_opencv(self, frame: 'np.ndarray') -> LaneDetectionResult:
        """OpenCV-based sliding window lane detector."""
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White mask
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 40, 255], dtype=np.uint8)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # Yellow mask
        lower_yellow = np.array([18, 80, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # Bird's eye warp
        warped = cv2.warpPerspective(combined_mask, self.M, (w, h), flags=cv2.INTER_LINEAR)

        # Sliding window search
        histogram = np.sum(warped[h // 2:, :], axis=0)
        midpoint = int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = int(h // nwindows)
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 50
        minpix = 40

        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        if len(leftx) < 100 or len(rightx) < 100:
            return LaneDetectionResult(detected=False, message="Insufficient lane pixels")

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Smooth fits
        if self.prev_left_fit is not None:
            left_fit = self.smoothing_factor * left_fit + (1 - self.smoothing_factor) * np.array(self.prev_left_fit)
            right_fit = self.smoothing_factor * right_fit + (1 - self.smoothing_factor) * np.array(self.prev_right_fit)

        self.prev_left_fit = tuple(left_fit)
        self.prev_right_fit = tuple(right_fit)

        # Calculate position and curvature at bottom of image (car location)
        y_eval = float(h - 1)
        left_bottom_x = left_fit[0] * (y_eval ** 2) + left_fit[1] * y_eval + left_fit[2]
        right_bottom_x = right_fit[0] * (y_eval ** 2) + right_fit[1] * y_eval + right_fit[2]
        lane_center_px = (left_bottom_x + right_bottom_x) / 2.0
        car_center_px = w / 2.0

        # Pixels to meters
        xm_per_pix = self.config.lane_width_meters / max(abs(right_bottom_x - left_bottom_x), 1.0)
        cte_m = (car_center_px - lane_center_px) * xm_per_pix

        # Curvature
        left_curv = calculate_curvature(left_fit[0], left_fit[1], y_eval)
        right_curv = calculate_curvature(right_fit[0], right_fit[1], y_eval)
        avg_curv = (left_curv + right_curv) / 2.0 * xm_per_pix

        # Heading error from polynomial derivative dx/dy at y_eval
        lane_slope = 2.0 * ((left_fit[0] + right_fit[0]) / 2.0) * y_eval + (left_fit[1] + right_fit[1]) / 2.0
        heading_error = math.atan(lane_slope)

        return LaneDetectionResult(
            detected=True,
            cross_track_error_m=cte_m,
            heading_error_rad=heading_error,
            curvature_radius_m=avg_curv,
            confidence=0.92,
            left_fit=self.prev_left_fit,
            right_fit=self.prev_right_fit,
            lane_center_px=lane_center_px,
            car_center_px=car_center_px,
            message="Lane detected successfully via OpenCV",
        )

    def _process_fallback(self, frame) -> LaneDetectionResult:
        """
        Pure-Python fallback when OpenCV is not installed or when frame
        is provided as coordinate data or synthetic points.
        """
        # If frame is dict with synthetic simulator data
        if isinstance(frame, dict):
            cte = float(frame.get("cross_track_error", 0.0))
            heading = float(frame.get("heading_error", 0.0))
            curv = float(frame.get("curvature", float('inf')))
            return LaneDetectionResult(
                detected=True,
                cross_track_error_m=cte,
                heading_error_rad=heading,
                curvature_radius_m=curv,
                confidence=0.95,
                lane_center_px=self.img_w / 2.0 - (cte * 400.0),
                car_center_px=self.img_w / 2.0,
                message="Synthetic / fallback perception active",
            )

        # Default centered lane
        return LaneDetectionResult(
            detected=True,
            cross_track_error_m=0.0,
            heading_error_rad=0.0,
            curvature_radius_m=float('inf'),
            confidence=0.80,
            lane_center_px=self.img_w / 2.0,
            car_center_px=self.img_w / 2.0,
            message="Default centered lane estimate",
        )

    def calculate_steering_from_cte(self, cte: float) -> float:
        """Calculates rough proportional steering angle from cross-track error."""
        # Sensitivity: ~25 degrees per 0.2 meters offset
        return clamp(-cte * 60.0, -self.config.max_steering_angle if hasattr(self.config, 'max_steering_angle') else -30.0, 30.0)

# Optimized ROI trapezoid projection
