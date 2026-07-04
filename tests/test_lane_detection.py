"""Unit tests for Lane Detection and Math Utilities."""

import unittest
import math
from src.utils.math_utils import fit_quadratic, calculate_curvature, clamp, normalize_angle, Vector2D
from src.perception.lane_detector import LaneDetector, LaneDetectionResult


class TestLaneDetection(unittest.TestCase):

    def test_vector2d_operations(self):
        v1 = Vector2D(3.0, 4.0)
        self.assertAlmostEqual(v1.magnitude(), 5.0)

        v2 = Vector2D(1.0, 2.0)
        v3 = v1 + v2
        self.assertEqual((v3.x, v3.y), (4.0, 6.0))

        v_rot = Vector2D(1.0, 0.0).rotate(math.pi / 2.0)
        self.assertAlmostEqual(v_rot.x, 0.0, places=5)
        self.assertAlmostEqual(v_rot.y, 1.0, places=5)

    def test_angle_normalization(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi, places=5)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi, places=5)

    def test_quadratic_fitting(self):
        # Parabola x = 2*y^2 + 3*y + 4
        y_vals = [0.0, 1.0, 2.0, 3.0, 4.0]
        x_vals = [2.0 * y**2 + 3.0 * y + 4.0 for y in y_vals]
        a, b, c = fit_quadratic(x_vals, y_vals)
        self.assertAlmostEqual(a, 2.0, places=2)
        self.assertAlmostEqual(b, 3.0, places=2)
        self.assertAlmostEqual(c, 4.0, places=2)

    def test_curvature_straight_line(self):
        # Straight line: a = 0 -> Curvature radius = inf
        rad = calculate_curvature(0.0, 1.0, 100.0)
        self.assertTrue(math.isinf(rad))

    def test_lane_detector_fallback(self):
        detector = LaneDetector()
        synthetic_frame = {"cross_track_error": -0.08, "heading_error": 0.02, "curvature": 5.0}
        res = detector.process_frame(synthetic_frame)
        self.assertTrue(res.detected)
        self.assertAlmostEqual(res.cross_track_error_m, -0.08)
        self.assertAlmostEqual(res.heading_error_rad, 0.02)


if __name__ == "__main__":
    unittest.main()
