"""Unit tests for PID Controller."""

import unittest
from src.control.pid_controller import PIDController


class TestPIDController(unittest.TestCase):

    def setUp(self):
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.05, output_limits=(-10.0, 10.0), integral_limit=5.0)

    def test_proportional_response(self):
        # Target = 5, Current = 0 -> Error = 5 -> P-term = 5.0
        out = self.pid.update(target=5.0, current=0.0, dt=0.01)
        self.assertAlmostEqual(self.pid.last_telemetry.p_term, 5.0, places=2)

    def test_anti_windup_clamping(self):
        # Apply persistent error over multiple steps
        for _ in range(100):
            self.pid.update(target=10.0, current=0.0, dt=0.1)

        # Integral must not exceed integral_limit (5.0)
        self.assertLessEqual(abs(self.pid.integral), 5.0 + 1e-6)

    def test_output_saturation_limits(self):
        # Huge error
        out = self.pid.update(target=1000.0, current=0.0, dt=0.05)
        self.assertEqual(out, 10.0)
        self.assertTrue(self.pid.last_telemetry.is_saturated)

    def test_negative_saturation(self):
        out = self.pid.update(target=-1000.0, current=0.0, dt=0.05)
        self.assertEqual(out, -10.0)

    def test_reset(self):
        self.pid.update(target=5.0, current=0.0, dt=0.1)
        self.assertNotEqual(self.pid.integral, 0.0)
        self.pid.reset()
        self.assertEqual(self.pid.integral, 0.0)
        self.assertEqual(self.pid.prev_error, 0.0)


if __name__ == "__main__":
    unittest.main()
