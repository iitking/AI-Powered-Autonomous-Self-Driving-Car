"""Unit tests for Extended Kalman Filter Sensor Fusion."""

import unittest
import math
from src.sensors.sensor_fusion import SensorFusionEKF


class TestSensorFusion(unittest.TestCase):

    def test_ekf_prediction(self):
        ekf = SensorFusionEKF(initial_x=0.0, initial_y=0.0, initial_yaw=0.0)
        # Give initial speed via measurement
        ekf.update_encoder(1.0)
        self.assertAlmostEqual(ekf.v, 1.0, delta=0.5)

        # Predict forward along X axis
        ekf.predict(accel_x=0.0, gyro_gz=0.0, dt=1.0)
        self.assertGreater(ekf.x, 0.5)
        self.assertAlmostEqual(ekf.y, 0.0, delta=0.1)

    def test_ekf_yaw_update(self):
        ekf = SensorFusionEKF()
        ekf.update_imu_heading(math.pi / 2.0)
        # Heading should move towards pi/2
        self.assertGreater(ekf.yaw, 0.1)


if __name__ == "__main__":
    unittest.main()
