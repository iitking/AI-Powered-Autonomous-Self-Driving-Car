"""Unit tests for Master Simulator and Vehicle Kinematics."""

import unittest
from src.config import DEFAULT_CONFIG
from src.simulation.simulator import AutonomousCarSimulator
from src.simulation.vehicle_model import KinematicBicycleModel


class TestSimulation(unittest.TestCase):

    def test_bicycle_kinematics(self):
        geom = DEFAULT_CONFIG.vehicle
        model = KinematicBicycleModel(geom)
        model.reset(x=0.0, y=0.0, yaw=0.0, v=0.0)

        # Accelerate forward
        state = model.step(target_steer_deg=0.0, throttle=0.8, brake=0.0, dt=0.1)
        self.assertGreater(state.velocity, 0.0)
        self.assertGreater(state.x, 0.0)
        self.assertAlmostEqual(state.y, 0.0, delta=0.01)

    def test_simulator_master_loop(self):
        sim = AutonomousCarSimulator(DEFAULT_CONFIG)
        sim.reset()

        # Step 20 iterations (1.0 second of simulation)
        for _ in range(20):
            sim.step(dt=0.05)

        state_dict = sim.get_full_state()
        self.assertIn("vehicle", state_dict)
        self.assertIn("control", state_dict)
        self.assertIn("perception", state_dict)
        self.assertIn("world", state_dict)
        self.assertEqual(state_dict["step"], 20)
        self.assertGreater(state_dict["vehicle"]["velocity_mps"], 0.1)


    def test_path_tracking_accuracy(self):
        """Verifies that autonomous vehicle tracks curves and keeps CTE < 0.10m."""
        sim = AutonomousCarSimulator(DEFAULT_CONFIG)
        sim.clear_obstacles()
        sim.world.traffic_lights[0].state = 'GREEN'
        sim.world.traffic_lights[0].cycle_time_s = 9999.0

        max_cte = 0.0
        # Run 300 steps through curve
        for _ in range(300):
            sim.step(dt=0.05)
            max_cte = max(max_cte, abs(sim.latest_lane.cross_track_error_m))

        self.assertLess(max_cte, 0.10, f"Max CTE ({max_cte:.3f}m) exceeded 0.10m threshold")


if __name__ == "__main__":
    unittest.main()
