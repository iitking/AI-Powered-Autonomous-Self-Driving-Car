"""Unit tests for Obstacle Overtaking, Dynamic Traffic ACC, and Pedestrian Yielding."""

import math
import unittest
from src.config import DEFAULT_CONFIG
from src.decision.state_machine import DrivingState
from src.simulation.simulator import AutonomousCarSimulator


class TestTrafficAndOvertake(unittest.TestCase):

    def test_smooth_obstacle_bypass_without_freezing(self):
        """Verifies car proactively maneuvers around obstacle without slamming emergency brake."""
        sim = AutonomousCarSimulator(DEFAULT_CONFIG)
        sim.clear_obstacles()
        sim.world.traffic_cars.clear()
        sim.world.traffic_lights[0].state = "GREEN"
        sim.world.traffic_lights[0].cycle_time_s = 9999.0

        # Place obstacle directly on ego centerline straight
        sim.add_obstacle(x=0.5, y=2.2, class_name="box")

        overtook = False
        min_speed = 999.0

        for step in range(160):
            sim.step(dt=0.05)
            v = sim.vehicle.state.velocity
            if step > 25:
                min_speed = min(min_speed, v)
            if sim.vehicle.state.x > 0.6:
                overtook = True
                break

        self.assertTrue(overtook, "Car failed to bypass obstacle!")
        self.assertGreater(min_speed, 0.40, f"Car slowed down excessively ({min_speed:.2f} m/s)!")

    def test_lead_car_adaptive_cruise_control(self):
        """Verifies car matches speed of moving lead vehicle."""
        sim = AutonomousCarSimulator(DEFAULT_CONFIG)
        sim.clear_obstacles()
        sim.world.traffic_lights[0].state = "GREEN"
        sim.world.traffic_lights[0].cycle_time_s = 9999.0

        # Run 40 steps behind lead vehicle
        acc_active = False
        for _ in range(40):
            sim.step(dt=0.05)
            if "ACC" in sim.latest_decision.maneuver:
                acc_active = True

        self.assertTrue(acc_active, "ACC was not activated behind lead vehicle!")
        self.assertAlmostEqual(sim.vehicle.state.velocity, 0.85, delta=0.25)

    def test_stalled_vehicle_overtake(self):
        """Verifies ego overtakes lead car when it stalls."""
        sim = AutonomousCarSimulator(DEFAULT_CONFIG)
        sim.clear_obstacles()
        sim.world.traffic_lights[0].state = "GREEN"
        sim.world.traffic_lights[0].cycle_time_s = 9999.0

        sim.stall_traffic_car()
        tc = sim.world.traffic_cars[0]
        stalled_x = tc.x

        overtook = False
        for _ in range(130):
            sim.step(dt=0.05)
            if sim.vehicle.state.x > stalled_x + 0.3:
                overtook = True
                break

        self.assertTrue(overtook, "Failed to overtake stalled lead traffic vehicle!")

    def test_pedestrian_crosswalk_yield_and_continue(self):
        """Verifies ego stops before crosswalk for pedestrian and resumes when cleared."""
        sim = AutonomousCarSimulator(DEFAULT_CONFIG)
        sim.clear_obstacles()
        sim.world.traffic_cars.clear()
        sim.world.traffic_lights[0].state = "GREEN"
        sim.world.traffic_lights[0].cycle_time_s = 9999.0

        # Position car before crosswalk on bottom straight
        sim.vehicle.reset(x=0.8, y=-sim.world.r, yaw=math.pi, v=1.0)
        sim.trigger_pedestrian()

        yielded = False
        resumed = False

        for _ in range(100):
            sim.step(dt=0.05)
            if "PEDESTRIAN" in sim.latest_decision.maneuver or sim.latest_decision.driving_state == DrivingState.STOPPED_AT_SIGN:
                yielded = True
            if yielded and sim.vehicle.state.velocity > 0.5:
                resumed = True
                break

        self.assertTrue(yielded, "Failed to yield to crossing pedestrian!")
        self.assertTrue(resumed, "Failed to resume driving after pedestrian cleared!")


if __name__ == "__main__":
    unittest.main()
