"""Unit tests for Path Planning and Obstacle Avoidance."""

import unittest
from src.navigation.path_planner import GridAStar
from src.navigation.obstacle_avoidance import ObstacleAvoidancePlanner


class TestPathPlanning(unittest.TestCase):

    def test_grid_astar_clear_path(self):
        planner = GridAStar(width_m=4.0, height_m=4.0, resolution_m=0.2)
        start = (0.5, 0.5)
        goal = (3.5, 3.5)
        path = planner.plan(start, goal)
        self.assertGreater(len(path), 0)
        self.assertAlmostEqual(path[0][0], start[0], delta=0.2)
        self.assertAlmostEqual(path[-1][0], goal[0], delta=0.2)

    def test_grid_astar_obstacle_avoidance(self):
        planner = GridAStar(width_m=5.0, height_m=5.0, resolution_m=0.1)
        # Block direct path at (2.5, 2.5)
        planner.set_obstacle(x=2.5, y=2.5, radius_m=0.3)
        start = (1.0, 2.5)
        goal = (4.0, 2.5)
        path = planner.plan(start, goal)
        self.assertGreater(len(path), 0)

        # None of the waypoints should collide with obstacle
        for px, py in path:
            dist = ((px - 2.5) ** 2 + (py - 2.5) ** 2) ** 0.5
            # Margin check
            self.assertGreater(dist, 0.25)

    def test_obstacle_avoidance_profile(self):
        avoidance = ObstacleAvoidancePlanner(clearance_margin_m=0.30, maneuver_distance_m=1.0)
        avoidance.trigger_avoidance(obstacle_distance_m=1.0, obstacle_lateral_pos_m=0.1)
        self.assertTrue(avoidance.active_maneuver)

        # Step halfway
        prof = avoidance.step(distance_increment_m=0.5)
        self.assertTrue(prof.active)
        self.assertAlmostEqual(prof.progress, 0.5, delta=0.05)
        self.assertNotEqual(prof.lateral_shift_m, 0.0)

        # Step through completion
        prof_end = avoidance.step(distance_increment_m=0.6)
        self.assertFalse(prof_end.active)


if __name__ == "__main__":
    unittest.main()
