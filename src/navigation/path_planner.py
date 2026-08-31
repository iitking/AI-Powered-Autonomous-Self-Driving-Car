"""
Path Planning Algorithms for Autonomous Navigation.
Implements 2D Grid-based A* Search with obstacle inflation and path smoothing.
"""

import heapq
import math
from typing import List, Optional, Set, Tuple


class GridAStar:
    """
    Grid-based A* Path Planner.
    Finds shortest collision-free path between (start_x, start_y) and (goal_x, goal_y).
    """

    def __init__(
        self,
        width_m: float,
        height_m: float,
        resolution_m: float = 0.10,
        robot_radius_m: float = 0.20,
    ):
        self.width_m = width_m
        self.height_m = height_m
        self.res = resolution_m
        self.robot_radius = robot_radius_m

        self.cols = int(math.ceil(width_m / self.res))
        self.rows = int(math.ceil(height_m / self.res))

        # Occupancy grid (0 = free, 1 = obstacle)
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Converts metric coordinate to grid cell indices (col, row)."""
        c = int(math.floor(x / self.res))
        r = int(math.floor(y / self.res))
        return (
            max(0, min(self.cols - 1, c)),
            max(0, min(self.rows - 1, r)),
        )

    def to_metric(self, c: int, r: int) -> Tuple[float, float]:
        """Converts grid cell indices to metric center coordinate."""
        return (
            (c + 0.5) * self.res,
            (r + 0.5) * self.res,
        )

    def set_obstacle(self, x: float, y: float, radius_m: float = 0.20):
        """Marks obstacle on grid with robot inflation radius."""
        inflation = radius_m + self.robot_radius
        c_obs, r_obs = self.to_grid(x, y)
        inflation_cells = int(math.ceil(inflation / self.res))

        for r in range(max(0, r_obs - inflation_cells), min(self.rows, r_obs + inflation_cells + 1)):
            for c in range(max(0, c_obs - inflation_cells), min(self.cols, c_obs + inflation_cells + 1)):
                mx, my = self.to_metric(c, r)
                dist = math.hypot(mx - x, my - y)
                if dist <= inflation:
                    self.grid[r][c] = 1

    def clear(self):
        """Resets grid to completely free space."""
        for r in range(self.rows):
            for c in range(self.cols):
                self.grid[r][c] = 0

    def plan(
        self,
        start_pos: Tuple[float, float],
        goal_pos: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        """
        Finds path from start to goal.
        Returns list of (x, y) metric coordinates.
        """
        sc, sr = self.to_grid(start_pos[0], start_pos[1])
        gc, gr = self.to_grid(goal_pos[0], goal_pos[1])

        if self.grid[sr][sc] == 1 or self.grid[gr][gc] == 1:
            # Force start/goal to free if car is initially touching inflated boundary
            self.grid[sr][sc] = 0
            self.grid[gr][gc] = 0

        # 8-connected neighbors: (dx, dy, cost)
        directions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414),
        ]

        # Priority queue stores: (f_score, g_score, (c, r))
        open_set: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, 0.0, (sc, sr)))

        came_from: dict = {}
        g_scores: dict = {(sc, sr): 0.0}
        closed_set: Set[Tuple[int, int]] = set()

        def heuristic(c1: int, r1: int, c2: int, r2: int) -> float:
            # Euclidean distance heuristic
            return math.hypot(c1 - c2, r1 - r2)

        while open_set:
            _, g, current = heapq.heappop(open_set)
            if current in closed_set:
                continue
            closed_set.add(current)

            if current == (gc, gr):
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()

                # Convert to metric coordinates and smooth
                metric_path = [self.to_metric(c, r) for c, r in path]
                return self.smooth_path(metric_path)

            cur_c, cur_r = current

            for dc, dr, move_cost in directions:
                nc, nr = cur_c + dc, cur_r + dr

                if 0 <= nc < self.cols and 0 <= nr < self.rows:
                    if self.grid[nr][nc] == 1:
                        continue  # Obstacle

                    tentative_g = g + move_cost
                    neighbor = (nc, nr)

                    if tentative_g < g_scores.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_scores[neighbor] = tentative_g
                        h = heuristic(nc, nr, gc, gr)
                        heapq.heappush(open_set, (tentative_g + h, tentative_g, neighbor))

        return []  # No path found

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Removes unnecessary collinear/redundant waypoints."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            furthest = i + 1
            for j in range(len(path) - 1, i + 1, -1):
                if self.is_line_of_sight(path[i], path[j]):
                    furthest = j
                    break
            smoothed.append(path[furthest])
            i = furthest

        return smoothed

    def is_line_of_sight(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """Raycasts between two points to verify unobstructed line of sight."""
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = int(dist / (self.res * 0.5)) + 1
        for s in range(steps + 1):
            t = s / max(steps, 1)
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            c, r = self.to_grid(x, y)
            if self.grid[r][c] == 1:
                return False
        return True

# Spline knot continuity constraints
