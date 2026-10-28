"""A* on the copied warehouse map. No ROS."""

from pathlib import Path
import math
import unittest

import numpy as np
import yaml

from fleet_nav.map_grid import (
    OccupancyMap,
    hold_crossing_path,
    load_occupancy,
    peer_is_planning_obstacle,
    peer_motion_speed,
)


ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / "src" / "fleet_nav" / "maps" / "warehouse_map.yaml"
FLEET = ROOT / "src" / "fleet_nav" / "config" / "fleet.yaml"
BRIDGE = ROOT / "src" / "fleet_nav" / "config" / "bridge.yaml"


class WarehouseMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.occupancy = load_occupancy(MAP)

    def test_a_crossing_robot_is_not_a_wall_and_the_wait_keeps_the_path(self):
        # Twist left at zero, pose moving at cruise. That robot is crossing.
        speed = peer_motion_speed(0.0, 0.30 * 0.20, 0.20)
        self.assertGreater(speed, 0.20)
        self.assertFalse(peer_is_planning_obstacle(speed))
        # A robot that has not moved is still an obstacle for the plan.
        still = peer_motion_speed(0.0, 0.0, 0.20)
        self.assertTrue(peer_is_planning_obstacle(still))
        # One short sample keeps the twist. A jump is not a cruise.
        self.assertAlmostEqual(peer_motion_speed(0.0, 0.04, 0.10), 0.0)
        self.assertAlmostEqual(peer_motion_speed(0.0, 0.40, 0.20), 0.0)
        # The robot that stopped for the crossing keeps its path.
        self.assertTrue(hold_crossing_path(True, 0.0, -1.0))
        # Priority, a side pass, and a finished wait may still replan.
        self.assertFalse(hold_crossing_path(True, 0.0, 0.0))
        self.assertFalse(hold_crossing_path(True, 1.0, 0.0))
        self.assertFalse(hold_crossing_path(False, 0.0, -1.0))

    def test_a_ray_stops_on_the_shelf_the_image_left_open(self):
        # East of the south shelf, looking west. The image lets this ray
        # through the box; the face is at x=-1.5.
        origin_x = -0.35
        hit = self.occupancy.range_to_mapped(origin_x, -0.8, math.pi, samples=1)
        self.assertAlmostEqual(hit, origin_x - (-1.5), delta=0.05)
        # South of the north shelf, looking north. x=2 is outside the image,
        # and the box is unknown where the image does cover it.
        hit = self.occupancy.range_to_mapped(2.0, 0.0, math.pi / 2.0, samples=1)
        self.assertAlmostEqual(hit, 0.5, delta=0.05)

    def test_shelf_is_blocked_and_spawns_are_free(self):
        shelf = self.occupancy.world_to_cell(2.0, 2.0)
        self.assertEqual(int(self.occupancy.grid[shelf]), 1)
        with FLEET.open(encoding="utf-8") as handle:
            fleet = yaml.safe_load(handle)
        for name, spec in fleet["robots"].items():
            cell = self.occupancy.world_to_cell(float(spec["x"]), float(spec["y"]))
            self.assertEqual(int(self.occupancy.grid[cell]), 0, name)

    def test_each_demo_goal_has_a_path_from_its_nearest_spawn(self):
        with FLEET.open(encoding="utf-8") as handle:
            fleet = yaml.safe_load(handle)
        robots = {
            name: (float(spec["x"]), float(spec["y"])) for name, spec in fleet["robots"].items()
        }
        used = set()
        for raw in fleet["demo_goals"]:
            goal = (float(raw[0]), float(raw[1]))
            name = min(robots, key=lambda item: math.hypot(robots[item][0] - goal[0], robots[item][1] - goal[1]))
            self.assertNotIn(name, used)
            used.add(name)
            path = self.occupancy.plan(robots[name], goal)
            self.assertGreater(len(path), 1, f"no path {name} {robots[name]} -> {goal}")
            self.assertAlmostEqual(path[-1][0], goal[0], delta=self.occupancy.resolution)
            self.assertAlmostEqual(path[-1][1], goal[1], delta=self.occupancy.resolution)

    def test_a_pose_inside_the_inflation_can_still_reach_open_floor(self):
        # (-2.55, -2.0) sits in the inflated margin of the shelf. The goal is free.
        # Clearing two cells used to report no path from there.
        path = self.occupancy.plan((-2.55, -2.0), (-3.02, -2.67))
        self.assertGreater(len(path), 1)
        self.assertAlmostEqual(path[-1][0], -3.02, delta=0.6)
        self.assertAlmostEqual(path[-1][1], -2.67, delta=0.6)

    def test_orange_route_uses_the_gap_between_shelf_and_south_wall(self):
        path = self.occupancy.plan((0.6, -3.6), (-3.02, -2.67))
        self.assertGreater(len(path), 1)
        gap = [point for point in path if -2.6 <= point[0] <= -1.2]
        self.assertTrue(gap)
        self.assertLess(min(point[1] for point in gap), -4.0)
        # The centre line stays outside the chassis, including on the shortcut.
        for point in path:
            self.assertGreaterEqual(self.occupancy.clearance(*point), 0.25)
        flips = 0
        previous = None
        for index in range(len(path) - 1):
            heading = math.atan2(
                path[index + 1][1] - path[index][1],
                path[index + 1][0] - path[index][0],
            )
            if previous is not None:
                delta = abs(math.atan2(math.sin(heading - previous), math.cos(heading - previous)))
                if delta > 0.5:
                    flips += 1
            previous = heading
        self.assertLessEqual(flips, 12)
        self.assertLess(len(path), 80)
        straight = self.occupancy.plan((0.6, -3.6), (0.7, -2.4))
        self.assertGreater(min(point[1] for point in straight), -3.8)

    def test_lookahead_does_not_aim_through_a_wall(self):
        grid = np.zeros((30, 30), dtype=np.uint8)
        grid[10:20, 14] = 1
        blank = np.zeros_like(grid, dtype=np.int8)
        occupancy = OccupancyMap(grid, 1.0, 0.0, 0.0, blank, np.zeros_like(grid, dtype=np.float32))
        path = [(5.5, 15.5), (5.5, 5.5), (20.5, 5.5), (20.5, 15.5)]
        target = occupancy.visible_lookahead(path, 0, 5.5, 15.5, 40.0)
        self.assertEqual(target, (20.5, 5.5))
        self.assertTrue(occupancy.segment_free(5.5, 15.5, target[0], target[1]))
        self.assertFalse(occupancy.segment_free(5.5, 15.5, 20.5, 15.5))
        self.assertIsNone(occupancy.visible_lookahead([(20.5, 15.5)], 0, 5.5, 15.5, 40.0))

    def test_bridge_lists_every_robot(self):
        with FLEET.open(encoding="utf-8") as handle:
            fleet = yaml.safe_load(handle)
        with BRIDGE.open(encoding="utf-8") as handle:
            bridge = yaml.safe_load(handle)
        topics = {item["gz_topic_name"] for item in bridge}
        for name in fleet["robots"]:
            for suffix in ("cmd_vel", "odometry", "scan", "tf"):
                self.assertIn(f"/model/{name}/{suffix}", topics)


if __name__ == "__main__":
    unittest.main()
