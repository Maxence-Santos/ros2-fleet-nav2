"""Walker routes stay in the open aisles. No ROS, no Gazebo."""

import math
import unittest
from pathlib import Path

import yaml

from fleet_nav.walkers import PEOPLE, linear_speed, pose_at


ROOT = Path(__file__).resolve().parents[1]
FLEET = ROOT / "src" / "fleet_nav" / "config" / "fleet.yaml"
BRIDGE = ROOT / "src" / "fleet_nav" / "config" / "bridge.yaml"

# Shelf boxes in warehouse.sdf, and the inner face of the 0.2 m walls at ±5.
SHELVES = (
    (1.5, 0.5, 2.5, 3.5),
    (-2.5, -3.5, -1.5, -0.5),
)
BODY = 0.25
WALL = 4.9


def _gap_to_box(x, y, box):
    nearest_x = min(max(x, box[0]), box[2])
    nearest_y = min(max(y, box[1]), box[3])
    return math.hypot(x - nearest_x, y - nearest_y)


class WalkerTests(unittest.TestCase):
    def test_three_people_walk_the_open_aisles(self):
        with FLEET.open(encoding="utf-8") as handle:
            robots = yaml.safe_load(handle)["robots"]
        spawns = [(spec["x"], spec["y"]) for spec in robots.values()]
        duration = max(person.period for person in PEOPLE)
        samples = [step * 0.05 for step in range(int(duration / 0.05) + 1)]
        tracks = {person.name: [pose_at(person, t) for t in samples] for person in PEOPLE}

        for person in PEOPLE:
            self.assertIn(person.phase, ("sin", "cos"))
            self.assertAlmostEqual(abs(linear_speed(person, 0.05)), person.speed)
            for x, y in tracks[person.name]:
                self.assertLess(abs(x), WALL - BODY)
                self.assertLess(abs(y), WALL - BODY)
                for shelf in SHELVES:
                    self.assertGreater(_gap_to_box(x, y, shelf), BODY + 0.05, person.name)
                for spawn in spawns:
                    self.assertGreater(math.hypot(x - spawn[0], y - spawn[1]), 0.80, person.name)
            # A full period of the square wave brings the person back.
            end = pose_at(person, person.period)
            self.assertAlmostEqual(end[0], person.x, delta=0.05)
            self.assertAlmostEqual(end[1], person.y, delta=0.05)

        for index, left in enumerate(PEOPLE):
            for right in PEOPLE[index + 1:]:
                for a, b in zip(tracks[left.name], tracks[right.name]):
                    self.assertGreater(math.hypot(a[0] - b[0], a[1] - b[1]), 0.60)

    def test_the_bridge_drives_each_person(self):
        with BRIDGE.open(encoding="utf-8") as handle:
            topics = {item["gz_topic_name"] for item in yaml.safe_load(handle)}
        for person in PEOPLE:
            self.assertIn(f"/model/{person.name}/cmd_vel", topics)


if __name__ == "__main__":
    unittest.main()
