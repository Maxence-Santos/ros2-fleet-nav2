"""Fleet assignment without ROS."""

from pathlib import Path
import unittest

import yaml

from fleet_nav.fleet_logic import BLOCKED, IDLE, NAVIGATING, FleetBook, click_action


FLEET = yaml.safe_load((Path(__file__).resolve().parents[1] / "src/fleet_nav/config/fleet.yaml").read_text())
SPAWN = {name: (float(item["x"]), float(item["y"])) for name, item in FLEET["robots"].items()}
DEMO = [(float(item[0]), float(item[1])) for item in FLEET["demo_goals"]]


def book_at_spawns(**kwargs):
    book = FleetBook(list(SPAWN), **kwargs)
    for name, (x, y) in SPAWN.items():
        book.set_pose(name, x, y, 0.0)
    return book


class FleetBookTests(unittest.TestCase):
    def test_demo_goal_is_not_given_to_another_robot_after_a_stall(self):
        book = FleetBook(["a", "b"], timeout_s=10.0, cooldown_s=8.0)
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.set_pose("b", 5.0, 0.0, 0.0)
        book.submit(1.0, 0.0, requeue=False)
        self.assertEqual(book.tick(0.0)[0].robot, "a")
        self.assertEqual(book.tick(10.0), [])
        self.assertEqual(book.robots["b"].status, IDLE)
        self.assertEqual(book.tick(30.0), [])

    def test_demo_goals_go_to_three_different_robots(self):
        book = book_at_spawns()
        for x, y in DEMO:
            book.submit(x, y)
        assigned = {item.robot for item in book.tick(0.0)}
        self.assertEqual(assigned, set(SPAWN))

    def test_nearest_idle_skips_a_busy_robot(self):
        book = FleetBook(["near", "far"])
        book.set_pose("near", 0.0, 0.0, 0.0)
        book.set_pose("far", 10.0, 0.0, 0.0)
        book.submit(1.0, 0.0)
        self.assertEqual(book.tick(0.0)[0].robot, "near")
        book.submit(1.2, 0.0)
        self.assertEqual(book.tick(0.1)[0].robot, "far")

    def test_small_steps_still_refresh_the_stall_timer(self):
        book = FleetBook(["a"], timeout_s=10.0, progress_m=0.3, cooldown_s=8.0)
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.submit(10.0, 0.0)
        book.tick(0.0)
        book.set_pose("a", 0.1, 0.0, 1.0)
        self.assertEqual(book.robots["a"].assigned_at, 0.0)
        book.set_pose("a", 0.4, 0.0, 2.0)
        self.assertEqual(book.robots["a"].assigned_at, 2.0)
        book.tick(11.0)
        self.assertEqual(book.robots["a"].status, NAVIGATING)

    def test_timeout_gives_the_mission_to_the_other_robot(self):
        book = FleetBook(["a", "b"], timeout_s=10.0, progress_m=0.3, cooldown_s=8.0)
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.set_pose("b", 10.0, 0.0, 0.0)
        book.submit(1.0, 0.0)
        self.assertEqual(book.tick(0.0)[0].robot, "a")
        reassigned = book.tick(10.0)
        self.assertEqual(reassigned[0].robot, "b")
        self.assertEqual(book.robots["a"].status, BLOCKED)
        self.assertEqual(book.take_cancels(), ["a"])

    def test_blocked_status_excludes_that_robot(self):
        book = FleetBook(["a", "b"], timeout_s=30.0, cooldown_s=8.0)
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.set_pose("b", 5.0, 0.0, 0.0)
        book.submit(1.0, 0.0)
        book.tick(0.0)
        book.note_status("a", NAVIGATING, 0.5)
        book.note_status("a", BLOCKED, 1.0)
        self.assertIsNone(book.robots["a"].mission_id)
        self.assertEqual(book.tick(1.0)[0].robot, "b")

    def test_success_frees_the_robot_without_requeue(self):
        book = FleetBook(["a"])
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.submit(1.0, 0.0)
        book.tick(0.0)
        book.note_status("a", NAVIGATING, 0.5)
        book.note_status("a", "succeeded", 1.0)
        self.assertEqual(book.robots["a"].status, IDLE)
        self.assertEqual(book.queue, [])
        self.assertEqual(book.tick(1.0), [])

    def test_every_robot_excluded_clears_and_retries_after_cooldown(self):
        book = FleetBook(["a"], timeout_s=10.0, cooldown_s=8.0)
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.submit(1.0, 0.0)
        book.tick(0.0)
        self.assertEqual(book.tick(10.0), [])
        self.assertEqual(book.robots["a"].status, BLOCKED)
        retried = book.tick(18.0)
        self.assertEqual(retried[0].robot, "a")
        self.assertEqual(retried[0].mission_id, 1)

    def test_a_clicked_goal_is_not_given_to_another_robot(self):
        book = FleetBook(["a", "b"], timeout_s=10.0, cooldown_s=8.0)
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.set_pose("b", 5.0, 0.0, 0.0)
        book.command("a", 1.0, 0.0, 0.0)
        book.note_status("a", NAVIGATING, 0.2)
        book.note_status("a", BLOCKED, 1.0)
        self.assertEqual(book.tick(1.0), [])
        self.assertEqual(book.robots["b"].status, IDLE)
        self.assertEqual(book.tick(20.0), [])

    def test_a_mission_sent_once_is_not_handed_over(self):
        book = FleetBook(["a", "b"], timeout_s=10.0, cooldown_s=8.0)
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.set_pose("b", 5.0, 0.0, 0.0)
        book.submit(1.0, 0.0, requeue=False)
        book.tick(0.0)
        book.note_status("a", NAVIGATING, 0.2)
        self.assertEqual(book.tick(10.0), [])
        self.assertEqual(book.robots["b"].status, IDLE)

    def test_a_stale_block_does_not_cancel_the_new_goal(self):
        book = FleetBook(["a", "b"], timeout_s=30.0)
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.set_pose("b", 5.0, 0.0, 0.0)
        book.command("a", 1.0, 0.0, 0.0)
        book.note_status("a", BLOCKED, 0.1)
        self.assertEqual(book.robots["a"].status, NAVIGATING)
        self.assertEqual(book.robots["a"].goal_x, 1.0)
        self.assertEqual(book.tick(0.1), [])

    def test_command_replaces_the_chosen_robot_only(self):
        book = FleetBook(["a", "b"])
        book.set_pose("a", 0.0, 0.0, 0.0)
        book.set_pose("b", 5.0, 0.0, 0.0)
        book.submit(1.0, 0.0)
        book.tick(0.0)
        assignment = book.command("a", 4.0, 0.0, 1.0)
        self.assertEqual(assignment.robot, "a")
        self.assertEqual(book.robots["a"].goal_x, 4.0)
        self.assertEqual(book.robots["b"].status, IDLE)
        self.assertEqual(book.take_cancels(), ["a"])
        self.assertEqual(book.tick(1.0), [])

    def test_click_on_a_robot_selects_it(self):
        poses = {"robot_1": (0.0, 0.0), "robot_2": (3.0, 0.0)}
        selected, goal = click_action(poses, 0.2, 0.1, None, 0.6)
        self.assertEqual(selected, "robot_1")
        self.assertIsNone(goal)

    def test_next_click_is_the_selected_robots_goal(self):
        poses = {"robot_1": (0.0, 0.0), "robot_2": (3.0, 0.0)}
        selected, goal = click_action(poses, 2.0, 2.0, "robot_1", 0.6)
        self.assertEqual(selected, "robot_1")
        self.assertEqual(goal, (2.0, 2.0))

    def test_goal_click_without_a_selection_does_nothing(self):
        selected, goal = click_action({"robot_1": (0.0, 0.0)}, 2.0, 2.0, None, 0.6)
        self.assertIsNone(selected)
        self.assertIsNone(goal)


if __name__ == "__main__":
    unittest.main()
