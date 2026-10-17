"""Corner following: finish the pivot when a flank is open."""

import unittest

import math

from fleet_nav.navigation_geometry import (
    AvoidanceState,
    bearing_off_path,
    choose_pass_side,
    command_on_path,
    frontal_blob,
    front_is_on_the_map,
    hold_pass_side,
    lateral_evasion_target,
    parallel_target,
    passing_command,
    passing_target,
    path_heading,
    path_lateral,
    path_lookahead,
    path_merge_target,
    pursuit_command,
    simplify_path,
    robot_pass_side,
    select_evasion_side,
    side_arrival,
    step_avoidance,
)


class PursuitTests(unittest.TestCase):
    def test_lookahead_aims_past_the_bend(self):
        path = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0), (0.4, 0.2), (0.4, 0.6), (0.4, 1.0)]
        target = path_lookahead(path, 0, 0.0, 0.0, 0.7)
        self.assertEqual(target, (0.4, 0.6))

    def test_a_close_corner_turns_toward_the_open_flank(self):
        linear, angular, stalling = pursuit_command(
            0.3, 0.0, 0.0,
            0.4, 0.8,
            front=0.30,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=1.50,
            right=0.40,
        )
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.5)
        self.assertFalse(stalling)

    def test_an_aligned_block_with_an_open_side_keeps_turning(self):
        # Heading already matches the lookahead, which still points at the corner.
        # The right side is the open one, so the pivot continues that way.
        linear, angular, stalling = pursuit_command(
            0.0, 0.0, 0.0,
            1.0, 0.0,
            front=0.20,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=0.40,
            right=1.50,
        )
        self.assertEqual(linear, 0.0)
        self.assertLess(angular, -0.5)
        self.assertFalse(stalling)

    def test_a_blocked_path_side_is_not_the_pivot_direction(self):
        linear, angular, stalling = pursuit_command(
            0.0, 0.0, 0.0,
            0.2, 1.0,
            front=0.25,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=0.10,
            right=1.20,
        )
        self.assertEqual(linear, 0.0)
        self.assertLess(angular, -0.5)
        self.assertFalse(stalling)

    def test_an_aligned_block_with_both_sides_tight_is_a_stall(self):
        linear, angular, stalling = pursuit_command(
            0.0, 0.0, 0.0,
            1.0, 0.0,
            front=0.20,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=0.10,
            right=0.10,
        )
        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.0)
        self.assertTrue(stalling)

    def _robot_scan(self):
        ranges = [3.0] * 360
        # Forward is index 180 when the scan starts at -pi.
        ranges[172:189] = [0.80] * 17
        return ranges

    def test_a_robot_ahead_is_a_narrow_blob(self):
        blob = frontal_blob(self._robot_scan(), -math.pi, 2.0 * math.pi / 360)
        self.assertIsNotNone(blob)
        low, high, nearest = blob
        self.assertLess(high - low, 0.5)
        self.assertAlmostEqual(nearest, 0.80)
        self.assertLess(abs(0.5 * (low + high)), 0.3)

    def test_a_wall_ahead_is_not_passed_as_a_robot(self):
        ranges = [3.0] * 360
        ranges[120:250] = [0.60] * 130
        blob = frontal_blob(ranges, -math.pi, 2.0 * math.pi / 360)
        self.assertIsNotNone(blob)
        low, high, nearest = blob
        self.assertIsNone(robot_pass_side(nearest, high - low, 0.5 * (low + high), 2.0, 2.0))

    def test_two_oncoming_robots_take_opposite_sides(self):
        side = robot_pass_side(0.80, 0.30, 0.0, left=2.0, right=1.2)
        self.assertEqual(side, 1.0)
        east = passing_target(0.0, 0.0, 0.0, side)
        west = passing_target(2.0, 0.0, math.pi, side)
        self.assertLess(east[1], 0.0)
        self.assertGreater(west[1], 0.0)

    def test_a_blocked_right_side_passes_on_the_left(self):
        side = robot_pass_side(0.70, 0.25, 0.05, left=1.5, right=0.20)
        self.assertEqual(side, -1.0)

    def test_passing_keeps_moving_instead_of_spinning(self):
        linear, angular = passing_command(
            0.0, 0.0, 0.0,
            0.7, -0.4,
            front=0.30,
            max_linear=0.30,
            max_angular=1.0,
        )
        self.assertGreater(linear, 0.1)
        self.assertLess(angular, 0.0)

    def test_parallel_target_nudges_toward_the_clearer_side(self):
        x, y = parallel_target(0.0, 0.0, 0.0, left=0.40, right=1.50, distance=0.60)
        self.assertGreater(x, 0.4)
        self.assertLess(y, 0.0)

    def test_parallel_target_stays_straight_between_two_walls(self):
        x, y = parallel_target(0.0, 0.0, 0.0, left=0.40, right=0.65, distance=0.60)
        self.assertGreater(x, 0.4)
        self.assertAlmostEqual(y, 0.0, places=3)

    def test_clear_and_aligned_drives_forward(self):
        linear, angular, stalling = pursuit_command(
            0.0, 0.0, 0.0,
            1.0, 0.0,
            front=2.0,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=0.20,
            right=0.20,
        )
        self.assertGreater(linear, 0.2)
        self.assertAlmostEqual(angular, 0.0)
        self.assertFalse(stalling)

    def test_shelves_beside_a_clear_line_are_not_a_robot(self):
        # Two compact returns well off the forward axis, with a clear gap between them.
        ranges = [3.0] * 360
        increment = 2.0 * math.pi / 360
        for center in (-0.95, 0.95):
            center_index = int(round((center + math.pi) / increment))
            ranges[center_index - 7:center_index + 8] = [0.70] * 15
        blob = frontal_blob(ranges, -math.pi, increment)
        self.assertIsNotNone(blob)
        low, high, nearest = blob
        bearing = 0.5 * (low + high)
        self.assertGreater(abs(bearing), 0.6)
        self.assertIsNone(robot_pass_side(nearest, high - low, bearing, 2.0, 2.0))

    def test_a_thin_shelf_edge_is_not_a_robot(self):
        ranges = [3.0] * 360
        ranges[180] = 0.60
        blob = frontal_blob(ranges, -math.pi, 2.0 * math.pi / 360)
        self.assertIsNotNone(blob)
        low, high, nearest = blob
        self.assertIsNone(robot_pass_side(nearest, high - low, 0.5 * (low + high), 2.0, 2.0))
        self.assertIsNone(robot_pass_side(0.60, 0.05, 0.0, left=2.0, right=2.0))

    def test_an_engaged_pass_does_not_flip_side(self):
        side = robot_pass_side(
            0.80, 0.30, 0.0, left=2.0, right=0.10, engaged_side=1.0,
        )
        self.assertEqual(side, 1.0)

    def test_open_flanks_on_a_straight_line_keep_going(self):
        # Something is inside the stop distance, but both sides of the corridor are open.
        linear, angular, stalling = pursuit_command(
            0.0, 0.0, 0.0,
            1.0, 0.0,
            front=0.30,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=2.0,
            right=2.0,
        )
        self.assertGreater(linear, 0.0)
        self.assertAlmostEqual(angular, 0.0)
        self.assertFalse(stalling)

    def test_a_stopped_robot_is_seen_beyond_one_metre(self):
        ranges = [3.0] * 360
        # Thirteen beams around index 180: about 0.21 rad, centered, at 1.60 m.
        ranges[174:187] = [1.60] * 13
        blob = frontal_blob(ranges, -math.pi, 2.0 * math.pi / 360)
        self.assertIsNotNone(blob)
        low, high, nearest = blob
        self.assertAlmostEqual(nearest, 1.60)
        side = robot_pass_side(nearest, high - low, 0.5 * (low + high), 2.0, 2.0)
        self.assertEqual(side, 1.0)

    def test_turning_onto_a_clear_line_does_not_dodge_a_shelf(self):
        # Spawn, nose yawed 0.40 rad: the shelf corner is in front of the nose
        # (bearing 0.11) but half a metre off the path.
        offset = bearing_off_path(0.40, 0.11, 0.0)
        self.assertGreater(abs(offset), 0.40)
        self.assertIsNone(robot_pass_side(
            1.22, 0.82, 0.11, left=1.8, right=1.2, path_offset=offset,
        ))

    def test_a_robot_on_the_path_is_passed_while_the_nose_is_still_turning(self):
        offset = bearing_off_path(0.20, -0.15, 0.0)
        self.assertLess(abs(offset), 0.10)
        self.assertEqual(
            robot_pass_side(1.20, 0.30, -0.15, left=2.0, right=2.0, path_offset=offset),
            1.0,
        )

    def test_a_far_return_off_the_axis_does_not_start_a_pass(self):
        self.assertIsNone(robot_pass_side(1.50, 0.22, 0.50, left=2.0, right=2.0))

    def test_open_space_pass_clears_two_chassis(self):
        _, y = passing_target(0.0, 0.0, 0.0, 1.0, clearance=2.0)
        self.assertAlmostEqual(y, -0.60, places=2)

    def test_a_pass_toward_a_near_wall_stays_inside_the_gap(self):
        _, y = passing_target(0.0, 0.0, 0.0, 1.0, clearance=0.60)
        self.assertAlmostEqual(y, -0.32, places=2)
        self.assertGreater(y, -0.60)

    def test_the_pass_turns_firmly_without_spinning(self):
        linear, angular = passing_command(
            0.0, 0.0, 0.0,
            1.20, -0.60,
            front=1.40,
            max_linear=0.30,
            max_angular=1.0,
        )
        self.assertGreater(linear, 0.20)
        self.assertLess(angular, -0.40)
        self.assertGreater(angular, -0.70)

    def test_a_robot_beside_the_nose_ends_the_pass(self):
        self.assertIsNone(
            robot_pass_side(0.70, 0.40, 1.40, left=2.0, right=2.0, engaged_side=1.0)
        )

    def test_a_latched_pass_along_a_straight_path_does_not_circle(self):
        path = [(i * 0.2, 0.0) for i in range(40)]
        x = y = yaw = 0.0
        index = 0
        max_abs_yaw = 0.0
        for _ in range(80):
            while index < len(path) - 1 and math.hypot(path[index][0] - x, path[index][1] - y) < 0.25:
                index += 1
            look = path_lookahead(path, index, x, y, 0.60)
            heading = path_heading(path, index, look[0], look[1], x, y)
            tx, ty = passing_target(
                x, y, heading, 1.0, path_x=look[0], path_y=look[1], clearance=2.0,
            )
            linear, angular = passing_command(x, y, yaw, tx, ty, 1.4, 0.30, 1.0)
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            max_abs_yaw = max(max_abs_yaw, abs(yaw))
        self.assertLess(max_abs_yaw, 1.0)
        self.assertGreater(x, 1.5)
        self.assertLess(y, -0.35)

    def test_cruise_aims_at_the_path_not_the_right_offset(self):
        path = [(0.0, 0.0), (2.0, 0.0)]
        tx, ty = path_merge_target(0.4, -0.45, path, 0, 0.6, 0.0)
        self.assertAlmostEqual(ty, 0.0, places=2)
        self.assertGreater(tx, 0.4)

    def test_rejoining_the_line_does_not_spin_in_place(self):
        # Beside the path, nose still along it. The lookahead is diagonal, but
        # the front is clear: drive back, do not stop and yaw.
        linear, angular, stalling = pursuit_command(
            0.0, -0.55, 0.0,
            0.60, 0.0,
            front=2.0,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=2.0,
            right=0.60,
        )
        self.assertGreater(linear, 0.10)
        self.assertLess(abs(angular), 0.85)
        self.assertFalse(stalling)

    def test_a_robot_on_the_flank_does_not_spin_the_pass_back(self):
        linear, angular, stalling = pursuit_command(
            0.8, -0.5, -0.4,
            1.4, 0.0,
            front=0.40,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=2.0,
            right=0.55,
        )
        self.assertGreater(linear, 0.05)
        self.assertLess(abs(angular), 0.85)
        self.assertFalse(stalling)

    def test_after_a_pass_the_robot_rejoins_without_stopping_to_spin(self):
        path = [(i * 0.2, 0.0) for i in range(50)]
        x = y = yaw = 0.0
        index = 0
        stopped_turn = 0.0
        max_abs_yaw = 0.0
        for step in range(80):
            t = step * 0.1
            while index < len(path) - 1 and math.hypot(path[index][0] - x, path[index][1] - y) < 0.25:
                index += 1
            look = path_lookahead(path, index, x, y, 0.60)
            if t < 3.0:
                heading = path_heading(path, index, look[0], look[1], x, y)
                tx, ty = passing_target(
                    x, y, heading, 1.0, path_x=look[0], path_y=look[1], clearance=2.0,
                )
                linear, angular = passing_command(x, y, yaw, tx, ty, 1.5, 0.30, 1.0)
            else:
                tx, ty = path_merge_target(x, y, path, index, look[0], look[1])
                linear, angular, _ = pursuit_command(
                    x, y, yaw, tx, ty, 2.0, 0.30, 1.0, 0.45, left=2.0, right=0.55,
                )
            if linear < 0.05 and abs(angular) > 0.2:
                stopped_turn += abs(angular) * 0.1
            yaw += angular * 0.1
            max_abs_yaw = max(max_abs_yaw, abs(yaw))
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
        self.assertLess(stopped_turn, 0.25)
        self.assertLess(max_abs_yaw, 1.0)
        self.assertGreater(x, 1.5)
        # Outside the pass the aim is the centreline, so the offset is given back.
        self.assertGreater(y, -0.20)
        self.assertLess(y, 0.20)

    def test_open_ground_without_a_visible_path_does_not_circle(self):
        x = y = yaw = 0.0
        for _ in range(60):
            tx, ty = parallel_target(x, y, yaw, 2.0, 2.0, 0.60)
            linear, angular, _ = pursuit_command(
                x, y, yaw, tx, ty, 2.0, 0.30, 1.0, 0.45, left=2.0, right=2.0,
            )
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
        self.assertLess(abs(yaw), 0.2)
        self.assertGreater(x, 1.0)

    def test_the_pass_continues_briefly_after_the_robot_leaves_the_scan(self):
        side, clear_since = hold_pass_side(None, 1.0, None, now=5.0)
        self.assertEqual(side, 1.0)
        self.assertEqual(clear_since, 5.0)
        side, clear_since = hold_pass_side(None, 1.0, clear_since, now=5.4)
        self.assertEqual(side, 1.0)
        side, _ = hold_pass_side(None, 1.0, 5.0, now=5.7)
        self.assertIsNone(side)

    def test_seeing_the_robot_again_drops_the_hold_timer(self):
        side, clear_since = hold_pass_side(1.0, 1.0, 5.0, now=6.0)
        self.assertEqual(side, 1.0)
        self.assertIsNone(clear_since)

    def test_a_misaligned_robot_on_the_line_turns_before_it_moves(self):
        # Spawn faces east. The path goes north. Rolling now would leave the line.
        linear, angular, stalling = command_on_path(
            -3.7, -3.7, 0.0,
            -3.7, -2.5,
            front=1.22,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=1.8,
            right=1.2,
            lateral=0.0,
        )
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.4)
        self.assertFalse(stalling)

    def test_a_bend_on_a_feasible_path_is_turned_while_moving(self):
        # Already on the route. The corner changes heading, but the line is
        # clear, so the robot must not stop and spin against the edge.
        linear, angular, stalling = command_on_path(
            0.0, 0.0, 0.0,
            0.6, 0.8,
            front=1.5,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=1.4,
            right=1.2,
            lateral=0.0,
            align=False,
        )
        self.assertGreater(linear, 0.10)
        self.assertGreater(angular, 0.4)
        self.assertFalse(stalling)

    def test_a_staircase_around_an_edge_becomes_one_feasible_line(self):
        # Grid steps alternate straight and diagonal. Clearance stays at 0.80 m
        # on the stair and falls to 0.10 m if the line cuts the corner.
        stair = []
        for i in range(12):
            stair.append((i * 0.02, 0.0 if i % 2 == 0 else 0.02))
        stair.append((0.30, 0.40))

        def clearance(x, y):
            if y > 0.05 and x < 0.20:
                return 0.10
            return 0.80

        def segment_free(x0, y0, x1, y1):
            return min(y0, y1) <= 0.05 or min(x0, x1) >= 0.20

        path = simplify_path(stair, clearance, segment_free, resolution=0.02)
        self.assertLess(len(path), len(stair))
        self.assertEqual(path[0], stair[0])
        self.assertEqual(path[-1], stair[-1])
        for start, end in zip(path, path[1:]):
            for sample in range(1, 8):
                blend = sample / 8
                x = start[0] + (end[0] - start[0]) * blend
                y = start[1] + (end[1] - start[1]) * blend
                self.assertGreaterEqual(clearance(x, y), 0.25)

    def test_an_offset_robot_rejoins_without_stopping(self):
        linear, angular, stalling = command_on_path(
            0.0, -0.55, 0.0,
            1.20, 0.0,
            front=2.0,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=2.0,
            right=0.60,
            lateral=0.55,
        )
        self.assertGreater(linear, 0.10)
        self.assertFalse(stalling)
        self.assertNotEqual(angular, 0.0)

    def test_two_robots_offset_to_their_own_right(self):
        side_east, _ = select_evasion_side(1.0, 0.0, 1.0, 0.0, 2.0, 2.0)
        side_west, _ = select_evasion_side(-1.0, 0.0, -1.0, 0.0, 2.0, 2.0)
        self.assertEqual(side_east, 1.0)
        self.assertEqual(side_west, 1.0)
        _, y_east = lateral_evasion_target(0.0, 0.0, 1.0, 0.0, side_east, 2.0, 2.0)
        _, y_west = lateral_evasion_target(0.0, 0.0, -1.0, 0.0, side_west, 2.0, 2.0)
        self.assertAlmostEqual(y_east, -0.35, places=2)
        self.assertAlmostEqual(y_west, 0.35, places=2)

    def test_turning_onto_the_line_does_not_start_an_evasion(self):
        state = AvoidanceState(closing=-0.4, last_front=1.22)
        command, state = step_avoidance(
            state, 1.0, -3.7, -3.7, 0.4,
            0.0, 1.0, 0.0, 1.0,
            front=1.20, left=1.8, right=1.5,
            heading_error=0.70,
        )
        self.assertIsNone(command)
        self.assertFalse(state.active)

    def test_a_scan_spike_does_not_start_an_evasion(self):
        state = AvoidanceState(last_front=3.0)
        command, state = step_avoidance(
            state, 1.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 1.0, 0.0,
            front=1.22, left=2.0, right=2.0,
            heading_error=0.0,
        )
        self.assertIsNone(command)
        self.assertFalse(state.active)
        self.assertLess(state.closing, -1.2)

    def test_a_wall_corner_does_not_start_an_evasion(self):
        # The forward return matches the map. Closing speed alone must not dodge it.
        state = AvoidanceState(closing=-0.3, last_front=1.50, approach_frames=5)
        command, _state = step_avoidance(
            state, 1.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 1.0, 0.0,
            front=1.40, left=1.5, right=0.40,
            heading_error=0.0,
            map_range=1.50,
        )
        self.assertIsNone(command)
        self.assertTrue(front_is_on_the_map(1.40, 1.50))

    def test_a_robot_closer_than_the_map_passes_on_the_open_side(self):
        self.assertIsNone(choose_pass_side(1.0, 0.0, 1.0, 0.0, left=0.40, right=0.50))
        self.assertEqual(choose_pass_side(1.0, 0.0, 1.0, 0.0, left=2.0, right=0.50), -1.0)
        state = AvoidanceState()
        front = 1.70
        command = None
        for step in range(6):
            front -= 0.03
            command, state = step_avoidance(
                state, step * 0.1, 0.0, 0.0, 0.0,
                1.0, 0.0, 1.2, 0.0,
                front=front, left=2.0, right=0.55,
                heading_error=0.0,
                map_range=3.0,
                path_x=0.0, path_y=0.0,
            )
            if command is not None:
                break
        self.assertIsNotNone(command)
        linear, angular = command
        # Still on the centreline: the offset is taken before the robot
        # draws level, so the first command is the slow hard turn.
        self.assertAlmostEqual(linear, 0.20)
        self.assertGreater(angular, 0.4)
        self.assertEqual(state.side, -1.0)

    def test_a_robot_on_the_left_is_passed_on_the_right(self):
        # The aim is also on the left, which used to pick that side and drive
        # into the stationary robot. The return is left of the nose, so the
        # pass goes right.
        self.assertEqual(
            choose_pass_side(1.0, 0.0, 1.0, 0.4, left=2.0, right=2.0, bearing=0.25),
            1.0,
        )
        # The body stays put in the world. A shrinking range at a fixed pose
        # would look like motion and the crossing rule would stop instead.
        state = AvoidanceState()
        ox, oy = 1.80, 0.46
        x = y = yaw = 0.0
        saw = False
        for step in range(16):
            dist = math.hypot(ox - x, oy - y)
            world = math.atan2(oy - y, ox - x)
            bearing = math.atan2(math.sin(world - yaw), math.cos(world - yaw))
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                1.0, 0.0, 1.2, 0.0,
                front=dist - 0.18, left=2.0, right=2.0,
                heading_error=0.0,
                map_range=6.0,
                path_x=0.0, path_y=0.0,
                bearing=bearing,
            )
            if command is None:
                linear, angular = 0.25, 0.0
            else:
                linear, angular = command
                self.assertEqual(state.side, 1.0)
                self.assertGreater(linear, 0.0)
                if not saw:
                    self.assertLess(angular, -0.2)
                saw = True
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
        self.assertTrue(saw)

    def test_a_right_hand_path_is_not_left_because_the_nose_is_yawed(self):
        # The path heads to the right. The other robot is left of that line,
        # and the right flank is open. The nose still points left of the
        # robot, so the raw scan bearing is negative and used to select left.
        path_heading_rad = -0.30
        yaw = path_heading_rad + 0.25
        nose_bearing = -0.13
        self.assertGreater(
            bearing_off_path(yaw, nose_bearing, path_heading_rad), 0.10,
        )
        self.assertLess(nose_bearing, -0.10)
        state = AvoidanceState()
        front = 1.70
        command = None
        path_ux = math.cos(path_heading_rad)
        path_uy = math.sin(path_heading_rad)
        for step in range(6):
            front -= 0.03
            command, state = step_avoidance(
                state, step * 0.1, 0.0, 0.0, yaw,
                path_ux, path_uy, path_ux, path_uy,
                front=front, left=2.0, right=2.0,
                heading_error=-0.25,
                map_range=3.0,
                path_x=0.0, path_y=0.0,
                bearing=nose_bearing,
            )
            if command is not None:
                break
        self.assertIsNotNone(command)
        _linear, angular = command
        self.assertLess(angular, -0.2)
        self.assertEqual(state.side, 1.0)

    def test_arriving_from_the_right_does_not_drive_into_the_robot(self):
        # Already on the right of the path, other robot ahead. The right flank
        # is only 0.80 m, so the old 1.20 m rule refused the pass and the
        # pursuit drove straight into it. Stay on the right, do not cross back.
        self.assertEqual(
            choose_pass_side(1.0, 0.0, 1.0, 0.0, left=2.0, right=0.80, bearing=0.30),
            1.0,
        )
        tx, ty = passing_target(
            0.0, -0.60, 0.0, 1.0,
            distance=0.80, lateral=0.55, clearance=0.80,
            path_x=0.0, path_y=0.0,
        )
        self.assertLessEqual(ty, -0.55)
        state = AvoidanceState()
        x, y, yaw = 0.0, -0.60, 0.0
        ox, oy = 1.8, 0.0
        started = False
        min_d = 99.0
        for step in range(80):
            dx, dy = ox - x, oy - y
            center = math.hypot(dx, dy)
            min_d = min(min_d, center)
            world = math.atan2(dy, dx)
            bearing = math.atan2(math.sin(world - yaw), math.cos(world - yaw))
            front = center if abs(bearing) <= 0.61 else 4.0
            right = 0.80
            aim_x, aim_y = x + 1.2, 0.0
            error = math.atan2(
                math.sin(math.atan2(aim_y - y, aim_x - x) - yaw),
                math.cos(math.atan2(aim_y - y, aim_x - x) - yaw),
            )
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                1.0, 0.0, aim_x - x, aim_y - y,
                front, 2.0, right, error,
                map_range=6.0,
                path_x=x, path_y=0.0,
                bearing=bearing if abs(bearing) <= 0.61 else 0.0,
            )
            if command is not None:
                started = True
                self.assertEqual(state.side, 1.0)
                linear, angular = command
            else:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, aim_x, aim_y, min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=2.0, right=right, lateral=abs(y), align=False,
                )
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            if center < 0.45:
                break
        self.assertTrue(started)
        self.assertGreater(min_d, 0.48)

    def test_arriving_on_a_moving_robots_right_yields_or_passes_behind(self):
        # Head-on, a stationary robot, and the robot who sees the other on its
        # own right are not this case. Only the one arriving on the other's
        # right stops, or shifts left to pass behind.
        self.assertIsNone(side_arrival(0.0, 0.40, -0.5))
        self.assertIsNone(side_arrival(0.50, 0.05, -0.5))
        self.assertEqual(side_arrival(0.50, 0.40, -0.4), "yield")
        self.assertEqual(side_arrival(0.50, 0.40, 0.4), "behind")
        self.assertIsNone(side_arrival(-0.50, 0.40, -0.4))

        state = AvoidanceState()
        command, state = step_avoidance(
            state, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 1.0, 0.0,
            front=1.30, left=2.0, right=2.0,
            heading_error=0.70,
            map_range=6.0,
            bearing=0.55,
        )
        self.assertIsNone(command)

        def cross(aim_x, aim_y, ox, oy, oy_step):
            state = AvoidanceState()
            seen = None
            for step in range(12):
                dist = math.hypot(ox, oy)
                command, state = step_avoidance(
                    state, step * 0.1, 0.0, 0.0, 0.0,
                    1.0, 0.0, aim_x, aim_y,
                    front=dist - 0.18, left=2.0, right=2.0,
                    heading_error=0.0,
                    map_range=6.0,
                    path_x=0.0, path_y=0.0,
                    bearing=math.atan2(oy, ox),
                )
                ox += 0.01
                oy += oy_step
                if command is not None and state.active:
                    seen = (command, state.side)
                    break
            return seen

        # Trajectory stays to the right of the robot on our left: stop.
        yielded = cross(1.0, 0.0, 1.40, 0.85, -0.05)
        self.assertIsNotNone(yielded)
        command, side = yielded
        self.assertEqual(command, (0.0, 0.0))
        self.assertEqual(side, 0.0)

        # Trajectory goes to its left: shift to our left and pass behind.
        behind = cross(0.4, 1.2, 1.40, 0.85, -0.05)
        self.assertIsNotNone(behind)
        command, side = behind
        self.assertEqual(side, -1.0)
        self.assertGreater(command[1], 0.0)

        # The other robot sees us on its right and keeps passing. It does not stop.
        other = cross(1.0, 0.0, 1.40, -0.85, 0.05)
        self.assertIsNotNone(other)
        command, side = other
        self.assertEqual(side, -1.0)
        self.assertGreater(command[0], 0.0)

        # Same place, but the body does not move in the world. The ordinary
        # right pass stays: a shrinking range at a fixed pose is not motion.
        state = AvoidanceState()
        ox, oy = 1.60, 1.00
        x = y = yaw = 0.0
        ordinary = False
        for step in range(10):
            dist = math.hypot(ox - x, oy - y)
            world = math.atan2(oy - y, ox - x)
            bearing = math.atan2(math.sin(world - yaw), math.cos(world - yaw))
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                1.0, 0.0, 1.2, 0.0,
                front=dist - 0.18, left=2.0, right=2.0,
                heading_error=0.0,
                map_range=6.0,
                path_x=0.0, path_y=0.0,
                bearing=bearing,
            )
            if command is None:
                linear, angular = 0.30, 0.0
            else:
                self.assertNotEqual(state.side, 0.0)
                linear, angular = command
                if state.side == 1.0:
                    ordinary = True
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
        self.assertTrue(ordinary)

    def test_a_robot_that_cuts_across_to_the_right_is_given_way(self):
        # Just left of the path, then across to our right. A right offset
        # follows that robot and the return to the line hits it.
        self.assertEqual(side_arrival(0.22, 0.40, -0.3), "yield")
        state = AvoidanceState()
        ox, oy = 1.50, 0.40
        stopped_after_cross = False
        for step in range(18):
            dist = math.hypot(ox, oy)
            command, state = step_avoidance(
                state, step * 0.1, 0.0, 0.0, 0.0,
                1.0, 0.0, 1.0, 0.0,
                front=dist - 0.18, left=2.0, right=2.0,
                heading_error=0.0,
                map_range=6.0,
                path_x=0.0, path_y=0.0,
                bearing=math.atan2(oy, ox),
            )
            ox += 0.02
            oy -= 0.06
            if oy < -0.25 and command is not None:
                self.assertEqual(command, (0.0, 0.0))
                self.assertEqual(state.side, 0.0)
                stopped_after_cross = True
        self.assertTrue(stopped_after_cross)

        # The right offset was already latched, and the other robot is now
        # on our right. Stop instead of following it and turning back.
        state = AvoidanceState(
            active=True, side=1.0, started=0.0, seen_off=0.30,
            last_front=1.15, other_speed=0.30, moving_frames=4,
            track_x=1.20, track_y=0.20,
        )
        ox, oy = 1.20, -0.45
        command, state = step_avoidance(
            state, 0.4, 0.0, 0.0, 0.0,
            1.0, 0.0, 1.0, 0.0,
            front=math.hypot(ox, oy) - 0.18, left=2.0, right=2.0,
            heading_error=0.0,
            map_range=6.0,
            bearing=math.atan2(oy, ox),
        )
        self.assertEqual(command, (0.0, 0.0))
        self.assertEqual(state.side, 0.0)

    def test_a_head_on_pass_stays_right_when_the_other_slides_left(self):
        # The pass starts on the axis. Later the other robot sits to our left
        # because we each took our own right. That must not become a stop.
        state = AvoidanceState()
        ox, oy = 1.60, 0.0
        saw_right = False
        for step in range(14):
            if step > 6:
                oy += 0.04
            else:
                ox -= 0.03
            dist = math.hypot(ox, oy)
            command, state = step_avoidance(
                state, step * 0.1, 0.0, 0.0, 0.0,
                1.0, 0.0, 1.0, 0.0,
                front=max(0.40, dist - 0.18), left=2.0, right=2.0,
                heading_error=0.0,
                map_range=6.0,
                path_x=0.0, path_y=0.0,
                bearing=math.atan2(oy, ox),
            )
            if state.active and state.side == 1.0:
                saw_right = True
            if saw_right and command is not None:
                self.assertEqual(state.side, 1.0)
                self.assertGreater(command[0], 0.0)
        self.assertTrue(saw_right)

    def test_a_shelf_wall_gap_stays_on_the_planned_line(self):
        # Shelf about 0.40 m on the left, wall about 0.65 m on the right.
        # The corner is shorter than the map ray, but neither flank is open.
        self.assertIsNone(choose_pass_side(1.0, 0.0, 1.0, 0.0, left=0.40, right=0.65))
        state = AvoidanceState(closing=-0.30, last_front=1.10, approach_frames=5)
        command, state = step_avoidance(
            state, 1.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 1.0, 0.0,
            front=0.95, left=0.40, right=0.65,
            heading_error=0.0,
            map_range=3.0,
            path_x=0.0, path_y=0.0,
        )
        self.assertIsNone(command)
        self.assertFalse(state.active)
        linear, angular, stalling = command_on_path(
            0.0, 0.0, 0.0,
            1.0, 0.0,
            front=0.40,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=0.40,
            right=0.65,
            lateral=0.0,
        )
        self.assertGreater(linear, 0.0)
        self.assertGreater(angular, -0.5)
        self.assertFalse(stalling)

    def test_a_turn_inside_the_gap_keeps_moving(self):
        linear, angular, stalling = command_on_path(
            0.0, 0.0, 0.0,
            0.6, 0.8,
            front=0.50,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=0.40,
            right=0.65,
            lateral=0.05,
        )
        self.assertGreater(linear, 0.0)
        self.assertLessEqual(abs(angular), 0.70)
        self.assertFalse(stalling)

    def test_a_pass_ends_when_both_sides_become_a_gap(self):
        state = AvoidanceState(active=True, side=1.0, started=0.0)
        command, state = step_avoidance(
            state, 1.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 1.0, 0.0,
            front=1.0, left=0.40, right=0.70,
            heading_error=0.0,
            map_range=3.0,
            map_left=0.50,
            map_right=0.80,
        )
        self.assertIsNone(command)
        self.assertFalse(state.active)

    def test_a_left_pass_of_a_stationary_robot_clears_the_chassis(self):
        # The other robot is on the right of the path, so the pass goes left.
        # The old 0.55 m carrot was still crossing the body when the two
        # drew level.
        other = (1.6, -0.35)
        path = [(i * 0.2, 0.0) for i in range(40)]
        state = AvoidanceState()
        x = y = yaw = 0.0
        index = 0
        min_sep = 99.0
        saw_left = False
        for step in range(160):
            while index < len(path) - 1 and math.hypot(path[index][0] - x, path[index][1] - y) < 0.25:
                index += 1
            look = path_lookahead(path, index, x, y, 0.60)
            aim = path_merge_target(x, y, path, index, look[0], look[1])
            heading = path_heading(path, index, look[0], look[1], x, y)
            front, left, right = self._body_sectors(x, y, yaw, other, 0.21)
            world = math.atan2(other[1] - y, other[0] - x)
            bearing = math.atan2(math.sin(world - yaw), math.cos(world - yaw))
            error = math.atan2(
                math.sin(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                math.cos(math.atan2(aim[1] - y, aim[0] - x) - yaw),
            )
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                math.cos(heading), math.sin(heading),
                aim[0] - x, aim[1] - y,
                front, left, right, error,
                map_range=6.0,
                path_x=look[0], path_y=look[1],
                bearing=bearing if abs(bearing) <= 0.61 else 0.0,
            )
            if command is not None:
                linear, angular = command
                if state.side < 0.0:
                    saw_left = True
            else:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, aim[0], aim[1], min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=left, right=right, lateral=abs(y), align=False,
                )
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            min_sep = min(min_sep, math.hypot(other[0] - x, other[1] - y))
            if x > other[0] + 0.8:
                break
        self.assertTrue(saw_left)
        self.assertGreater(min_sep, 0.70)

    def test_a_person_beside_the_path_is_passed_like_a_robot(self):
        # A walker is a 0.25 m cylinder, wider than a chassis. The pass is the
        # same lidar offset: nothing in the navigator is special to people.
        other = (1.6, -0.35)
        path = [(i * 0.2, 0.0) for i in range(40)]
        state = AvoidanceState()
        x = y = yaw = 0.0
        index = 0
        min_sep = 99.0
        saw_left = False
        for step in range(160):
            while index < len(path) - 1 and math.hypot(path[index][0] - x, path[index][1] - y) < 0.25:
                index += 1
            look = path_lookahead(path, index, x, y, 0.60)
            aim = path_merge_target(x, y, path, index, look[0], look[1])
            heading = path_heading(path, index, look[0], look[1], x, y)
            front, left, right = self._body_sectors(x, y, yaw, other, 0.25)
            world = math.atan2(other[1] - y, other[0] - x)
            bearing = math.atan2(math.sin(world - yaw), math.cos(world - yaw))
            error = math.atan2(
                math.sin(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                math.cos(math.atan2(aim[1] - y, aim[0] - x) - yaw),
            )
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                math.cos(heading), math.sin(heading),
                aim[0] - x, aim[1] - y,
                front, left, right, error,
                map_range=6.0,
                path_x=look[0], path_y=look[1],
                bearing=bearing if abs(bearing) <= 0.61 else 0.0,
            )
            if command is not None:
                linear, angular = command
                if state.side < 0.0:
                    saw_left = True
            else:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, aim[0], aim[1], min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=left, right=right, lateral=abs(y), align=False,
                )
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            min_sep = min(min_sep, math.hypot(other[0] - x, other[1] - y))
            if x > other[0] + 0.8:
                break
        self.assertTrue(saw_left)
        self.assertGreater(min_sep, 0.70)

    def test_a_mapped_wall_ahead_does_not_leave_the_line(self):
        # Driving at a wall the map already has. The old pass slid sideways into the flank.
        state = AvoidanceState()
        x = y = yaw = 0.0
        for step in range(40):
            now = step * 0.1
            wall = 2.4 - x
            command, state = step_avoidance(
                state, now, x, y, yaw,
                1.0, 0.0, 1.0, 0.0,
                front=wall - 0.05, left=1.6, right=1.5,
                heading_error=0.0,
                map_range=wall,
                path_x=0.0, path_y=0.0,
            )
            self.assertIsNone(command)
            self.assertFalse(state.active)
            linear, angular, _stalling = command_on_path(
                x, y, yaw, x + 1.0, 0.0, wall - 0.05, 0.30, 1.0, 0.45,
                left=1.6, right=1.5, lateral=0.0,
            )
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
        self.assertGreater(x, 0.5)
        self.assertAlmostEqual(y, 0.0, delta=0.05)
        self.assertAlmostEqual(yaw, 0.0, delta=0.15)

    def test_a_robot_closing_on_the_path_starts_the_pedestrian_pass(self):
        state = AvoidanceState()
        front = 1.70
        command = None
        for step in range(6):
            front -= 0.03
            command, state = step_avoidance(
                state, step * 0.1, 0.0, 0.0, 0.0,
                1.0, 0.0, 1.2, 0.0,
                front=front, left=2.0, right=2.0,
                heading_error=0.0,
            )
            if command is not None:
                break
        self.assertIsNotNone(command)
        linear, angular = command
        # Still on the centreline: the offset is taken before the robot
        # draws level, so the first command is the slow hard turn.
        self.assertAlmostEqual(linear, 0.20)
        self.assertLess(angular, -0.4)
        self.assertGreaterEqual(angular, -1.2)
        self.assertEqual(state.side, 1.0)

    def test_departure_stays_on_the_line_while_a_shelf_crosses_the_nose(self):
        # Face east, path goes north. The shelf corner sits in the forward
        # sector until the nose has turned. The robot must yaw in place.
        state = AvoidanceState()
        x = y = -3.7
        yaw = 0.0
        path = [(-3.7, -3.7 + i * 0.2) for i in range(20)]
        for step in range(40):
            now = step * 0.1
            index = 0
            look = path_lookahead(path, index, x, y, 0.60)
            aim = path_merge_target(x, y, path, index, look[0], look[1])
            heading = path_heading(path, index, look[0], look[1], x, y)
            lateral = path_lateral(x, y, path, index)
            error = math.atan2(
                math.sin(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                math.cos(math.atan2(aim[1] - y, aim[0] - x) - yaw),
            )
            front = 1.22 if abs(yaw) < 1.0 else 3.0
            command, state = step_avoidance(
                state, now, x, y, yaw,
                math.cos(heading), math.sin(heading),
                aim[0] - x, aim[1] - y,
                front, 1.8, 1.4, error,
            )
            self.assertIsNone(command)
            linear, angular, _stalling = command_on_path(
                x, y, yaw, aim[0], aim[1], front, 0.30, 1.0, 0.45,
                left=1.8, right=1.4, lateral=lateral,
            )
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
        self.assertAlmostEqual(x, -3.7, delta=0.08)
        self.assertGreater(yaw, 1.2)
        self.assertGreater(y, -3.5)

    def test_a_stationary_robot_is_passed_like_a_pedestrian(self):
        other = (2.2, 0.0)
        radius = 0.21
        path = [(i * 0.2, 0.0) for i in range(40)]
        state = AvoidanceState()
        x = y = yaw = 0.0
        index = 0
        min_sep = 99.0
        max_abs_yaw = 0.0
        saw_pass = False
        for step in range(180):
            now = step * 0.1
            while index < len(path) - 1 and math.hypot(path[index][0] - x, path[index][1] - y) < 0.25:
                index += 1
            look = path_lookahead(path, index, x, y, 0.60)
            aim = path_merge_target(x, y, path, index, look[0], look[1])
            heading = path_heading(path, index, look[0], look[1], x, y)
            lateral = path_lateral(x, y, path, index)
            front, left, right = self._body_sectors(x, y, yaw, other, radius)
            error = math.atan2(
                math.sin(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                math.cos(math.atan2(aim[1] - y, aim[0] - x) - yaw),
            )
            command, state = step_avoidance(
                state, now, x, y, yaw,
                math.cos(heading), math.sin(heading),
                aim[0] - x, aim[1] - y,
                front, left, right, error,
                map_range=3.0,
                path_x=look[0], path_y=look[1],
            )
            if command is not None:
                saw_pass = True
                linear, angular = command
            else:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, aim[0], aim[1], front, 0.30, 1.0, 0.45,
                    left=left, right=right, lateral=lateral,
                )
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            min_sep = min(min_sep, math.hypot(other[0] - x, other[1] - y))
            max_abs_yaw = max(max_abs_yaw, abs(yaw))
        self.assertTrue(saw_pass)
        self.assertGreater(min_sep, 0.50)
        self.assertGreater(x, other[0] + 0.3)
        self.assertLess(max_abs_yaw, 1.2)

    def _body_sectors(self, x, y, yaw, other, radius):
        dx = other[0] - x
        dy = other[1] - y
        surface = max(0.05, math.hypot(dx, dy) - radius)
        bearing = math.atan2(
            math.sin(math.atan2(dy, dx) - yaw),
            math.cos(math.atan2(dy, dx) - yaw),
        )
        front = left = right = 3.0
        if -0.61 <= bearing <= 0.61:
            front = surface
        elif 0.61 < bearing <= 1.92:
            left = surface
        elif -1.92 <= bearing < -0.61:
            right = surface
        return front, left, right
