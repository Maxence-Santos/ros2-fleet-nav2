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
    sector_body_returns,
    sector_distances_from_scan,
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

    def test_a_u_turn_spins_in_place_even_after_cruise(self):
        # Already cruising, goal straight behind. Rolling through the turn
        # leaves the chassis beside the path for the rest of the leg.
        linear, angular, stalling = command_on_path(
            0.0, 0.0, 0.0,
            -1.2, 0.05,
            front=2.0,
            max_linear=0.30,
            max_angular=1.0,
            stop_distance=0.45,
            left=1.8,
            right=1.4,
            lateral=0.04,
            align=False,
        )
        self.assertEqual(linear, 0.0)
        self.assertGreater(abs(angular), 0.4)
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

        # That robot is on our left, so this one has priority. It stays on
        # the path. Stopping here, or arcing left, is the perpendicular miss.
        yielded = cross(1.0, 0.0, 1.40, 0.85, -0.05)
        self.assertIsNotNone(yielded)
        command, side = yielded
        self.assertGreater(command[0], 0.20)
        self.assertLess(abs(command[1]), 0.50)
        self.assertEqual(side, 0.0)

        # The aim goes to that robot's left. They are still on our left, so
        # the left arc is not the answer: this robot keeps the line.
        behind = cross(0.4, 1.2, 1.40, 0.85, -0.05)
        self.assertIsNotNone(behind)
        command, side = behind
        self.assertGreater(command[0], 0.20)
        self.assertLess(abs(command[1]), 0.50)
        self.assertEqual(side, 0.0)

        # That robot is crossing from our right toward our left. A left offset
        # follows it off the path. Stop instead, and let it go by.
        other = cross(1.0, 0.0, 1.40, -0.85, 0.05)
        self.assertIsNotNone(other)
        command, side = other
        self.assertEqual(side, 0.0)
        self.assertEqual(command, (0.0, 0.0))

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

    def test_a_perpendicular_robot_is_stopped_for_instead_of_followed(self):
        # The other robot crosses our path from the right. Offsetting left
        # yaws into its direction, holds the parallel, and cannot rejoin.
        # Stop while it is still in front and stay on our own line.
        state = AvoidanceState()
        x = y = yaw = 0.0
        ox, oy = 1.70, -1.10
        max_abs_y = 0.0
        min_centre = 99.0
        followed = False
        stopped_while_ahead = False
        for step in range(45):
            dist = math.hypot(ox - x, oy - y)
            bearing = math.atan2(
                math.sin(math.atan2(oy - y, ox - x) - yaw),
                math.cos(math.atan2(oy - y, ox - x) - yaw),
            )
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                1.0, 0.0, 1.2, 0.0,
                front=max(0.05, dist - 0.18), left=2.0, right=2.0,
                heading_error=0.0,
                map_range=6.0,
                path_x=0.0, path_y=0.0,
                bearing=bearing,
            )
            if command is None:
                linear, angular = 0.30, 0.0
            else:
                linear, angular = command
            if (
                state.active and state.side != 0.0
                and state.moving_frames >= 3 and state.other_speed >= 0.18
            ):
                followed = True
            if (
                command == (0.0, 0.0) and state.side == 0.0
                and oy - y < 0.30
            ):
                stopped_while_ahead = True
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            oy += 0.030
            max_abs_y = max(max_abs_y, abs(y))
            min_centre = min(min_centre, math.hypot(ox - x, oy - y))
            if oy - y > 1.0 and step > 15:
                break
        self.assertFalse(followed)
        self.assertTrue(stopped_while_ahead)
        self.assertLess(max_abs_y, 0.25)
        self.assertGreater(min_centre, 0.70)

    def test_the_robot_that_reaches_the_crossing_first_keeps_its_line(self):
        # Crossing at (2, 0). The northbound robot has 1.0 m to go and the
        # eastbound one 1.1 m. The closer one used to stop across the other's
        # path, and the later one drove through the open crossing.
        def drive(me, other, state, now):
            sectors = self._walker_scan(
                me["x"], me["y"], me["yaw"], other["x"], other["y"], radius=0.21,
            )
            hx, hy = math.cos(me["hd"]), math.sin(me["hd"])
            aim_x = me["x"] + 0.80 * hx
            aim_y = me["y"] + 0.80 * hy
            error = math.atan2(
                math.sin(me["hd"] - me["yaw"]), math.cos(me["hd"] - me["yaw"]),
            )
            front = sectors.front if sectors.front < 900.0 else 999.0
            left = sectors.left if sectors.left < 900.0 else 4.0
            right = sectors.right if sectors.right < 900.0 else 4.0
            command, state = step_avoidance(
                state, now, me["x"], me["y"], me["yaw"],
                hx, hy, aim_x - me["x"], aim_y - me["y"],
                front, left, right, error,
                map_range=8.0, map_left=8.0, map_right=8.0,
                path_x=me["px"], path_y=me["py"],
                bearing=0.0 if sectors.front > 900.0 else sectors.front_angle,
                left_bearing=None if sectors.left > 900.0 else sectors.left_angle,
                right_bearing=None if sectors.right > 900.0 else sectors.right_angle,
                dest_x=me["goal"][0] - me["x"], dest_y=me["goal"][1] - me["y"],
            )
            if command is None:
                linear, angular, _stalling = command_on_path(
                    me["x"], me["y"], me["yaw"], aim_x, aim_y,
                    min(front, 3.0), 0.30, 1.0, 0.45,
                    left=min(left, 3.0), right=min(right, 3.0), align=False,
                )
                stopped = False
            else:
                linear, angular = command
                stopped = command == (0.0, 0.0) and state.side == 0.0
            return state, linear, angular, stopped

        def scene(a, b):
            sa = AvoidanceState()
            sb = AvoidanceState()
            min_d = 99.0
            stop_a = stop_b = 0
            for step in range(70):
                now = step * 0.1
                sa, lin_a, ang_a, stopped_a = drive(a, b, sa, now)
                sb, lin_b, ang_b, stopped_b = drive(b, a, sb, now)
                if stopped_a:
                    stop_a += 1
                if stopped_b:
                    stop_b += 1
                a["yaw"] += ang_a * 0.1
                b["yaw"] += ang_b * 0.1
                a["x"] += lin_a * math.cos(a["yaw"]) * 0.1
                a["y"] += lin_a * math.sin(a["yaw"]) * 0.1
                b["x"] += lin_b * math.cos(b["yaw"]) * 0.1
                b["y"] += lin_b * math.sin(b["yaw"]) * 0.1
                min_d = min(min_d, math.hypot(a["x"] - b["x"], a["y"] - b["y"]))
                if a["mark"] is None and a["x"] >= 2.0:
                    a["mark"] = (a["x"], a["y"], b["x"], b["y"])
                if b["mark"] is None and b["y"] >= 0.0:
                    b["mark"] = (a["x"], a["y"], b["x"], b["y"])
            return a, b, min_d, stop_a, stop_b

        def robot(x, y, hd, goal, px, py):
            return {
                "x": x, "y": y, "yaw": hd, "hd": hd, "goal": goal,
                "px": px, "py": py, "mark": None,
            }

        # Northbound is on the eastbound robot's right, so it has priority
        # even when the distances are almost the same. Eastbound waits.
        a = robot(0.9, 0.0, 0.0, (4.0, 0.0), 0.0, 0.0)
        b = robot(2.0, -1.0, math.pi / 2, (2.0, 3.0), 2.0, -3.0)
        a, b, min_d, stop_a, stop_b = scene(a, b)
        self.assertIsNotNone(b["mark"])
        self.assertLess(b["mark"][0], 1.50)
        self.assertLess(abs(b["mark"][2] - 2.0), 0.35)
        self.assertGreater(stop_a, stop_b)
        self.assertGreater(min_d, 0.55)

        # Southbound is on the eastbound robot's left. Eastbound has priority
        # and keeps the line. Who is closer does not change the side.
        a = robot(1.5, 0.0, 0.0, (4.0, 0.0), 0.0, 0.0)
        b = robot(2.0, 1.7, -math.pi / 2, (2.0, -3.0), 2.0, 3.0)
        a, b, min_d, stop_a, stop_b = scene(a, b)
        self.assertIsNotNone(a["mark"])
        self.assertGreater(a["mark"][3], 0.40)
        self.assertLess(abs(a["mark"][1]), 0.35)
        self.assertGreater(stop_b, stop_a)
        self.assertGreater(min_d, 0.55)

    def _square_robot_ranges(self, x, y, yaw, ox, oy, oyaw):
        """Lidar ranges to the other chassis, from the sensor 0.15 m ahead."""
        lx = x + 0.15 * math.cos(yaw)
        ly = y + 0.15 * math.sin(yaw)
        samples = 360
        inc = 2.0 * math.pi / samples
        ranges = []
        c, s = math.cos(oyaw), math.sin(oyaw)
        half = 0.15
        for index in range(samples):
            angle = -math.pi + index * inc
            dx, dy = math.cos(yaw + angle), math.sin(yaw + angle)
            # Other chassis, in its own frame.
            bx = c * (lx - ox) + s * (ly - oy)
            by = -s * (lx - ox) + c * (ly - oy)
            rdx = c * dx + s * dy
            rdy = -s * dx + c * dy
            hit = None
            for edge, origin, direction, other_o, other_d in (
                (half, bx, rdx, by, rdy),
                (-half, bx, rdx, by, rdy),
                (half, by, rdy, bx, rdx),
                (-half, by, rdy, bx, rdx),
            ):
                if abs(direction) < 1e-9:
                    continue
                t = (edge - origin) / direction
                if t <= 0.05:
                    continue
                other = other_o + t * other_d
                if -half - 1e-6 <= other <= half + 1e-6 and (hit is None or t < hit):
                    hit = t
            ranges.append(hit if hit is not None and 0.15 < hit <= 3.0 else float("inf"))
        return ranges, inc

    def _square_cross(
        self, ax, ay, bx, by, b_dir=1.0, steps=140, bias=0.0,
        wobble=0.0, blank_every=0,
    ):
        """Eastbound A on y=ay. B travels on x=bx, north if b_dir > 0.

        Square lidar, empty sectors stay 999, map ray is 3 m. ``bias`` shifts
        finite ranges so a noisy sweep cannot pick a different robot.
        """
        def make(x, y, heading, goal):
            path = []
            for i in range(40):
                a = i / 39.0
                path.append((x + (goal[0] - x) * a, y + (goal[1] - y) * a))
            return {
                "x": x, "y": y, "yaw": heading, "v": 0.0, "w": 0.0,
                "goal": goal, "path": path, "cruise": False, "index": 0,
            }

        def advance(me):
            path = me["path"]
            index = me["index"]
            while index < len(path) - 1:
                waypoint = path[index]
                if math.hypot(waypoint[0] - me["x"], waypoint[1] - me["y"]) < 0.25:
                    index += 1
                    continue
                heading = path_heading(path, index, waypoint[0], waypoint[1], me["x"], me["y"])
                along = (me["x"] - waypoint[0]) * math.cos(heading) + (me["y"] - waypoint[1]) * math.sin(heading)
                if along > 0.0:
                    index += 1
                    continue
                break
            me["index"] = index

        def drive(me, other, state, now):
            advance(me)
            ranges, inc = self._square_robot_ranges(
                me["x"], me["y"], me["yaw"], other["x"], other["y"], other["yaw"],
            )
            sectors = sector_distances_from_scan(ranges, -math.pi, inc, range_min=0.15)
            path = me["path"]
            index = me["index"]
            waypoint = path[max(0, min(index, len(path) - 1))]
            aim = path_merge_target(me["x"], me["y"], path, index, waypoint[0], waypoint[1])
            heading = path_heading(path, index, waypoint[0], waypoint[1], me["x"], me["y"])
            error = math.atan2(
                math.sin(math.atan2(aim[1] - me["y"], aim[0] - me["x"]) - me["yaw"]),
                math.cos(math.atan2(aim[1] - me["y"], aim[0] - me["x"]) - me["yaw"]),
            )
            def shift(value, empty):
                if value >= 900.0 or (blank_every and int(round(now * 10)) % blank_every == 0):
                    return empty
                extra = wobble if int(round(now * 10)) % 2 == 0 else -wobble
                return max(0.05, value + bias + extra)

            front = shift(sectors.front, 999.0)
            left = shift(sectors.left, 999.0)
            right = shift(sectors.right, 999.0)
            command, state = step_avoidance(
                state, now, me["x"], me["y"], me["yaw"],
                math.cos(heading), math.sin(heading),
                aim[0] - me["x"], aim[1] - me["y"],
                front, left, right, error,
                map_range=3.0, map_left=3.0, map_right=3.0,
                path_x=waypoint[0], path_y=waypoint[1],
                bearing=sectors.front_angle,
                left_bearing=sectors.left_angle,
                right_bearing=sectors.right_angle,
                dest_x=me["goal"][0] - me["x"], dest_y=me["goal"][1] - me["y"],
            )
            if command is None:
                linear, angular, _stalling = command_on_path(
                    me["x"], me["y"], me["yaw"], aim[0], aim[1], min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=min(left, 3.0), right=min(right, 3.0),
                    lateral=path_lateral(me["x"], me["y"], path, index),
                    align=not me["cruise"],
                )
                if abs(error) <= 0.30:
                    me["cruise"] = True
            else:
                linear, angular = command
            me["v"] += max(-0.10, min(0.10, linear - me["v"]))
            me["w"] += max(-0.20, min(0.20, angular - me["w"]))
            me["yaw"] += me["w"] * 0.1
            me["x"] += me["v"] * math.cos(me["yaw"]) * 0.1
            me["y"] += me["v"] * math.sin(me["yaw"]) * 0.1
            stopped = command == (0.0, 0.0) and state.side == 0.0
            behind = state.active and state.side < 0.0
            offset = state.active and state.side != 0.0
            lateral = path_lateral(me["x"], me["y"], path, me["index"])
            return state, stopped, behind, lateral, linear, offset

        a = make(ax, ay, 0.0, (ax + 8.0, ay))
        b = make(bx, by, math.pi / 2 * (1.0 if b_dir > 0.0 else -1.0), (bx, by + 8.0 * b_dir))
        sa, sb = AvoidanceState(), AvoidanceState()
        stats = {
            "stop_a": 0, "stop_b": 0, "lat_a": 0.0, "lat_b": 0.0,
            "behind_a": False, "behind_b": False, "off_a": 0, "off_b": 0,
            "min_d": 99.0,
            "ax_when_b_clear": None, "by_when_a_clear": None,
            "rev_late_a": 0, "rev_late_b": 0,
            "rev_early_a": 0, "rev_early_b": 0,
        }
        late_from = max(0, steps - 40)
        for step in range(steps):
            now = step * 0.1
            snap_a = {"x": a["x"], "y": a["y"], "yaw": a["yaw"]}
            snap_b = {"x": b["x"], "y": b["y"], "yaw": b["yaw"]}
            sa, stop_a, behind_a, lat_a, lin_a, off_a = drive(a, snap_b, sa, now)
            sb, stop_b, behind_b, lat_b, lin_b, off_b = drive(b, snap_a, sb, now)
            if step == late_from:
                stats["ax_late"] = a["x"]
                stats["ay_late"] = a["y"]
                stats["bx_late"] = b["x"]
                stats["by_late"] = b["y"]
            if stop_a:
                stats["stop_a"] += 1
            if stop_b:
                stats["stop_b"] += 1
            stats["lat_a"] = max(stats["lat_a"], lat_a)
            stats["lat_b"] = max(stats["lat_b"], lat_b)
            stats["behind_a"] = stats["behind_a"] or behind_a
            stats["behind_b"] = stats["behind_b"] or behind_b
            if off_a:
                stats["off_a"] += 1
            if off_b:
                stats["off_b"] += 1
            stats["min_d"] = min(stats["min_d"], math.hypot(a["x"] - b["x"], a["y"] - b["y"]))
            b_past = (b_dir > 0.0 and b["y"] >= ay + 0.70) or (b_dir < 0.0 and b["y"] <= ay - 0.70)
            a_past = a["x"] >= bx + 0.55
            if b_past and lin_a < -0.05:
                stats["rev_late_a"] += 1
            if a_past and lin_b < -0.05:
                stats["rev_late_b"] += 1
            if step < 15 and lin_a < -0.05:
                stats["rev_early_a"] += 1
            if step < 15 and lin_b < -0.05:
                stats["rev_early_b"] += 1
            if stats["ax_when_b_clear"] is None and b_past:
                stats["ax_when_b_clear"] = a["x"]
            if stats["by_when_a_clear"] is None and a["x"] >= bx + 0.35:
                stats["by_when_a_clear"] = b["y"]
        stats["ax"] = a["x"]
        stats["ay"] = a["y"]
        stats["bx"] = b["x"]
        stats["by"] = b["y"]
        return stats

    def test_a_perpendicular_second_robot_stops_instead_of_passing_behind(self):
        # Crossing at (2, 0). Empty sectors stay 999 and the map ray is the
        # 3 m lidar limit. Priority is the robot that has the other on its
        # left. The other stops on its line, then starts again once the first
        # is past the crossing. Neither one arcs off the path.
        def check(stats, yielder, where):
            self.assertEqual(stats["off_a"], 0, where)
            self.assertEqual(stats["off_b"], 0, where)
            self.assertFalse(stats["behind_a"], where)
            self.assertFalse(stats["behind_b"], where)
            self.assertLess(stats["lat_a"], 0.20, where)
            self.assertLess(stats["lat_b"], 0.20, where)
            self.assertGreater(stats["min_d"], 0.60, where)
            if yielder == "a":
                self.assertIsNotNone(stats["ax_when_b_clear"], where)
                self.assertLess(stats["ax_when_b_clear"], 1.85, where)
                self.assertLess(abs(stats["bx"] - 2.0), 0.35, where)
                self.assertGreater(stats["stop_a"], stats["stop_b"], where)
                self.assertLess(stats["stop_b"], 8, where)
                self.assertGreater(stats["ax"], stats["ax_when_b_clear"] + 0.30, where)
                self.assertLess(abs(stats["ay"]), 0.20, where)
                self.assertLess(stats["rev_late_a"], 3, where)
            else:
                self.assertIsNotNone(stats["by_when_a_clear"], where)
                # Still on the north approach when A clears the crossing.
                self.assertGreater(stats["by_when_a_clear"], 0.15, where)
                self.assertLess(abs(stats["ay"]), 0.20, where)
                self.assertGreater(stats["ax"], 2.30, where)
                self.assertGreater(stats["stop_b"], stats["stop_a"], where)
                self.assertLess(stats["stop_a"], 8, where)
                self.assertLess(stats["by"], stats["by_when_a_clear"] - 0.30, where)
                self.assertLess(abs(stats["bx"] - 2.0), 0.35, where)
                self.assertLess(stats["rev_late_b"], 3, where)

        # From the south, B is on A's right. A waits, including when A is closer.
        for ax, by, bias in (
            (1.0, -0.4, 0.0),
            (1.0, -1.6, 0.0),
            (0.4, -0.5, 0.0),
            (1.4, -0.3, 0.0),
            (1.0, -0.4, 0.06),
            (1.0, -0.4, -0.06),
        ):
            stats = self._square_cross(ax, 0.0, 2.0, by, b_dir=1.0, bias=bias)
            check(stats, "a", f"A from x={ax}, B from y={by}, bias={bias}")

        # A is already on the crossing and B is still far up the aisle.
        # A goes through. B must not freeze after A has left.
        for ax, by, bias in ((1.6, -1.8, 0.0), (1.6, -1.8, 0.06)):
            stats = self._square_cross(ax, 0.0, 2.0, by, b_dir=1.0, bias=bias)
            where = f"A from x={ax}, B from y={by}, bias={bias}"
            self.assertEqual(stats["off_a"], 0, where)
            self.assertEqual(stats["off_b"], 0, where)
            self.assertGreater(stats["min_d"], 0.60, where)
            self.assertLess(stats["lat_a"], 0.20, where)
            self.assertLess(stats["lat_b"], 0.20, where)
            self.assertGreater(stats["by"], 1.0, where)
            self.assertLess(stats["stop_b"], 16, where)
            self.assertLess(stats["rev_late_a"], 3, where)
            self.assertLess(stats["rev_late_b"], 3, where)

        # From the north, B is on A's left. B waits, including when B is closer.
        for ax, by, bias in (
            (1.0, 0.4, 0.0),
            (1.0, 1.6, 0.0),
            (0.4, 1.8, 0.0),
            (1.6, 0.5, 0.0),
            (1.0, 0.4, -0.06),
        ):
            stats = self._square_cross(ax, 0.0, 2.0, by, b_dir=-1.0, bias=bias)
            check(stats, "b", f"A from x={ax}, B from y={by}, bias={bias}")

    def test_a_wobbly_perpendicular_crossing_stays_on_the_lines(self):
        # A sweep that jitters, or drops the robot for one frame, used to
        # start a side pass. One robot, or both, left the line and they met
        # again on the way back. The robot with the other on its right still
        # stops. The other does not arc off.
        cases = (
            (1.0, -0.4, 1.0, "a"),
            (1.0, -1.6, 1.0, "a"),
            (1.4, -0.3, 1.0, "a"),
            (1.6, -1.8, 1.0, "far"),
            (1.0, 0.4, -1.0, "b"),
            (1.6, 0.5, -1.0, "b"),
        )
        for ax, by, direction, kind in cases:
            for wobble, blank in ((0.05, 0), (0.0, 4)):
                stats = self._square_cross(
                    ax, 0.0, 2.0, by, b_dir=direction,
                    wobble=wobble, blank_every=blank,
                )
                where = (
                    f"A from x={ax}, B from y={by}, dir={direction}, "
                    f"wobble={wobble}, blank={blank}"
                )
                self.assertEqual(stats["off_a"], 0, where)
                self.assertEqual(stats["off_b"], 0, where)
                # One dropped sweep on the close north start reads 0.598 m.
                # The clean runs stay above 0.60. Below 0.55 the squares meet.
                self.assertGreater(stats["min_d"], 0.55 if blank else 0.60, where)
                self.assertLess(stats["lat_a"], 0.20, where)
                self.assertLess(stats["lat_b"], 0.20, where)
                self.assertLess(stats["rev_late_a"], 3, where)
                self.assertLess(stats["rev_late_b"], 3, where)
                if kind == "a":
                    self.assertGreater(stats["stop_a"], stats["stop_b"], where)
                    self.assertLess(stats["stop_b"], 8, where)
                elif kind == "b":
                    self.assertGreater(stats["stop_b"], stats["stop_a"], where)
                    self.assertLess(stats["stop_a"], 8, where)
                else:
                    # Already in the crossing, with the other still up the
                    # aisle. A goes through. A wobbling range can hold B's
                    # pause, but B still clears the crossing and neither
                    # robot arcs off. A dropped sweep must not freeze B.
                    self.assertGreater(stats["ax"], 3.0, where)
                    if wobble == 0.0:
                        self.assertLess(stats["stop_b"], 16, where)
                        self.assertGreater(stats["by"], 1.0, where)
                    else:
                        self.assertGreater(stats["by"], 0.40, where)

    def test_a_speed_dip_does_not_stop_the_robot_that_has_priority(self):
        # Seen at cruise, then the hit slows into the walker band while the
        # other chassis is still in the forward sector and one flank. Releasing
        # the straight command there makes path following stop, and both robots
        # sit short of the intersection.
        state = AvoidanceState()
        ox, oy, oyaw = 0.70, 0.40, math.pi / 2
        blocked = 0
        now = 0.0
        for speed, frames in ((-0.30, 6), (-0.15, 8)):
            for _ in range(frames):
                ranges, inc = self._square_robot_ranges(0.0, 0.0, 0.0, ox, oy, oyaw)
                sectors = sector_distances_from_scan(ranges, -math.pi, inc, range_min=0.15)
                command, state = step_avoidance(
                    state, now, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0,
                    sectors.front, sectors.left, sectors.right, 0.0,
                    map_range=3.0, map_left=3.0, map_right=3.0,
                    path_x=0.6, path_y=0.0,
                    bearing=sectors.front_angle,
                    left_bearing=sectors.left_angle,
                    right_bearing=sectors.right_angle,
                    dest_x=4.0, dest_y=0.0,
                )
                in_band = (
                    state.seen_fast
                    and 0.18 <= state.other_speed < 0.24
                    and sectors.front < 0.45
                    and min(sectors.left, sectors.right) < 0.50
                )
                if in_band:
                    blocked += 1
                    self.assertIsNotNone(command)
                    self.assertGreater(command[0], 0.20)
                    self.assertEqual(state.side, 0.0)
                    self.assertTrue(state.cross_go)
                oy += speed * 0.1
                now += 0.1
        self.assertGreater(blocked, 0)

    def test_after_a_perpendicular_crossing_both_robots_leave_on_their_lines(self):
        # Already near the intersection. The robot with priority used to yaw
        # off its line, latch a yield, and then alternate forward and reverse
        # on an empty scan after the other robot had left.
        for ax, by, direction in (
            (1.80, -0.60, 1.0),
            (1.80, -0.30, 1.0),
            (1.80, -0.75, 1.0),
            (1.50, 0.30, -1.0),
        ):
            stats = self._square_cross(ax, 0.0, 2.0, by, b_dir=direction, steps=180)
            where = f"A from x={ax}, B from y={by}, dir={direction}"
            self.assertGreater(abs(stats["ax"] - stats["ax_late"]), 0.80, where)
            self.assertGreater(abs(stats["by"] - stats["by_late"]), 0.80, where)
            self.assertLess(stats["rev_late_a"], 3, where)
            self.assertLess(stats["rev_late_b"], 3, where)
            self.assertLess(abs(stats["ay"] - 0.0), 0.35, where)
            self.assertLess(abs(stats["bx"] - 2.0), 0.35, where)

    def test_a_far_crossing_does_not_stop_the_robot_on_the_left(self):
        # Paths cross at (3, 0). B is still about 3 m down its aisle, inside
        # the lidar but far from the crossing. The robot that has B on its
        # right used to freeze as soon as that return existed.
        stats = self._square_cross(0.0, 0.0, 3.0, -3.0, b_dir=1.0, steps=50)
        self.assertGreater(stats["ax"], 1.00)
        self.assertLess(stats["stop_a"], 20)
        self.assertGreater(stats["min_d"], 1.50)
        self.assertLess(stats["lat_a"], 0.20)
        self.assertLess(stats["lat_b"], 0.20)
        self.assertLess(stats["by"], -1.20)

    def _square_face_gap(self, ax, ay, ayaw, bx, by, byaw, half=0.15):
        """Positive when two oriented chassis squares are apart."""

        def project(cx, cy, yaw, axis_x, axis_y):
            c, s = math.cos(yaw), math.sin(yaw)
            centre = cx * axis_x + cy * axis_y
            extent = half * (abs(c * axis_x + s * axis_y) + abs(-s * axis_x + c * axis_y))
            return centre, extent

        best = -99.0
        for yaw in (ayaw, byaw):
            c, s = math.cos(yaw), math.sin(yaw)
            for axis_x, axis_y in ((c, s), (-s, c)):
                ca, ea = project(ax, ay, ayaw, axis_x, axis_y)
                cb, eb = project(bx, by, byaw, axis_x, axis_y)
                best = max(best, abs(ca - cb) - ea - eb)
        return best

    def _beside_cross(self, ax, by, b_dir=1.0, dwell=0, steps=80):
        """Eastbound A on y=0, B on x=2. Both may sit still before they move."""

        def make(x, y, heading, goal):
            path = [
                (x + (goal[0] - x) * i / 39.0, y + (goal[1] - y) * i / 39.0)
                for i in range(40)
            ]
            return {
                "x": x, "y": y, "yaw": heading, "v": 0.0, "w": 0.0,
                "goal": goal, "path": path, "cruise": False, "index": 0,
            }

        def advance(me):
            path, index = me["path"], me["index"]
            while index < len(path) - 1:
                waypoint = path[index]
                if math.hypot(waypoint[0] - me["x"], waypoint[1] - me["y"]) < 0.25:
                    index += 1
                    continue
                heading = path_heading(path, index, waypoint[0], waypoint[1], me["x"], me["y"])
                along = (
                    (me["x"] - waypoint[0]) * math.cos(heading)
                    + (me["y"] - waypoint[1]) * math.sin(heading)
                )
                if along > 0.0:
                    index += 1
                    continue
                break
            me["index"] = index

        def drive(me, other, state, now, move):
            advance(me)
            ranges, inc = self._square_robot_ranges(
                me["x"], me["y"], me["yaw"], other["x"], other["y"], other["yaw"],
            )
            sectors = sector_distances_from_scan(ranges, -math.pi, inc, range_min=0.15)
            path, index = me["path"], me["index"]
            waypoint = path[max(0, min(index, len(path) - 1))]
            aim = path_merge_target(me["x"], me["y"], path, index, waypoint[0], waypoint[1])
            heading = path_heading(path, index, waypoint[0], waypoint[1], me["x"], me["y"])
            error = math.atan2(
                math.sin(math.atan2(aim[1] - me["y"], aim[0] - me["x"]) - me["yaw"]),
                math.cos(math.atan2(aim[1] - me["y"], aim[0] - me["x"]) - me["yaw"]),
            )
            front = sectors.front if sectors.front < 900.0 else 999.0
            left = sectors.left if sectors.left < 900.0 else 999.0
            right = sectors.right if sectors.right < 900.0 else 999.0
            command, state = step_avoidance(
                state, now, me["x"], me["y"], me["yaw"],
                math.cos(heading), math.sin(heading),
                aim[0] - me["x"], aim[1] - me["y"],
                front, left, right, error,
                map_range=3.0, map_left=3.0, map_right=3.0,
                path_x=waypoint[0], path_y=waypoint[1],
                bearing=sectors.front_angle,
                left_bearing=sectors.left_angle,
                right_bearing=sectors.right_angle,
                dest_x=me["goal"][0] - me["x"], dest_y=me["goal"][1] - me["y"],
            )
            if command is None:
                linear, angular, _stalling = command_on_path(
                    me["x"], me["y"], me["yaw"], aim[0], aim[1], min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=min(left, 3.0), right=min(right, 3.0),
                    lateral=path_lateral(me["x"], me["y"], path, index),
                    align=not me["cruise"],
                )
                if abs(error) <= 0.30:
                    me["cruise"] = True
            else:
                linear, angular = command
            if move:
                me["v"] += max(-0.10, min(0.10, linear - me["v"]))
                me["w"] += max(-0.20, min(0.20, angular - me["w"]))
                me["yaw"] += me["w"] * 0.1
                me["x"] += me["v"] * math.cos(me["yaw"]) * 0.1
                me["y"] += me["v"] * math.sin(me["yaw"]) * 0.1
            lateral = abs(path_lateral(me["x"], me["y"], path, me["index"]))
            return state, linear, lateral

        heading_b = math.pi / 2 if b_dir > 0.0 else -math.pi / 2
        a = make(ax, 0.0, 0.0, (ax + 8.0, 0.0))
        b = make(2.0, by, heading_b, (2.0, by + 8.0 * b_dir))
        sa, sb = AvoidanceState(), AvoidanceState()
        stats = {
            "min_gap": self._square_face_gap(ax, 0.0, 0.0, 2.0, by, heading_b),
            "lat_a": 0.0, "lat_b": 0.0,
            "rev_a": 0, "rev_b": 0,
            "min_ax": ax, "max_by": by, "min_by": by,
        }
        for step in range(dwell + steps):
            now = step * 0.1
            move = step >= dwell
            snap_a = {"x": a["x"], "y": a["y"], "yaw": a["yaw"]}
            snap_b = {"x": b["x"], "y": b["y"], "yaw": b["yaw"]}
            sa, lin_a, lat_a = drive(a, snap_b, sa, now, move)
            sb, lin_b, lat_b = drive(b, snap_a, sb, now, move)
            stats["min_gap"] = min(
                stats["min_gap"],
                self._square_face_gap(a["x"], a["y"], a["yaw"], b["x"], b["y"], b["yaw"]),
            )
            stats["lat_a"] = max(stats["lat_a"], lat_a)
            stats["lat_b"] = max(stats["lat_b"], lat_b)
            if move and lin_a < -0.05:
                stats["rev_a"] += 1
            if move and lin_b < -0.05:
                stats["rev_b"] += 1
            stats["min_ax"] = min(stats["min_ax"], a["x"])
            stats["max_by"] = max(stats["max_by"], b["y"])
            stats["min_by"] = min(stats["min_by"], b["y"])
        stats["ax"], stats["ay"] = a["x"], a["y"]
        stats["bx"], stats["by"] = b["x"], b["y"]
        return stats

    def test_robots_already_beside_yield_as_soon_as_they_move(self):
        # Both are already stopped within lidar range, so each return is an
        # obstacle before either one has a speed. When they then cross, the
        # robot that has the other on its right backs up its own line and the
        # other keeps going. A fresh goal resets that memory; a dwell is the
        # same pair sitting still with the track already warm.
        def check(ax, by, direction, dwell, minimum_gap):
            stats = self._beside_cross(ax, by, direction, dwell=dwell)
            where = f"A={ax}, B={by}, dir={direction}, dwell={dwell}"
            self.assertGreater(stats["min_gap"], minimum_gap, where)
            self.assertLess(stats["lat_a"], 0.20, where)
            self.assertLess(stats["lat_b"], 0.20, where)
            self.assertGreater(stats["ax"], 2.40, where)
            self.assertLess(abs(stats["bx"] - 2.0), 0.35, where)
            if direction > 0.0:
                self.assertGreater(stats["by"], 0.80, where)
                self.assertGreater(stats["rev_a"], 4, where)
                self.assertLess(stats["min_ax"], ax - 0.04, where)
                self.assertLess(stats["rev_b"], 4, where)
            else:
                self.assertLess(stats["by"], -0.60, where)
                self.assertGreater(stats["rev_b"], 4, where)
                self.assertGreater(stats["max_by"], by + 0.04, where)
                self.assertLess(stats["rev_a"], 4, where)

        for dwell in (0, 15):
            check(1.6, -0.5, 1.0, dwell, 0.05)
            check(1.8, -0.4, 1.0, dwell, 0.0)
            check(1.6, 0.5, -1.0, dwell, 0.05)
            check(1.8, 0.4, -1.0, dwell, 0.05)

    def test_a_yielder_on_the_approach_does_not_arc_or_reverse_yet(self):
        # Still more than a metre out. A side pass here yaws the second
        # robot off its line, and a reverse starts before anyone is beside
        # the chassis.
        for ax, by, direction in (
            (1.80, 1.80, -1.0),
            (1.80, -1.80, 1.0),
        ):
            stats = self._square_cross(ax, 0.0, 2.0, by, b_dir=direction, steps=160)
            where = f"A from x={ax}, B from y={by}, dir={direction}"
            self.assertLess(stats["lat_a"], 0.20, where)
            self.assertLess(stats["lat_b"], 0.20, where)
            self.assertLess(stats["rev_early_a"], 3, where)
            self.assertLess(stats["rev_early_b"], 3, where)
            self.assertGreater(stats["min_d"], 0.55, where)

    def _square_gap(self, rx, ry, yaw, px, py, half=0.15, radius=0.25):
        dx, dy = px - rx, py - ry
        c, s = math.cos(yaw), math.sin(yaw)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        qx = min(half, max(-half, lx))
        qy = min(half, max(-half, ly))
        return math.hypot(lx - qx, ly - qy) - radius

    def _drive_yield(self, x, y, yaw, px, py, vx, vy, state, aim_x=1.4):
        dx, dy = px - x, py - y
        dist = math.hypot(dx, dy)
        bearing = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        reading = max(0.05, dist - 0.18)
        front = left = right = 4.0
        front_bearing = 0.0
        left_bearing = None
        right_bearing = None
        if abs(bearing) <= 0.61:
            front = reading
            front_bearing = bearing
        elif bearing > 0.61:
            left = reading
            left_bearing = bearing
        else:
            right = reading
            right_bearing = bearing
        error = math.atan2(math.sin(math.atan2(0.0 - y, aim_x - x) - yaw), math.cos(math.atan2(0.0 - y, aim_x - x) - yaw))
        command, state = step_avoidance(
            state, 0.0, x, y, yaw,
            1.0, 0.0, aim_x - x, 0.0 - y,
            front, left, right, error,
            map_range=6.0, map_left=6.0, map_right=6.0,
            path_x=x, path_y=0.0,
            bearing=front_bearing,
            left_bearing=left_bearing,
            right_bearing=right_bearing,
        )
        return command, state

    def test_a_walker_still_far_from_the_robot_is_crossed_in_front(self):
        # The line x = 0.22 runs through the chassis. Staying put, the 0.25 m
        # cylinder meets the square. The walker is still 1.2 m up that line,
        # on the left rather than in the forward cone, so there is time to
        # accelerate and leave the line before they arrive.
        px, py = 0.22, 1.20
        vx, vy = 0.0, -0.20
        self.assertLess(self._square_gap(0.0, 0.0, 0.0, px, 0.0), 0.0)
        state = AvoidanceState(
            track_x=px - vx * 0.1, track_y=py - vy * 0.1,
            other_speed=0.20, moving_frames=4,
        )
        x = y = yaw = 0.0
        min_gap = 99.0
        saw_fast = False
        crossed = False
        for _step in range(80):
            command, state = self._drive_yield(x, y, yaw, px, py, vx, vy, state)
            if command is None:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, 1.4, 0.0, 3.0, 0.30, 1.0, 0.45,
                    left=2.0, right=2.0, lateral=abs(y), align=False,
                )
            else:
                linear, angular = command
            if linear >= 0.40:
                saw_fast = True
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            px += vx * 0.1
            py += vy * 0.1
            gap = self._square_gap(x, y, yaw, px, py)
            min_gap = min(min_gap, gap)
            if x > px + 0.30 and py > y + 0.25:
                crossed = True
            if py < y - 0.50:
                break
        self.assertTrue(saw_fast)
        self.assertTrue(crossed)
        self.assertGreater(min_gap, 0.02)

    def test_a_close_yield_backs_off_and_faces_the_walker(self):
        # Already stopped, nose about 45° to the walker's line, and the
        # walker is too close for a sprint to clear them. Holding (0, 0)
        # leaves a corner in the cylinder.
        yaw = 0.80
        px, py = 0.34, 0.42
        vx, vy = 0.0, -0.22
        self.assertLess(self._square_gap(0.0, 0.0, yaw, px, 0.0), 0.0)
        state = AvoidanceState(
            active=True, side=0.0, started=0.0,
            track_x=px, track_y=py - vy * 0.1,
            other_speed=0.22, moving_frames=4,
        )
        x = y = 0.0
        min_gap = 99.0
        for step in range(50):
            state.started = 0.0
            command, state = self._drive_yield(x, y, yaw, px, py, vx, vy, state)
            self.assertIsNotNone(command)
            linear, angular = command
            self.assertLess(linear, 0.30)
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            px += vx * 0.1
            py += vy * 0.1
            min_gap = min(min_gap, self._square_gap(x, y, yaw, px, py))
            if py < y - 0.40:
                break
        self.assertGreater(min_gap, 0.02)
        # Velocity is straight down, so a flat face looks along x, yaw 0 or pi.
        err = abs(math.atan2(math.sin(yaw), math.cos(yaw)))
        err = min(err, abs(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))))
        self.assertLess(err, 0.40)

    def test_a_goal_behind_a_walker_on_the_right_stays_straight(self):
        # The walker is on our left and walking toward us. The goal is past
        # them and still on their right. Swinging left, across their front,
        # then coming back is the long way around.
        px, py = 1.60, 0.62
        vx, vy = -0.20, 0.0
        goal = (3.2, 0.0)
        state = AvoidanceState(
            track_x=px - vx * 0.1, track_y=py,
            track_vx=vx, track_vy=vy,
            other_speed=0.20, moving_frames=5,
        )
        x = y = yaw = 0.0
        min_gap = 99.0
        max_abs_yaw = 0.0
        max_y = 0.0
        for step in range(50):
            dx, dy = px - x, py - y
            dist = math.hypot(dx, dy)
            bearing = math.atan2(
                math.sin(math.atan2(dy, dx) - yaw),
                math.cos(math.atan2(dy, dx) - yaw),
            )
            reading = max(0.05, dist - 0.18)
            front = left = right = 4.0
            front_bearing = 0.0
            left_bearing = None
            right_bearing = None
            if abs(bearing) <= 0.61:
                front = reading
                front_bearing = bearing
            elif bearing > 0.61:
                left = reading
                left_bearing = bearing
            else:
                right = reading
                right_bearing = bearing
            # The live aim is the 0.80 m path carrot. The goal, behind the
            # walker and still on their right, is a separate point.
            error = math.atan2(
                math.sin(math.atan2(0.0 - y, 0.80) - yaw),
                math.cos(math.atan2(0.0 - y, 0.80) - yaw),
            )
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                1.0, 0.0, 0.80, 0.0 - y,
                front, left, right, error,
                map_range=8.0, map_left=8.0, map_right=8.0,
                path_x=x, path_y=0.0,
                bearing=front_bearing,
                left_bearing=left_bearing,
                right_bearing=right_bearing,
                dest_x=goal[0] - x, dest_y=goal[1] - y,
            )
            if state.active and state.side < 0.0:
                self.fail("swung left across the walker")
            if command is None:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, goal[0], goal[1], min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=2.0, right=2.0, lateral=abs(y), align=False,
                )
            else:
                linear, angular = command
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            px += vx * 0.1
            py += vy * 0.1
            min_gap = min(min_gap, self._square_gap(x, y, yaw, px, py))
            max_abs_yaw = max(max_abs_yaw, abs(yaw))
            max_y = max(max_y, y)
        self.assertGreater(x, 1.0)
        self.assertLess(max_abs_yaw, 0.45)
        self.assertLess(max_y, 0.20)
        self.assertGreater(min_gap, 0.02)

    def _walker_scan(self, x, y, yaw, px, py, radius=0.25):
        lx = x + 0.15 * math.cos(yaw)
        ly = y + 0.15 * math.sin(yaw)
        samples = 360
        ranges = []
        for index in range(samples):
            angle = -math.pi + index * (2.0 * math.pi / samples)
            direction = yaw + angle
            dx, dy = math.cos(direction), math.sin(direction)
            fx, fy = lx - px, ly - py
            closest = fx * dx + fy * dy
            disc = closest * closest - (fx * fx + fy * fy - radius * radius)
            hit = float("inf")
            if disc >= 0.0:
                distance = -closest - math.sqrt(disc)
                if 0.15 < distance <= 3.0:
                    hit = distance
            ranges.append(hit)
        return sector_distances_from_scan(
            ranges, -math.pi, 2.0 * math.pi / samples, range_min=0.15,
        )

    def _cross_oncoming_walker(self, lat, vx, vy, goal=(5.0, 0.0)):
        """Drive at a walker coming the other way. Return min gap and side offset."""
        state = AvoidanceState()
        x = y = yaw = 0.0
        px, py = 2.2, lat
        min_gap = 99.0
        max_offset = 0.0
        held = 99.0
        committed = False
        for step in range(100):
            sectors = self._walker_scan(x, y, yaw, px, py)
            path = [(0.0, 0.0), (8.0, 0.0)]
            aim = path_merge_target(x, y, path, 0, 0.0, 0.0)
            error = math.atan2(
                math.sin(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                math.cos(math.atan2(aim[1] - y, aim[0] - x) - yaw),
            )
            front = sectors.front
            left = sectors.left if sectors.left < 900.0 else 4.0
            right = sectors.right if sectors.right < 900.0 else 4.0
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                1.0, 0.0, aim[0] - x, aim[1] - y,
                front if front < 900.0 else 999.0,
                left, right, error,
                map_range=6.0, map_left=6.0, map_right=6.0,
                path_x=0.0, path_y=0.0,
                bearing=sectors.front_angle,
                left_bearing=None if sectors.left > 900.0 else sectors.left_angle,
                right_bearing=None if sectors.right > 900.0 else sectors.right_angle,
                dest_x=goal[0] - x, dest_y=goal[1] - y,
            )
            if command is None:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, aim[0], aim[1], min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=min(left, 3.0), right=min(right, 3.0),
                    lateral=abs(y), align=False,
                )
            else:
                linear, angular = command
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            px += vx * 0.1
            py += vy * 0.1
            min_gap = min(min_gap, self._square_gap(x, y, yaw, px, py))
            max_offset = max(max_offset, abs(y))
            # Once the offset is taken, coming back while the walker is still
            # ahead is the collision. Record the tightest offset until they
            # are behind the chassis.
            if abs(y) > 0.50:
                committed = True
            if committed and (px - x) > -0.45:
                held = min(held, abs(y))
            if px < x - 0.8 and step > 15:
                break
        return min_gap, max_offset, held

    def test_a_head_on_walker_does_not_reverse_when_the_hit_jumps(self):
        # One sideways jump of the return used to latch the robot yield.
        # The chassis then backed up on the centreline for as long as the
        # walker kept coming, instead of taking its own right.
        state = AvoidanceState()
        x = y = yaw = 0.0
        px, py = 2.2, 0.0
        path = [(i * 0.2, 0.0) for i in range(40)]
        index = 0
        cruise = False
        reverses = 0
        max_offset = 0.0
        for step in range(90):
            while index < len(path) - 1:
                waypoint = path[index]
                if math.hypot(waypoint[0] - x, waypoint[1] - y) < 0.25:
                    index += 1
                    continue
                heading = path_heading(path, index, waypoint[0], waypoint[1], x, y)
                along_wp = (
                    (x - waypoint[0]) * math.cos(heading)
                    + (y - waypoint[1]) * math.sin(heading)
                )
                if along_wp > 0.0:
                    index += 1
                    continue
                break
            kick = 0.12 if step == 18 else 0.0
            sectors = self._walker_scan(x, y, yaw, px, py + kick)
            waypoint = path[index]
            aim = path_merge_target(x, y, path, index, waypoint[0], waypoint[1])
            heading = path_heading(path, index, waypoint[0], waypoint[1], x, y)
            lateral = path_lateral(x, y, path, index)
            error = math.atan2(
                math.sin(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                math.cos(math.atan2(aim[1] - y, aim[0] - x) - yaw),
            )

            def sector(value):
                return value if value < 900.0 else 999.0

            front = sector(sectors.front)
            left = sector(sectors.left)
            right = sector(sectors.right)
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                math.cos(heading), math.sin(heading),
                aim[0] - x, aim[1] - y,
                front, left, right, error,
                map_range=3.0, map_left=3.0, map_right=3.0,
                path_x=waypoint[0], path_y=waypoint[1],
                bearing=sectors.front_angle,
                left_bearing=sectors.left_angle,
                right_bearing=sectors.right_angle,
                dest_x=6.0 - x, dest_y=-y,
            )
            if command is None:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, aim[0], aim[1], min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=min(left, 3.0), right=min(right, 3.0),
                    lateral=lateral, align=not cruise,
                )
                if abs(error) <= 0.30:
                    cruise = True
            else:
                linear, angular = command
            if linear < -0.05:
                reverses += 1
            max_offset = max(max_offset, abs(y))
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            px -= 0.20 * 0.1
            if px < x - 0.8 and step > 20:
                break
        self.assertLess(reverses, 3)
        self.assertGreater(max_offset, 0.45)
        self.assertEqual(state.cross_hold, 0.0)

    def test_a_head_on_walker_clears_the_side(self):
        # Coming toward the robot, on the line and just off it. The pass used
        # to yaw them out of the forward cone, give up the offset, and let
        # the cylinder meet the side. A later return to the centreline, while
        # they are still in front, is the same collision.
        for lat in (0.0, 0.15, -0.15, 0.30, -0.30, 0.45, -0.45):
            gap, offset, held = self._cross_oncoming_walker(lat, -0.20, 0.0)
            self.assertGreater(gap, 0.10, f"lateral {lat}")
            if abs(lat) <= 0.15:
                self.assertGreater(offset, 0.55, f"lateral {lat}")
                self.assertGreater(held, 0.45, f"lateral {lat}")
        # A small angle toward the path is still a side, not a graze.
        gap, _offset, _held = self._cross_oncoming_walker(0.50, -0.18, -0.08)
        self.assertGreater(gap, 0.10)
        gap, _offset, _held = self._cross_oncoming_walker(-0.50, -0.18, 0.08)
        self.assertGreater(gap, 0.10)

    def test_a_head_on_offset_holds_when_the_walker_leaves_the_scan(self):
        # The offset is already taken. The walker is still in front, but the
        # yaw has carried them out of the forward cone and the side sector.
        # An empty scan used to end the pass, and the return to the line met them.
        state = AvoidanceState(
            active=True, side=1.0, started=0.0, until=1.9,
            track_x=0.70, track_y=0.0, track_vx=-0.20, track_vy=0.0,
            other_speed=0.20, moving_frames=6, seen_off=0.0,
        )
        command, state = step_avoidance(
            state, 2.0, 0.0, -0.65, 0.0,
            1.0, 0.0, 0.80, 0.65,
            front=999.0, left=999.0, right=999.0,
            heading_error=0.70,
            map_range=6.0, map_left=6.0, map_right=6.0,
            path_x=0.0, path_y=0.0,
            dest_x=4.0, dest_y=0.65,
        )
        self.assertIsNotNone(command)
        self.assertTrue(state.active)
        self.assertGreater(state.side, 0.0)
        self.assertGreater(command[0], 0.0)

    def test_a_clear_offset_pose_does_not_rejoin_while_the_walker_is_ahead(self):
        # From the offset pose the straight run looks clear, so the pass was
        # dropped and the centreline carrot pulled the chassis back across them.
        bearing = 0.82
        rang = 0.70
        x, y, yaw = 0.0, -0.55, 0.0
        cx = x + (rang + 0.18) * math.cos(yaw + bearing)
        cy = y + (rang + 0.18) * math.sin(yaw + bearing)
        state = AvoidanceState(
            active=True, side=1.0, started=0.0, until=1.9,
            track_x=cx + 0.02, track_y=cy, track_vx=-0.20, track_vy=0.0,
            other_speed=0.20, moving_frames=8, seen_off=0.25,
        )
        command, state = step_avoidance(
            state, 2.0, x, y, yaw,
            1.0, 0.0, 0.80, 0.55,
            front=999.0, left=rang, right=3.0,
            heading_error=0.50,
            map_range=6.0, map_left=6.0, map_right=6.0,
            path_x=0.0, path_y=0.0,
            bearing=0.0, left_bearing=bearing,
            dest_x=4.0, dest_y=0.55,
        )
        self.assertIsNotNone(command)
        self.assertTrue(state.active)
        self.assertGreater(state.side, 0.0)
        self.assertGreater(command[0], 0.0)

    def test_a_head_on_offset_is_not_dropped_when_the_hit_slides(self):
        # The cylinder's closest point slides sideways as the robot yaws, so
        # the track is no longer "along the path". The walker then leaves the
        # 110° sector while still in front of the rear face. Dropping the
        # offset there steers back onto the centreline and the next yaw swings
        # a rear corner into them. Empty sectors are 999, the same value the
        # live scan publishes, and the map ray is the 3 m lidar limit.
        state = AvoidanceState()
        x = y = yaw = 0.0
        px, py = 2.2, 0.0
        vx, vy = -0.20, 0.0
        goal = (6.0, 0.0)
        path = [(i * 0.2, 0.0) for i in range(40)]
        index = 0
        cruising = False
        committed = False
        dropped = False
        restarts = 0
        was_active = False
        min_rear = 99.0
        min_gap = 99.0
        for step in range(120):
            while index < len(path) - 1:
                waypoint = path[index]
                if math.hypot(waypoint[0] - x, waypoint[1] - y) < 0.25:
                    index += 1
                    continue
                heading = path_heading(path, index, waypoint[0], waypoint[1], x, y)
                along_wp = (x - waypoint[0]) * math.cos(heading) + (y - waypoint[1]) * math.sin(heading)
                if along_wp > 0.0:
                    index += 1
                    continue
                break
            sectors = self._walker_scan(x, y, yaw, px, py)
            waypoint = path[index]
            aim = path_merge_target(x, y, path, index, waypoint[0], waypoint[1])
            heading = path_heading(path, index, waypoint[0], waypoint[1], x, y)
            lateral = path_lateral(x, y, path, index)
            error = math.atan2(
                math.sin(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                math.cos(math.atan2(aim[1] - y, aim[0] - x) - yaw),
            )

            def sector(value):
                return value if value < 900.0 else 999.0

            front = sector(sectors.front)
            left = sector(sectors.left)
            right = sector(sectors.right)
            true_along = (px - x) * math.cos(heading) + (py - y) * math.sin(heading)
            command, state = step_avoidance(
                state, step * 0.1, x, y, yaw,
                math.cos(heading), math.sin(heading),
                aim[0] - x, aim[1] - y,
                front, left, right, error,
                map_range=3.0, map_left=3.0, map_right=3.0,
                path_x=waypoint[0], path_y=waypoint[1],
                bearing=sectors.front_angle,
                left_bearing=sectors.left_angle,
                right_bearing=sectors.right_angle,
                dest_x=goal[0] - x, dest_y=goal[1] - y,
            )
            if command is None:
                linear, angular, _stalling = command_on_path(
                    x, y, yaw, aim[0], aim[1], min(front, 3.0),
                    0.30, 1.0, 0.45,
                    left=min(left, 3.0), right=min(right, 3.0),
                    lateral=lateral, align=not cruising,
                )
            else:
                linear, angular = command
            if command is None and abs(error) <= 0.30:
                cruising = True
            if lateral > 0.50:
                committed = True
            if state.active and not was_active and committed:
                restarts += 1
            # Centre still ahead of the rear face: the 0.25 m cylinder reaches
            # x = -0.15 on the robot. Rejoining here is the second pivot.
            if committed and true_along > -0.40 and was_active and not state.active:
                dropped = True
            was_active = state.active
            yaw += angular * 0.1
            x += linear * math.cos(yaw) * 0.1
            y += linear * math.sin(yaw) * 0.1
            px += vx * 0.1
            py += vy * 0.1
            min_gap = min(min_gap, self._square_gap(x, y, yaw, px, py))
            min_rear = min(min_rear, self._rear_gap(x, y, yaw, px, py))
            if true_along < -0.9 and step > 20:
                break
        self.assertTrue(committed)
        self.assertFalse(dropped)
        self.assertEqual(restarts, 0)
        self.assertGreater(min_gap, 0.05)
        self.assertGreater(min_rear, 0.05)

    def test_a_shelf_corner_does_not_hide_a_head_on_walker(self):
        # The closest front hit is a mapped shelf corner. The walker is the
        # next ray in the same sector. Using only the minimum never starts
        # the offset, and the yaw then begins too late for the rear corner.
        inc = 2.0 * math.pi / 360
        ranges = [3.0] * 360

        def at(angle: float) -> int:
            return int(round((angle + math.pi) / inc)) % 360

        ranges[at(0.0)] = 1.70
        ranges[at(0.40)] = 0.70

        def map_at(angle: float) -> float:
            if abs(angle - 0.40) < 0.08:
                return 0.70
            return 3.0

        body = sector_body_returns(ranges, -math.pi, inc, map_at, range_min=0.15)
        raw = sector_distances_from_scan(ranges, -math.pi, inc, range_min=0.15)
        self.assertAlmostEqual(raw.front, 0.70, places=2)
        self.assertAlmostEqual(body.front, 1.70, places=2)
        hidden = AvoidanceState(closing=-0.25, last_front=0.80, approach_frames=4)
        command, hidden = step_avoidance(
            hidden, 0.4, 0.0, 0.0, 0.0,
            1.0, 0.0, 1.5, 0.0,
            front=0.70, left=2.5, right=0.55,
            heading_error=0.0,
            map_range=0.70, map_right=0.55,
            path_x=0.0, path_y=0.0,
            bearing=0.40,
        )
        self.assertFalse(hidden.active)
        self.assertIsNone(command)
        seen = AvoidanceState(closing=-0.25, last_front=1.90, approach_frames=4)
        command, seen = step_avoidance(
            seen, 0.4, 0.0, 0.0, 0.0,
            1.0, 0.0, 1.5, 0.0,
            front=0.70, left=2.5, right=0.55,
            heading_error=0.0,
            map_range=3.0, map_right=0.55,
            path_x=0.0, path_y=0.0,
            bearing=0.40,
            open_front=body.front,
            open_bearing=body.front_angle,
        )
        self.assertIsNotNone(command)
        self.assertTrue(seen.active)
        self.assertLess(seen.side, 0.0)

    def _rear_gap(self, rx, ry, yaw, px, py, half=0.15, radius=0.25):
        c, s = math.cos(yaw), math.sin(yaw)
        best = 99.0
        for ly in (-half, half):
            wx = rx + c * (-half) - s * ly
            wy = ry + s * (-half) + c * ly
            best = min(best, math.hypot(px - wx, py - wy) - radius)
        return best

    def test_a_head_on_offset_ends_once_the_walker_is_behind(self):
        state = AvoidanceState(
            active=True, side=1.0, started=0.0, until=1.9,
            track_x=-0.80, track_y=0.0, track_vx=-0.20, track_vy=0.0,
            other_speed=0.20, moving_frames=6, seen_off=0.0,
        )
        command, state = step_avoidance(
            state, 2.0, 0.0, -0.65, 0.0,
            1.0, 0.0, 0.80, 0.65,
            front=999.0, left=999.0, right=999.0,
            heading_error=0.40,
            map_range=6.0, map_left=6.0, map_right=6.0,
            path_x=0.0, path_y=0.0,
            dest_x=4.0, dest_y=0.65,
        )
        self.assertIsNone(command)
        self.assertFalse(state.active)

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

    def test_a_stationary_robot_off_the_path_does_not_stop_the_passer(self):
        # The other chassis is parked to the right of a straight eastbound
        # leg. It is not on the line and it does not move. Driving past slides
        # the lidar hit, and one cross-track sample used to latch a yield.
        # The parked robot never crosses the path, so the stop never ended.
        for lateral in (0.40, 1.10, 2.80):
            other = (1.6, -lateral)
            path = [(i * 0.2, 0.0) for i in range(30)]
            state = AvoidanceState()
            x = y = yaw = 0.0
            index = 0
            stopped = 0
            yielded = 0
            for step in range(120):
                now = step * 0.1
                while index < len(path) - 1 and math.hypot(path[index][0] - x, path[index][1] - y) < 0.25:
                    index += 1
                look = path_lookahead(path, index, x, y, 0.60)
                aim = path_merge_target(x, y, path, index, look[0], look[1])
                heading = path_heading(path, index, look[0], look[1], x, y)
                ranges, inc = self._square_robot_ranges(x, y, yaw, other[0], other[1], 0.0)
                sectors = sector_distances_from_scan(ranges, -math.pi, inc, range_min=0.15)
                front = sectors.front if sectors.front < 900.0 else 999.0
                left = sectors.left if sectors.left < 900.0 else 999.0
                right = sectors.right if sectors.right < 900.0 else 999.0
                error = math.atan2(
                    math.sin(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                    math.cos(math.atan2(aim[1] - y, aim[0] - x) - yaw),
                )
                command, state = step_avoidance(
                    state, now, x, y, yaw,
                    math.cos(heading), math.sin(heading),
                    aim[0] - x, aim[1] - y,
                    front, left, right, error,
                    map_range=3.0, map_left=3.0, map_right=3.0,
                    path_x=look[0], path_y=look[1],
                    bearing=sectors.front_angle,
                    left_bearing=sectors.left_angle,
                    right_bearing=sectors.right_angle,
                    dest_x=5.0 - x, dest_y=0.0 - y,
                )
                if command is None:
                    linear, angular, _stalling = command_on_path(
                        x, y, yaw, aim[0], aim[1], min(front, 3.0),
                        0.30, 1.0, 0.45,
                        left=min(left, 3.0), right=min(right, 3.0),
                        lateral=path_lateral(x, y, path, index),
                        align=False,
                    )
                else:
                    linear, angular = command
                if linear < 0.05:
                    stopped += 1
                if state.cross_hold != 0.0:
                    yielded += 1
                yaw += angular * 0.1
                x += linear * math.cos(yaw) * 0.1
                y += linear * math.sin(yaw) * 0.1
            self.assertLess(stopped, 5, lateral)
            self.assertLess(yielded, 5, lateral)
            self.assertGreater(x, other[0] + 0.40, lateral)

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
