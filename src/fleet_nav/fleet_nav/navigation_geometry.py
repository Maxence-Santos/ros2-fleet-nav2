"""
Pure geometry helpers for the shared warehouse planner.

This is the A* and scan-sector code from ros2-slam-nav2, copied so this
repository stays buildable on its own. Kept free of ROS imports.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


# LiDAR sector half-angles (radians) matching nav_controller.scan_callback
FRONT_HALF_ANGLE = 0.61   # ±35°
LEFT_MAX_ANGLE = 1.92     # +110°
RIGHT_MIN_ANGLE = -1.92   # -110°
REAR_ABS_ANGLE = 2.35     # ±135°


@dataclass(frozen=True)
class SectorDistances:
    """Minimum range (m) in each 360° sector. 999.0 means no return."""

    front: float
    left: float
    right: float
    rear: float
    front_angle: float = 0.0
    left_angle: float = 1.27
    right_angle: float = -1.27


def sector_distances_from_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.0,
    empty_value: float = 999.0,
) -> SectorDistances:
    """
    Split a LaserScan-like array into four sector minima.

    Sectors (robot frame, 0 = forward, CCW positive):
      - front : |angle| <= 35°
      - left  :  35° < angle <= 110°
      - right : -110° <= angle < -35°
      - rear  : |angle| > 135°
    """
    front_min = empty_value
    left_min = empty_value
    right_min = empty_value
    rear_min = empty_value
    front_angle = 0.0
    left_angle = 1.27
    right_angle = -1.27

    for i, r in enumerate(ranges):
        if r is None or math.isinf(r) or math.isnan(r) or r < range_min:
            continue
        angle = angle_min + i * angle_increment

        if -FRONT_HALF_ANGLE <= angle <= FRONT_HALF_ANGLE and r < front_min:
            front_min = r
            front_angle = angle
        elif FRONT_HALF_ANGLE < angle <= LEFT_MAX_ANGLE and r < left_min:
            left_min = r
            left_angle = angle
        elif RIGHT_MIN_ANGLE <= angle < -FRONT_HALF_ANGLE and r < right_min:
            right_min = r
            right_angle = angle
        elif abs(angle) > REAR_ABS_ANGLE:
            rear_min = min(rear_min, r)

    return SectorDistances(
        front=front_min,
        left=left_min,
        right=right_min,
        rear=rear_min,
        front_angle=front_angle,
        left_angle=left_angle,
        right_angle=right_angle,
    )


def frontal_blob(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.0,
    close: float = 2.00,
    gap: float = 0.18,
) -> Tuple[float, float, float] | None:
    """Closest compact cluster in front, as ``(angle_min, angle_max, nearest)``.

    Hits separated by more than ``gap`` radians stay in different clusters, so
    two shelves beside a clear corridor are not merged into one obstacle on
    the straight line. The returned cluster is the one nearest the forward axis.
    """
    hits: List[Tuple[float, float]] = []
    for i, reading in enumerate(ranges):
        if reading is None or math.isinf(reading) or math.isnan(reading):
            continue
        if reading < range_min or reading >= close:
            continue
        angle = angle_min + i * angle_increment
        if abs(angle) > 1.57:
            continue
        hits.append((angle, reading))
    if not hits:
        return None
    hits.sort()
    clusters: List[List[Tuple[float, float]]] = [[hits[0]]]
    for angle, reading in hits[1:]:
        if angle - clusters[-1][-1][0] > gap:
            clusters.append([])
        clusters[-1].append((angle, reading))

    def bearing(cluster: List[Tuple[float, float]]) -> float:
        return 0.5 * (cluster[0][0] + cluster[-1][0])

    cluster = min(clusters, key=lambda item: abs(bearing(item)))
    nearest = min(reading for _, reading in cluster)
    return cluster[0][0], cluster[-1][0], nearest


def robot_pass_side(
    nearest: float,
    width: float,
    bearing: float,
    left: float,
    right: float,
    engaged_side: float | None = None,
    engage: float = 1.80,
    release: float = 2.30,
    min_width: float = 0.12,
    max_width: float = 0.90,
    max_bearing: float = 0.40,
    far_bearing: float = 0.28,
    far_range: float = 1.20,
    room: float = 0.45,
    path_offset: float = 0.0,
    max_offset: float = 0.30,
) -> float | None:
    """+1 to pass on the right, -1 on the left, None when the way ahead is clear.

    A stopped robot on the path counts from ``engage`` metres, the same range
    the first project uses for a pedestrian. ``path_offset`` is that return's
    angle off the path, not off the nose: while the robot is still turning
    onto the line, a shelf beside the path swings in front of the nose and
    must not start a pass. Once a side is chosen it stays chosen.
    Right is preferred, so two robots facing each other offset to opposite
    sides of the crossing.
    """
    # Keep the chosen side while that robot is still ahead. Once it has slid
    # out to the flank, the straight-line follower takes over. Holding the
    # offset for every return inside ``release`` is what made the robot orbit.
    if (
        engaged_side is not None
        and nearest < release
        and abs(bearing) <= 1.20
        and width <= 1.20
    ):
        return engaged_side
    if abs(path_offset) > max_offset:
        return None
    bearing_limit = far_bearing if nearest >= far_range else max_bearing
    compact = (
        nearest < engage
        and min_width <= width <= max_width
        and abs(bearing) <= bearing_limit
    )
    if not compact:
        return None
    if right >= room:
        return 1.0
    if left >= room:
        return -1.0
    return None


def hold_pass_side(
    detected: float | None,
    active: float | None,
    clear_since: float | None,
    now: float,
    hold: float = 0.6,
) -> Tuple[float | None, float | None]:
    """Keep the chosen side briefly after the other robot leaves the forward scan."""
    if detected is not None:
        return detected, None
    if active is None:
        return None, None
    if clear_since is None:
        clear_since = now
    if now - clear_since < hold:
        return active, clear_since
    return None, None


def bearing_off_path(yaw: float, blob_bearing: float, path_heading_rad: float) -> float:
    """Angle of a lidar return relative to the path direction."""
    delta = yaw + blob_bearing - path_heading_rad
    return math.atan2(math.sin(delta), math.cos(delta))


def path_heading(
    path: Sequence[Tuple[float, float]],
    index: int,
    fallback_x: float,
    fallback_y: float,
    x: float,
    y: float,
) -> float:
    """Direction of the current path leg, or toward ``fallback`` when the leg is empty.

    The leg is followed until it is at least 0.20 m long. A single grid step
    alternates between straight and diagonal, and that 45° flip was a stop
    at every cell along an edge.
    """
    if path:
        i = max(0, min(index, len(path) - 1))
        nxt = i
        traveled = 0.0
        prev_x, prev_y = path[i]
        while nxt < len(path) - 1 and traveled < 0.20:
            nxt += 1
            traveled += math.hypot(path[nxt][0] - prev_x, path[nxt][1] - prev_y)
            prev_x, prev_y = path[nxt]
        dx = path[nxt][0] - path[i][0]
        dy = path[nxt][1] - path[i][1]
        if dx * dx + dy * dy > 1e-8:
            return math.atan2(dy, dx)
    return math.atan2(fallback_y - y, fallback_x - x)


# Chassis is 0.30 m square (corner at 0.21 m). Wheels reach 0.20 m.
# 0.25 m is the closest the centre may pass an obstacle and still fit.
ROBOT_RADIUS = 0.25


def simplify_path(
    path: Sequence[Tuple[float, float]],
    clearance,
    segment_free,
    margin: float = ROBOT_RADIUS,
    resolution: float = 0.02,
    epsilon: float = 0.05,
) -> List[Tuple[float, float]]:
    """Drop the grid staircase without cutting a corner.

    A straight and a diagonal step alternate, so the heading flips by 45° on
    every cell. Points within ``epsilon`` of a straight line collapse, but
    only when that line stays as far from obstacles as the cells it replaces.
    A chord that shaves the shelf is refused: the detour around the edge stays.
    """
    if len(path) < 3:
        return list(path)
    step_m = max(resolution, 0.02)
    points = list(path)

    def chord_ok(start: Tuple[float, float], end: Tuple[float, float], floor: float) -> bool:
        if not segment_free(start[0], start[1], end[0], end[1]):
            return False
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if length < 1e-6:
            return True
        samples = max(1, int(math.ceil(length / step_m)))
        for sample in range(1, samples):
            blend = sample / samples
            x = start[0] + (end[0] - start[0]) * blend
            y = start[1] + (end[1] - start[1]) * blend
            if clearance(x, y) < floor - 1e-6:
                return False
        return True

    def recurse(span: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if len(span) < 3:
            return span
        ax, ay = span[0]
        bx, by = span[-1]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        farthest = 0.0
        index = 1
        for i in range(1, len(span) - 1):
            if length < 1e-6:
                dist = math.hypot(span[i][0] - ax, span[i][1] - ay)
            else:
                dist = abs((span[i][0] - ax) * dy - (span[i][1] - ay) * dx) / length
            if dist > farthest:
                farthest = dist
                index = i
        nearest = min(clearance(px, py) for px, py in span)
        floor = max(min(margin, nearest), nearest - resolution)
        if farthest <= epsilon and chord_ok(span[0], span[-1], floor):
            return [span[0], span[-1]]
        return recurse(span[: index + 1])[:-1] + recurse(span[index:])

    return resample_path(recurse(points), spacing=0.35)


def resample_path(
    path: Sequence[Tuple[float, float]],
    spacing: float = 0.35,
) -> List[Tuple[float, float]]:
    """Points about ``spacing`` apart, so the follower can still catch each one.

    A handful of corners is a feasible line, but the tracker only advances
    when it comes within 0.25 m of the current point. Longer gaps let it
    miss the corner and keep the old heading.
    """
    if len(path) < 2:
        return list(path)
    points: List[Tuple[float, float]] = [path[0]]
    for start, end in zip(path, path[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 1e-8:
            continue
        # Keep the corner. Splitting a long leg must not skip it, or the
        # new chord cuts back toward the shelf.
        steps = max(1, int(math.ceil(length / spacing)))
        for step in range(1, steps + 1):
            blend = step / steps
            points.append((
                start[0] + dx * blend,
                start[1] + dy * blend,
            ))
    return points


def path_lateral(
    x: float,
    y: float,
    path: Sequence[Tuple[float, float]],
    index: int,
) -> float:
    """Distance in metres from the robot to the current path leg."""
    if not path:
        return 0.0
    i = max(0, min(index, len(path) - 1))
    px, py = path[i]
    heading = path_heading(path, index, px, py, x, y)
    dx = x - px
    dy = y - py
    return abs(dx * math.sin(heading) - dy * math.cos(heading))


def path_merge_target(
    x: float,
    y: float,
    path: Sequence[Tuple[float, float]],
    index: int,
    path_x: float,
    path_y: float,
    distance: float = 0.80,
) -> Tuple[float, float]:
    """A point ahead on the path centreline, so the cruise returns to the line.

    The pass carrot keeps whatever rightward offset the robot already has.
    Reusing it outside an avoidance left the robot on that parallel. The lead
    is long enough that the correction is an arc, not a stop-and-yaw.
    """
    heading = path_heading(path, index, path_x, path_y, x, y)
    forward_x = math.cos(heading)
    forward_y = math.sin(heading)
    along = (x - path_x) * forward_x + (y - path_y) * forward_y
    ahead = along + distance
    return (
        path_x + ahead * forward_x,
        path_y + ahead * forward_y,
    )


def passing_target(
    x: float,
    y: float,
    heading: float,
    side: float,
    distance: float = 1.20,
    lateral: float = 0.60,
    clearance: float = 999.0,
    margin: float = 0.28,
    path_x: float | None = None,
    path_y: float | None = None,
) -> Tuple[float, float]:
    """A point on the line parallel to ``heading``, ``lateral`` metres to one side.

    The parallel is taken through ``(path_x, path_y)`` (a point on the path).
    Chasing a point fixed to the robot's nose, far off to the side, makes the
    heading wrap and the robot orbit that point. ``side`` > 0 is right.
    ``lateral`` shrinks when the chosen flank is a wall.
    """
    if path_x is None:
        path_x = x
    if path_y is None:
        path_y = y
    forward_x = math.cos(heading)
    forward_y = math.sin(heading)
    right_x = math.sin(heading)
    right_y = -math.cos(heading)
    # ``clearance`` is the lidar range from the robot. Treating it as a cap on
    # the distance from the path pulls a robot that is already on the safe
    # side back toward the one it is passing.
    right_offset = (x - path_x) * right_x + (y - path_y) * right_y
    on_side = right_offset if side > 0.0 else -right_offset
    room = max(0.0, clearance - margin)
    max_offset = max(0.0, on_side) + room
    wanted = max(lateral, on_side) if on_side > 0.0 else lateral
    offset = min(wanted, max_offset)
    sign = 1.0 if side > 0.0 else -1.0
    along = (x - path_x) * forward_x + (y - path_y) * forward_y
    ahead = along + distance
    return (
        path_x + ahead * forward_x + sign * offset * right_x,
        path_y + ahead * forward_y + sign * offset * right_y,
    )


def passing_command(
    x: float,
    y: float,
    yaw: float,
    target_x: float,
    target_y: float,
    front: float,
    max_linear: float,
    max_angular: float,
) -> Tuple[float, float]:
    """Steer onto the offset line while keeping forward speed.

    The turn is capped. Full ``max_angular`` on a carrot 0.7 m to the side
    closed the loop into a circle instead of a straight pass.
    """
    heading = math.atan2(target_y - y, target_x - x)
    error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
    cap = min(max_angular, 0.55)
    angular = max(-cap, min(cap, 1.6 * error))
    if front < 0.22:
        return max_linear * 0.40, angular
    return max_linear * 0.90, angular


def parallel_target(
    x: float,
    y: float,
    yaw: float,
    left: float,
    right: float,
    distance: float,
    bias: float = 0.35,
) -> Tuple[float, float]:
    """A point just ahead, nudged toward the clearer flank.

    Used when every path chord crosses an obstacle. Steering here keeps the
    robot parallel to that obstacle instead of yawing back into it.
    """
    # Both flanks open: a constant bias on the current yaw integrates into a circle.
    # Both flanks tight: the longer reading is still a wall. Bias would yaw into it.
    if (left >= 0.80 and right >= 0.80) or (left < 1.0 and right < 1.0):
        side = 0.0
    else:
        side = bias if left >= right else -bias
    heading = yaw + side
    return (x + math.cos(heading) * distance, y + math.sin(heading) * distance)


def path_lookahead(
    path: Sequence[Tuple[float, float]],
    index: int,
    x: float,
    y: float,
    distance: float,
) -> Tuple[float, float] | None:
    """Point about ``distance`` metres ahead along the path, else the last point."""
    if not path:
        return None
    index = max(0, min(index, len(path) - 1))
    traveled = 0.0
    prev_x, prev_y = x, y
    chosen = path[index]
    for point in path[index:]:
        traveled += math.hypot(point[0] - prev_x, point[1] - prev_y)
        prev_x, prev_y = point
        chosen = point
        if traveled >= distance:
            break
    return chosen


def pursuit_command(
    x: float,
    y: float,
    yaw: float,
    target_x: float,
    target_y: float,
    front: float,
    max_linear: float,
    max_angular: float,
    stop_distance: float,
    align_heading: float = 0.45,
    left: float = 999.0,
    right: float = 999.0,
    pivot_room: float = 0.30,
) -> Tuple[float, float, bool]:
    """Linear and angular speeds, plus whether the robot is boxed in.

    A clear front, or another robot only beside the chassis, is a moving turn
    back onto the path. Stopping there is the in-place spin after an avoidance.
    Translation stops for a real corner: one flank inside 0.50 m and the other
    at least 0.80 m clear. The robot then yaws at full rate toward that open
    side (left on a tie). A longer flank of only 0.30 to 0.80 m is the wall of
    a shelf gap, so the robot keeps creeping along the path. The stall flag is
    set only when both flanks are tighter than ``pivot_room``.
    """
    heading = math.atan2(target_y - y, target_x - x)
    error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
    angular = max(-max_angular, min(max_angular, 1.4 * error))

    def moving(scale: float) -> Tuple[float, float, bool]:
        cap = min(max_angular, 0.70)
        turn = max(-cap, min(cap, 1.4 * error))
        # ``align_heading`` used to zero translation. It now only slows the arc.
        blend = min(abs(error) / align_heading, 1.0) if align_heading > 0.0 else 1.0
        linear = max_linear * scale * (1.0 - 0.40 * blend)
        return linear, turn, False

    if front >= stop_distance:
        return moving(1.0)

    # Both flanks open: the return is not a wall. Keep crawling so a straight
    # line is not abandoned for a spin.
    if left >= 0.80 and right >= 0.80:
        crawl = 0.50 if front < 0.35 else 0.75
        return moving(crawl)

    tighter = min(left, right)
    opener = max(left, right)
    if tighter < 0.50 and opener >= 0.80:
        # Full rate, chosen from clearance rather than the heading error.
        # A small error means the lookahead chord still points into the corner.
        if left >= right:
            return 0.0, max_angular, False
        return 0.0, -max_angular, False
    if tighter < pivot_room and opener < pivot_room:
        return 0.0, angular, True
    return moving(0.55)


def command_on_path(
    x: float,
    y: float,
    yaw: float,
    target_x: float,
    target_y: float,
    front: float,
    max_linear: float,
    max_angular: float,
    stop_distance: float,
    left: float = 999.0,
    right: float = 999.0,
    lateral: float = 0.0,
    align_error: float = 0.30,
    align_lateral: float = 0.20,
    align: bool = True,
) -> Tuple[float, float, bool]:
    """Follow ``pursuit_command``, but face the path before the first step.

    The first project's cruise stops translation while the heading error is
    above 0.30 rad. Doing that only when the robot is already on the line, and
    both flanks have 0.80 m, keeps the departure on the trajectory. In a shelf
    gap that stop yaws the chassis into the nearer wall. Once the robot is off
    to the side, the moving rejoin stays, so a finished pass does not become
    an in-place spin. A front return inside 0.20 m is the same emergency stop
    as the first project.
    """
    linear, angular, stalling = pursuit_command(
        x, y, yaw, target_x, target_y, front,
        max_linear, max_angular, stop_distance,
        left=left, right=right,
    )
    if linear == 0.0:
        return linear, angular, stalling
    if front < 0.20:
        # The nose is in the obstacle. A small turn toward the open side
        # takes the margin back; freezing here left the robot against the edge.
        if max(left, right) >= min(left, right) + 0.15:
            turn = 0.40 if left >= right else -0.40
            return 0.0, turn, False
        return 0.0, 0.0, False
    heading = math.atan2(target_y - y, target_x - x)
    error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
    if (
        align
        and min(left, right) >= 0.80
        and abs(lateral) <= align_lateral
        and abs(error) > align_error
    ):
        turn = max(-0.6, min(0.6, 1.2 * error))
        return 0.0, turn, False
    return linear, angular, stalling


def update_closing_speed(
    previous_distance: float | None,
    current_distance: float,
    previous_speed: float,
    dt: float = 0.05,
    alpha: float = 0.3,
) -> Tuple[float, float]:
    """
    EMA-smoothed range rate dD/dt (m/s).

    Negative speed => obstacle closing in.
    Returns (new_speed, current_distance_for_next_step).
    """
    if previous_distance is None or dt <= 0.0:
        return previous_speed, current_distance
    raw_rate = (current_distance - previous_distance) / dt
    new_speed = (1.0 - alpha) * previous_speed + alpha * raw_rate
    return new_speed, current_distance


def is_oncoming(
    front_dist: float,
    closing_speed: float,
    dist_thresh: float = 1.80,
    speed_thresh: float = -0.08,
) -> bool:
    """True if a front obstacle is close and approaching."""
    return front_dist < dist_thresh and closing_speed < speed_thresh


def is_rear_threat(
    rear_dist: float,
    rear_closing_speed: float,
    dist_thresh: float = 0.75,
    speed_thresh: float = -0.08,
) -> bool:
    return rear_dist < dist_thresh and rear_closing_speed < speed_thresh


def is_flank_threat(
    left_dist: float,
    left_closing_speed: float,
    right_dist: float,
    right_closing_speed: float,
    dist_thresh: float = 0.70,
    speed_thresh: float = -0.08,
) -> bool:
    left_hit = left_dist < dist_thresh and left_closing_speed < speed_thresh
    right_hit = right_dist < dist_thresh and right_closing_speed < speed_thresh
    return left_hit or right_hit


def is_remerge_blocked(
    evading: bool,
    front_dist: float,
    left_dist: float,
    path_progress_dist: float,
    front_thresh: float = 0.85,
    left_thresh: float = 0.65,
    min_path_dist: float = 0.40,
) -> bool:
    """Yield-on-re-merge: stop if corridor blocked after / outside evasion."""
    if evading or path_progress_dist <= min_path_dist:
        return False
    return front_dist < front_thresh or left_dist < left_thresh


def select_evasion_side(
    path_dir_x: float,
    path_dir_y: float,
    to_goal_x: float,
    to_goal_y: float,
    left_clearance: float,
    right_clearance: float,
    prefer_clearance: float = 0.45,
    fallback_clearance: float = 0.40,
) -> Tuple[float, str]:
    """
    Goal-aware left/right lane selection.

    cross_product = u × d (2D). Positive => goal is to the LEFT of path direction.
    Returns (side, label) where side = +1.0 (right) or -1.0 (left).
    """
    cross_prod = path_dir_x * to_goal_y - path_dir_y * to_goal_x
    prefer_left = cross_prod > 0.0 and left_clearance > prefer_clearance
    prefer_right = cross_prod <= 0.0 and right_clearance > prefer_clearance

    if prefer_left:
        return -1.0, "LEFT (towards goal)"
    if prefer_right:
        return 1.0, "RIGHT (towards goal)"
    if right_clearance > fallback_clearance:
        return 1.0, "RIGHT (fallback)"
    return -1.0, "LEFT (fallback)"


def lateral_evasion_target(
    robot_x: float,
    robot_y: float,
    path_ux: float,
    path_uy: float,
    evasion_side: float,
    left_clearance: float,
    right_clearance: float,
    forward_offset: float = 0.80,
    max_lateral: float = 0.35,
    min_lateral: float = 0.15,
    wall_margin: float = 0.40,
) -> Tuple[float, float]:
    """
    Virtual waypoint shifted laterally for closed-loop evasion.

    evasion_side > 0 => shift along right normal (uy, -ux).
    """
    # Right normal to path direction
    n_rx = path_uy
    n_ry = -path_ux

    if evasion_side > 0:
        safe_offset = min(max_lateral, max(min_lateral, right_clearance - wall_margin))
        evade_x = robot_x + forward_offset * path_ux + safe_offset * n_rx
        evade_y = robot_y + forward_offset * path_uy + safe_offset * n_ry
    else:
        safe_offset = min(max_lateral, max(min_lateral, left_clearance - wall_margin))
        evade_x = robot_x + forward_offset * path_ux - safe_offset * n_rx
        evade_y = robot_y + forward_offset * path_uy - safe_offset * n_ry
    return evade_x, evade_y


def should_end_evasion(
    time_in_evasion: float,
    timer_expired: bool,
    flank_clear: bool,
    front_dist: float,
    left_dist: float,
    right_dist: float,
    min_evasion_time: float = 0.8,
    wall_front: float = 0.50,
    wall_side: float = 0.30,
) -> Tuple[bool, str]:
    """Dual safety lock: hysteresis + flank clearance, or wall hazard abort."""
    is_wall_danger = (
        front_dist < wall_front
        or right_dist < wall_side
        or left_dist < wall_side
    )
    if is_wall_danger:
        return True, "Wall hazard"
    if time_in_evasion >= min_evasion_time and timer_expired and flank_clear:
        return True, "Flank clear"
    return False, ""


@dataclass
class AvoidanceState:
    """Closing-speed evasion, same timing as the first project's pedestrians."""

    active: bool = False
    side: float = 1.0
    started: float = 0.0
    until: float = 0.0
    closing: float = 0.0
    last_front: float | None = None
    approach_frames: int = 0
    track_x: float | None = None
    track_y: float | None = None
    other_speed: float = 0.0
    moving_frames: int = 0
    seen_off: float = 0.0


def front_is_on_the_map(front: float, map_range: float, slack: float = 0.45) -> bool:
    """True when the forward return is the wall or shelf the map already has.

    Another robot is not on the map, so its return is shorter than this ray
    by more than ``slack``. A matching return must not start a pass: that is
    the dodge that walks into a wall.
    """
    return front + slack >= map_range


def choose_pass_side(
    path_ux: float,
    path_uy: float,
    to_x: float,
    to_y: float,
    left: float,
    right: float,
    room: float = 1.20,
    bearing: float = 0.0,
) -> float | None:
    """Side to pass another robot, or None when that side is not open.

    ``bearing`` is the forward return relative to the path, positive when the
    other robot is left of that line. Pass on the opposite side of the path.
    The nose is the wrong reference: a nose yawed left of a path that already
    runs to the right makes that robot look like it is on the right, and the
    pass then leaves the open right side. A return on the path keeps the
    head-on rule, own right when the waypoint is straight ahead. A side closer
    than ``room`` is a wall. The other side is used only for that head-on case.
    Neither open means no pass.
    """
    if bearing > 0.10:
        side = 1.0
    elif bearing < -0.10:
        side = -1.0
    else:
        side, _label = select_evasion_side(path_ux, path_uy, to_x, to_y, left, right)
    clearance = right if side > 0.0 else left
    # Head-on still wants a wide lane. A robot already off to one side must be
    # passed as soon as the chassis fits: requiring 1.20 m on that side, then
    # refusing to move, is what drove into a robot approached from the right.
    needed = room if abs(bearing) <= 0.10 else 0.55
    if clearance >= needed:
        return side
    # The off-centre robot stays on its side. Falling back toward it is the collision.
    if abs(bearing) > 0.10:
        return None
    other = -side
    other_clearance = right if other > 0.0 else left
    if other_clearance >= room:
        return other
    return None


def side_arrival(off_path: float, other_speed: float, aim_cross: float) -> str | None:
    """How the robot that is arriving on the other's right should cross.

    ``off_path`` > 0 means the other robot is left of our path, so we are on
    its right. ``aim_cross`` > 0 means our aim is left of that robot. A right
    offset then drives across its bow. Stop and let it pass when our
    trajectory stays on its right. Shift to our left and pass behind it when
    our trajectory goes to its left. A stationary robot and a head-on robot
    are not this case.
    """
    # 0.12 rad is the head-on band. Just outside it, a moving robot on the
    # left is already the cut: a right offset follows them across the bow.
    if other_speed < 0.18 or off_path <= 0.12:
        return None
    if aim_cross > 0.08:
        return "behind"
    return "yield"


def _evasion_command(
    x: float,
    y: float,
    yaw: float,
    path_ux: float,
    path_uy: float,
    side: float,
    left: float,
    right: float,
    path_x: float,
    path_y: float,
    front: float,
) -> Tuple[float, float]:
    """Project-1 speeds, on a line parallel to the path.

    The first project's carrot is attached to the robot, so a pass that lasts
    longer than a moving pedestrian keeps sliding sideways into a wall. The
    offset here is taken from the path and clamped to the free flank.
    """
    heading = math.atan2(path_uy, path_ux)
    clearance = right if side > 0.0 else left
    right_x = math.sin(heading)
    right_y = -math.cos(heading)
    right_offset = (x - path_x) * right_x + (y - path_y) * right_y
    on_side = right_offset if side > 0.0 else -right_offset
    # Two chassis meet if the centres are still under about 0.50 m when they
    # draw level. 0.55 m on a carrot 0.80 m ahead was reached only after that.
    lateral = 0.75
    distance = 0.40 if on_side < 0.55 else 0.90
    evade_x, evade_y = passing_target(
        x, y, heading, side,
        distance=distance,
        lateral=lateral,
        clearance=clearance,
        margin=0.28,
        path_x=path_x,
        path_y=path_y,
    )
    target_yaw = math.atan2(evade_y - y, evade_x - x)
    error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
    linear = 0.35 if front >= 0.30 else 0.18
    if on_side < 0.55:
        linear = min(linear, 0.20)
    return linear, max(-1.2, min(1.2, 1.8 * error))


def step_avoidance(
    state: AvoidanceState,
    now: float,
    x: float,
    y: float,
    yaw: float,
    path_ux: float,
    path_uy: float,
    to_x: float,
    to_y: float,
    front: float,
    left: float,
    right: float,
    heading_error: float,
    dt: float = 0.1,
    map_range: float = 999.0,
    map_left: float = 999.0,
    map_right: float = 999.0,
    path_x: float | None = None,
    path_y: float | None = None,
    bearing: float = 0.0,
) -> Tuple[Tuple[float, float] | None, AvoidanceState]:
    """Pass another robot, or ``None`` to follow the path.

    The trigger is the first project's: the forward return is closing inside
    1.80 m. A wall or shelf already on the map matches ``map_range`` and does
    not count, including while the nose is still turning onto the line. The
    side and the speeds are the first project's, but the offset stays on a
    line parallel to the path so the pass cannot walk into the flank wall.
    It ends once the other robot has left the forward sector and the trailing
    flank, the chosen flank drops under 0.50 m, or both flanks are a corridor.
    A moving robot met on its right is not passed on our right: that cut
    crosses its bow. We stop when our trajectory stays to its right, and pass
    behind it on our left when our trajectory goes to its left.
    """
    if path_x is None:
        path_x = x
    if path_y is None:
        path_y = y
    previous = state.last_front
    closing, last_front = update_closing_speed(
        state.last_front, front, state.closing, dt=dt,
    )
    on_map = front_is_on_the_map(front, map_range)
    approaching = (
        previous is not None
        and front < previous - 0.004
        and -1.2 < closing < -0.08
        and front < 1.80
        and not on_map
    )
    frames = state.approach_frames + 1 if approaching else 0
    if abs(heading_error) > 0.30:
        frames = 0

    # The return is a range, not a centre. Push it forward by about a chassis
    # radius so a stationary body stays put while our own pose changes.
    # Closing speed cannot tell that body from one that is crossing.
    track_x = state.track_x
    track_y = state.track_y
    other_speed = state.other_speed
    moving_frames = state.moving_frames
    if on_map or front > 2.4:
        other_speed = 0.0
        moving_frames = 0
        track_x = None
        track_y = None
    else:
        ray = yaw + bearing
        cx = x + (front + 0.18) * math.cos(ray)
        cy = y + (front + 0.18) * math.sin(ray)
        if track_x is not None and dt > 1e-6:
            hop = math.hypot(cx - track_x, cy - track_y) / dt
            if hop < 1.5:
                other_speed = 0.6 * other_speed + 0.4 * hop
                moving_frames = moving_frames + 1 if other_speed >= 0.18 else 0
        track_x, track_y = cx, cy
    tracked_speed = other_speed if moving_frames >= 3 else 0.0
    off_path = bearing_off_path(yaw, bearing, math.atan2(path_uy, path_ux))
    sight = yaw + bearing
    aim_cross = math.cos(sight) * to_y - math.sin(sight) * to_x
    decision = side_arrival(off_path, tracked_speed, aim_cross)
    if decision == "behind" and left < 0.55:
        decision = "yield"
    gap = left < 1.0 and right < 1.0
    # Remember which side the other robot was on when this manoeuvre started.
    # A head-on pass also slides them toward our left later; that one must
    # keep its right offset. A pass that started with them already on our
    # left is the cut.
    latched_off = state.seen_off if state.active else off_path
    # They began on our left and have now crossed onto our right. The latched
    # right offset follows them, and the return to the line drives back into
    # them. Stop until they have gone past.
    followed = (
        state.active
        and state.side > 0.0
        and state.seen_off > 0.12
        and tracked_speed >= 0.18
        and not on_map
        and front < 1.70
        and off_path < -0.20
    )
    if followed:
        decision = "yield"

    def parked(active: bool, side: float, started: float, until: float, count: int) -> AvoidanceState:
        return AvoidanceState(
            active=active,
            side=side,
            started=started,
            until=until,
            closing=closing,
            last_front=last_front,
            approach_frames=count,
            track_x=track_x,
            track_y=track_y,
            other_speed=other_speed,
            moving_frames=moving_frames,
            seen_off=latched_off if active else 0.0,
        )

    def crossing_now() -> bool:
        return decision is not None and not on_map and front < 1.80 and abs(heading_error) <= 0.80

    # A yield has no steering side. The wall abort belongs to a moving pass.
    # Stay stopped while the other robot is still moving across the front,
    # even after it leaves the left side. Releasing on that sign change is
    # what started the right offset, the follow, and the turn back into it.
    if state.active and state.side == 0.0:
        held = now - state.started
        if (
            decision == "behind"
            and crossing_now()
            and not gap
        ):
            return _evasion_command(
                x, y, yaw, path_ux, path_uy, -1.0, left, right, path_x, path_y, front,
            ), parked(True, -1.0, now, now, frames)
        # Range opening at 1.20 m is not "past": that is the moment the other
        # robot is crossing the line. Wait until the return is far, is the
        # map again, or the other robot has stopped.
        released = (
            held >= 2.5
            or on_map
            or front >= 1.70
            or other_speed < 0.12
        )
        if released:
            return None, parked(False, 0.0, state.started, state.until, 0)
        return (0.0, 0.0), parked(True, 0.0, state.started, state.until, frames)

    # Only the robot that is on the other's right changes plan. A pass already
    # going left is the behind manoeuvre and must not become a stop. A right
    # pass started while they were already off to our left is the cut across
    # the bow, so it is replaced once they are confirmed to be moving. A
    # head-on right pass (they started on the axis) is left alone.
    take_crossing = crossing_now() and not (state.active and state.side < 0.0)
    if (
        take_crossing
        and state.active
        and state.side > 0.0
        and state.seen_off <= 0.12
        and not followed
    ):
        take_crossing = False
    if take_crossing and decision == "behind" and gap:
        take_crossing = False
    if take_crossing:
        if decision == "yield":
            return (0.0, 0.0), parked(True, 0.0, now, now, frames)
        return _evasion_command(
            x, y, yaw, path_ux, path_uy, -1.0, left, right, path_x, path_y, front,
        ), parked(True, -1.0, now, now, frames)

    if state.active:
        clearance = right if state.side > 0.0 else left
        # Steering-side clearance is the wall. The other robot swings through
        # the trailing flank; a shelf there matches the map and must not hold
        # the pass open, or the robot drives parallel into the next wall.
        if state.side > 0.0:
            trailing, trailing_map = left, map_left
        else:
            trailing, trailing_map = right, map_right
        still_there = trailing < 1.20 and not front_is_on_the_map(trailing, trailing_map)
        # Under 0.50 m the chassis no longer fits on that side. Both flanks
        # under 1 m is the shelf-wall gap when those returns are the map.
        # The robot being passed also reads under 1 m: cancelling then drives
        # straight back into it. 0.90 m used to cancel a real pass beside a
        # wall and the robot then drove into the one it was approaching.
        wall = clearance < 0.50 or (gap and not still_there)
        passed = (
            on_map
            and not still_there
            and (now - state.started) >= 0.8
            and front >= 0.80
        )
        if wall or passed:
            return None, parked(False, state.side, state.started, state.until, 0)
        return _evasion_command(
            x, y, yaw, path_ux, path_uy, state.side, left, right, path_x, path_y, front,
        ), parked(True, state.side, state.started, state.until, frames)

    if (
        frames >= 3
        and abs(heading_error) <= 0.30
        and not on_map
        and is_oncoming(front, closing)
    ):
        # The scan bearing is relative to the nose. The pass has to use the
        # path: the right-hand trajectory stays on the right when that flank
        # is open, even if the nose is still left of the other robot.
        side = choose_pass_side(
            path_ux, path_uy, to_x, to_y, left, right, bearing=off_path,
        )
        if side is not None:
            return _evasion_command(
                x, y, yaw, path_ux, path_uy, side, left, right, path_x, path_y, front,
            ), parked(True, side, now, now + 2.2, frames)
    return None, parked(False, state.side, state.started, state.until, frames)


def inflate_occupancy(
    occupied_mask: "object",
    inflation_cells: int,
) -> "object":
    """
    Binary inflate True cells by chebyshev radius inflation_cells.

    occupied_mask: 2D numpy bool/uint8 array.
    Returns uint8 grid (1 = blocked, 0 = free).
    """
    import numpy as np

    h, w = occupied_mask.shape
    grid = np.zeros((h, w), dtype=np.uint8)
    rows, cols = np.where(occupied_mask)
    r = max(0, int(inflation_cells))
    for row, col in zip(rows, cols):
        r0 = max(0, row - r)
        r1 = min(h, row + r + 1)
        c0 = max(0, col - r)
        c1 = min(w, col + r + 1)
        grid[r0:r1, c0:c1] = 1
    return grid


def nearest_free_cell(
    grid,
    row: int,
    col: int,
    max_radius: int,
) -> Tuple[int, int] | None:
    """Nearest free cell, or None when the blocked region is deeper than max_radius.

    A robot stopped against a corner sits inside the inflated obstacle. Clearing
    only a couple of cells around it does not reach the corridor.
    """
    height, width = grid.shape
    row = max(0, min(height - 1, row))
    col = max(0, min(width - 1, col))
    if grid[row, col] == 0:
        return row, col
    best: Tuple[int, int] | None = None
    best_distance = math.inf
    for dr in range(-max_radius, max_radius + 1):
        for dc in range(-max_radius, max_radius + 1):
            distance = math.hypot(dr, dc)
            if distance == 0 or distance > max_radius or distance >= best_distance:
                continue
            next_row = row + dr
            next_col = col + dc
            if not (0 <= next_row < height and 0 <= next_col < width):
                continue
            if grid[next_row, next_col] != 0:
                continue
            best = (next_row, next_col)
            best_distance = distance
    return best


def astar_grid(
    grid,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    clear_radius: int = 3,
    penalty=None,
) -> List[Tuple[int, int]]:
    """
    8-connected A* on a binary occupancy grid (1 = blocked).

    ``penalty`` is an optional float array of the same shape. Free cells near
    an obstacle cost more, so the path sits in the middle of a gap instead of
    shaving the inflated corner. A diagonal step is refused when it would cut
    that corner.

    Returns list of (row, col) from start+1 step to goal, or empty if no path.
    """
    import heapq

    import numpy as np

    height, width = grid.shape
    work = grid.copy()
    s_row, s_col = start
    g_row, g_col = goal
    s_row = max(0, min(height - 1, s_row))
    s_col = max(0, min(width - 1, s_col))
    g_row = max(0, min(height - 1, g_row))
    g_col = max(0, min(width - 1, g_col))

    for dr in range(-clear_radius, clear_radius + 1):
        for dc in range(-clear_radius, clear_radius + 1):
            for r, c in ((s_row + dr, s_col + dc), (g_row + dr, g_col + dc)):
                if 0 <= r < height and 0 <= c < width:
                    work[r, c] = 0

    open_set: list = []
    heapq.heappush(open_set, (0.0, (s_row, s_col)))
    came_from = {}
    g_score = {(s_row, s_col): 0.0}

    def heuristic(r: int, c: int) -> float:
        return math.hypot(r - g_row, c - g_col)

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    found = False
    iterations = 0
    max_iterations = width * height

    while open_set and iterations < max_iterations:
        iterations += 1
        _, current = heapq.heappop(open_set)
        if current == (g_row, g_col):
            found = True
            break
        r, c = current
        for dr, dc in neighbors:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if work[nr, nc] == 1:
                continue
            if dr != 0 and dc != 0 and (work[r + dr, c] == 1 or work[r, c + dc] == 1):
                continue
            step = math.hypot(dr, dc)
            if penalty is not None:
                step *= 1.0 + float(penalty[nr, nc])
            tentative = g_score[current] + step
            if (nr, nc) not in g_score or tentative < g_score[(nr, nc)]:
                g_score[(nr, nc)] = tentative
                heapq.heappush(open_set, (tentative + heuristic(nr, nc), (nr, nc)))
                came_from[(nr, nc)] = current

    if not found:
        return []

    path_cells: List[Tuple[int, int]] = []
    curr = (g_row, g_col)
    while curr in came_from:
        path_cells.append(curr)
        curr = came_from[curr]
    path_cells.reverse()
    return path_cells
