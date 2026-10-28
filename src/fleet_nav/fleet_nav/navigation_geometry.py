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


def sector_body_returns(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    map_range_at,
    range_min: float = 0.0,
    slack: float = 0.45,
    empty_value: float = 999.0,
) -> SectorDistances:
    """Closest return in each sector that the saved map does not already have.

    The sector minimum can be a shelf corner. A walker on the next ray is
    then invisible, and the offset only starts once they are closer than
    that corner. Wall clearance still uses the raw minimum.
    """
    best_range = {
        "front": empty_value,
        "left": empty_value,
        "right": empty_value,
        "rear": empty_value,
    }
    best_angle = {"front": 0.0, "left": 1.27, "right": -1.27, "rear": math.pi}
    for index, reading in enumerate(ranges):
        if reading is None or math.isinf(reading) or math.isnan(reading) or reading < range_min:
            continue
        angle = angle_min + index * angle_increment
        if reading + slack >= map_range_at(angle):
            continue
        if -FRONT_HALF_ANGLE <= angle <= FRONT_HALF_ANGLE:
            key = "front"
        elif FRONT_HALF_ANGLE < angle <= LEFT_MAX_ANGLE:
            key = "left"
        elif RIGHT_MIN_ANGLE <= angle < -FRONT_HALF_ANGLE:
            key = "right"
        elif abs(angle) > REAR_ABS_ANGLE:
            key = "rear"
        else:
            continue
        if reading < best_range[key]:
            best_range[key] = reading
            best_angle[key] = angle
    return SectorDistances(
        front=best_range["front"],
        left=best_range["left"],
        right=best_range["right"],
        rear=best_range["rear"],
        front_angle=best_angle["front"],
        left_angle=best_angle["left"],
        right_angle=best_angle["right"],
    )


# The lidar plane is above the chassis, so another robot's scan only meets
# the lidar housing. The housing sits inside this footprint: chassis box
# from x=-0.10 to 0.20 in the base frame, wheels out to y=±0.20. Painting
# that rectangle into the scan is what the pass and the stop both measure.
FLEET_LIDAR_FORWARD = 0.05
_FLEET_X0 = -0.10
_FLEET_X1 = 0.20
_FLEET_Y0 = -0.20
_FLEET_Y1 = 0.20


def ranges_with_fleet(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    lidar_x: float,
    lidar_y: float,
    yaw: float,
    mates: Sequence[Tuple[float, float, float]],
    range_min: float = 0.15,
    range_max: float = 3.0,
) -> Sequence[float]:
    """Shorten ``ranges`` where another robot's footprint stands.

    ``mates`` are ``(x, y, yaw)`` of the other bases. The ray starts at the
    lidar, ``FLEET_LIDAR_FORWARD`` metres ahead of our own base. A miss stays
    the original return, so a wall that is closer than the robot is kept.
    """
    if not mates:
        return ranges
    painted = list(ranges)
    for index, reading in enumerate(painted):
        angle = angle_min + index * angle_increment
        dx = math.cos(yaw + angle)
        dy = math.sin(yaw + angle)
        best = reading if reading is not None and not math.isinf(reading) and not math.isnan(reading) else None
        for mate_x, mate_y, mate_yaw in mates:
            c = math.cos(mate_yaw)
            s = math.sin(mate_yaw)
            bx = c * (lidar_x - mate_x) + s * (lidar_y - mate_y)
            by = -s * (lidar_x - mate_x) + c * (lidar_y - mate_y)
            rdx = c * dx + s * dy
            rdy = -s * dx + c * dy
            for edge, origin, direction, lo, hi, other_o, other_d in (
                (_FLEET_X1, bx, rdx, _FLEET_Y0, _FLEET_Y1, by, rdy),
                (_FLEET_X0, bx, rdx, _FLEET_Y0, _FLEET_Y1, by, rdy),
                (_FLEET_Y1, by, rdy, _FLEET_X0, _FLEET_X1, bx, rdx),
                (_FLEET_Y0, by, rdy, _FLEET_X0, _FLEET_X1, bx, rdx),
            ):
                if abs(direction) < 1e-9:
                    continue
                travel = (edge - origin) / direction
                if travel <= range_min or travel > range_max:
                    continue
                other = other_o + travel * other_d
                if lo - 1e-6 <= other <= hi + 1e-6 and (best is None or travel < best):
                    best = travel
        if best is not None:
            painted[index] = best
    return painted


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
# Walker cylinder. A flat face of the square reaches 0.15 m, a corner 0.21 m.
PERSON_RADIUS = 0.25
CHASSIS_HALF = 0.15
CORNER_REACH = CHASSIS_HALF * math.sqrt(2.0)
YIELD_DASH = 0.42
YIELD_GAP = 0.10


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
    an in-place spin. A demi-tour is the exception: past about 1.8 rad the
    robot spins in place even after cruise has started, as long as both flanks
    clear the chassis corner. A front return inside 0.20 m is the same
    emergency stop as the first project.
    """
    linear, angular, stalling = pursuit_command(
        x, y, yaw, target_x, target_y, front,
        max_linear, max_angular, stop_distance,
        left=left, right=right,
    )
    if front < 0.20:
        # The nose is in the obstacle. A small turn toward the open side
        # takes the margin back; freezing here left the robot against the edge.
        if max(left, right) >= min(left, right) + 0.15:
            turn = 0.40 if left >= right else -0.40
            return 0.0, turn, False
        return 0.0, 0.0, False
    heading = math.atan2(target_y - y, target_x - x)
    error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
    # A demi-tour while rolling walks the chassis off the line, and the rest
    # of the leg is then followed in parallel. Spin until the nose faces the
    # path. A mild corner stays a moving turn. Below 0.35 m the corner of the
    # square would meet the wall, so that gap still creeps.
    if min(left, right) >= 0.35 and abs(error) > 1.80:
        turn = max(-0.6, min(0.6, 1.2 * error))
        return 0.0, turn, False
    if linear == 0.0:
        return linear, angular, stalling
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
    track_vx: float = 0.0
    track_vy: float = 0.0
    # -1 or +1 while stopped for a robot crossing that way. 0 otherwise.
    cross_hold: float = 0.0
    # True once the other body has been tracked at robot cruise, above a
    # walker's 0.20 m/s. Cleared when that track is dropped.
    seen_fast: bool = False
    # True once this robot was clearly first at that crossing. A later dip
    # in the other's speed must not put it back into the yield.
    cross_go: bool = False
    # Latched once this side pass has seen the other body walk the path.
    # A later sample that slides around the cylinder is not a new crossing.
    path_hold: bool = False
    hold_vx: float = 0.0
    hold_vy: float = 0.0
    # How long a body has stayed beside the chassis without starting to move.
    # A parked robot is released to the ordinary pass. A robot that then
    # starts is owned again on the next moving sample.
    still_for: float = 0.0
    # Sweeps that missed a robot we were already yielding to or following.
    # One empty sector in Gazebo is not "they left".
    missed: int = 0
    # Consecutive frames whose tracked velocity is across the path. A crossing
    # robot's closest lidar point can sit under 0.24 m/s for the whole
    # approach; three of those frames is still that robot.
    cross_frames: int = 0
    # Lateral offset of the other body, and how many frames ago that sample
    # was taken. A parked chassis wobbles about that offset. A robot that is
    # crossing is closer to the line after the window.
    left_anchor: float = 0.0
    anchor_age: int = 0
    side_closing: bool = False
    # Successive four-frame windows that moved at least 6 cm toward the
    # path. One window is range wobble. Two, or a single 10 cm closure,
    # is a robot actually crossing.
    close_hits: int = 0
    # Recent centre steps, in m/s. Kept so a one-frame cut still has a
    # velocity sample. The speed itself comes from ``pos_samples``.
    vel_samples: tuple = ()
    # Recent centre positions, (time, x, y). Range noise is a fast step
    # one way and the opposite step next, so a median of those speeds stays
    # loud and can point the wrong way. The median of the positions is the
    # body, and the speed is how that median moved.
    pos_samples: tuple = ()
    # Once a crossing has actually been seen, a later noisy frame must not
    # throw the yield away. A parked chassis never sets this.
    proved_cross: bool = False
    # The longer window stayed at a walker's pace. A later spike, from the
    # hit sliding around the cylinder as the nose yaws, is not a robot.
    walker_known: bool = False


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


def _moving_along_path(path_ux: float, path_uy: float, vx: float, vy: float) -> bool:
    """True when the other body is travelling along the path, not across it.

    A walker coming toward the robot, or walking the same way, is this case.
    A robot cutting from one side to the other is not: its lateral speed is
    the whole motion.
    """
    if math.hypot(vx, vy) < 0.12:
        return False
    lateral = abs(path_ux * vy - path_uy * vx)
    return lateral < 0.08


def _mostly_across(path_ux: float, path_uy: float, vx: float, vy: float) -> bool:
    """True when the other body is cutting across the path, not travelling along it.

    A walker coming along the path, even a little off the centreline, is not
    this case. The 0.70 ratio keeps a shallow head-on angle on the side offset.
    """
    speed = math.hypot(vx, vy)
    if speed < 0.12:
        return False
    cross = abs(path_ux * vy - path_uy * vx)
    return cross >= 0.08 and cross >= 0.70 * speed


def _follows_across(path_ux: float, path_uy: float, vx: float, vy: float, side: float) -> bool:
    """True when ``side`` runs with a robot that is crossing the path.

    Positive cross-track velocity is toward the left of the path. A left
    offset while they move left, or a right offset while they move right,
    yaws into their travel and holds that parallel instead of letting them
    pass. Motion along the path is not this case.
    """
    if side == 0.0 or not _mostly_across(path_ux, path_uy, vx, vy):
        return False
    cross = path_ux * vy - path_uy * vx
    return (side < 0.0 and cross > 0.0) or (side > 0.0 and cross < 0.0)


def _crossing_order(
    along: float | None,
    ahead: float | None,
    their_speed: float,
    other_on_right: bool,
) -> str:
    """Who reaches the crossing first if this robot keeps the 0.30 m/s cruise.

    ``first`` stays on the path. ``second`` waits. ``unclear`` does not start
    a wait. The cruise is the speed the navigator would command on the path,
    not the speed being sent now: a robot that has already stopped would
    otherwise always look later, and both would wait. A near tie uses the
    shorter distance, then the side, so the two robots agree on one of them.
    """
    if along is None or ahead is None or their_speed < 0.18:
        return "unclear"
    if ahead <= -0.40 or along < -0.15:
        return "first"
    our_time = max(along, 0.0) / 0.30
    their_time = max(ahead, 0.0) / their_speed
    if our_time + 0.15 < their_time:
        return "first"
    if their_time + 0.15 < our_time:
        return "second"
    if along + 0.08 < ahead:
        return "first"
    if ahead + 0.08 < along:
        return "second"
    if other_on_right:
        return "first"
    return "second"


def _clearly_first(along: float | None, ahead: float | None, their_speed: float) -> bool:
    """True when this robot is first by enough that a speed blip cannot explain it."""
    if along is None or ahead is None or their_speed < 0.18:
        return False
    if ahead <= -0.40 or along < -0.15:
        return False
    our_time = max(along, 0.0) / 0.30
    their_time = max(ahead, 0.0) / their_speed
    return our_time + 0.60 < their_time


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


def _straight_along_path(yaw: float, path_ux: float, path_uy: float) -> tuple[float, float]:
    """Cruise on the path heading.

    The corner pivot in ``command_on_path`` reads a robot beside the chassis
    as a wall and yaws off the line. This command is the whole answer for the
    robot that has priority, and for the one that has finished waiting.
    """
    error = _wrap(math.atan2(path_uy, path_ux) - yaw)
    turn = max(-0.70, min(0.70, 1.4 * error))
    # Already yawed off the leg, driving would leave the crossing beside the
    # line. Square up first. A small error keeps the cruise.
    if abs(error) > 0.45:
        return 0.0, turn
    return 0.30, turn


def _reverse_along_path(yaw: float, path_ux: float, path_uy: float) -> tuple[float, float]:
    """Back up on the path heading, without leaving the line.

    A yield that starts already inside the crossing is a stopped chassis on
    the other robot's path. Cruise speed in reverse clears that path before
    the robot with priority arrives. Slower than that, and the two squares meet.
    """
    error = _wrap(math.atan2(path_uy, path_ux) - yaw)
    linear = -0.30 if abs(error) < 0.45 else 0.0
    return linear, max(-0.70, min(0.70, 1.4 * error))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _select_body_ray(
    x: float,
    y: float,
    yaw: float,
    front: float,
    bearing: float,
    on_map: bool,
    left: float,
    left_bearing: float | None,
    map_left: float,
    right: float,
    right_bearing: float | None,
    map_right: float,
    track_x: float | None,
    track_y: float | None,
) -> tuple[float, float] | None:
    """Range and nose angle of the moving body, or None when the return is the map.

    The forward cone is 35°. A walker crossing the path is often still in the
    left or right sector when the chassis is already on their line. The flank
    is used only when the caller measured it and it is shorter than the wall.
    """
    options: list[tuple[float, float]] = []
    if not on_map and front <= 2.4:
        options.append((front, bearing))
    if (
        left_bearing is not None
        and left <= 2.4
        and not front_is_on_the_map(left, map_left)
    ):
        options.append((left, left_bearing))
    if (
        right_bearing is not None
        and right <= 2.4
        and not front_is_on_the_map(right, map_right)
    ):
        options.append((right, right_bearing))
    if not options:
        return None
    if track_x is not None:
        best: tuple[float, float] | None = None
        best_d = 0.60
        for rng, ang in options:
            cx = x + (rng + 0.18) * math.cos(yaw + ang)
            cy = y + (rng + 0.18) * math.sin(yaw + ang)
            dist = math.hypot(cx - track_x, cy - track_y)
            if dist <= best_d:
                best = (rng, ang)
                best_d = dist
        if best is not None:
            return best
    return min(options, key=lambda item: item[0])


def _chassis_extent(yaw: float, nx: float, ny: float) -> float:
    """How far the square sticks out toward a unit world direction."""
    c, s = math.cos(yaw), math.sin(yaw)
    bx = c * nx + s * ny
    by = -s * nx + c * ny
    return CHASSIS_HALF * (abs(bx) + abs(by))


def _walker_line(
    x: float,
    y: float,
    path_ux: float,
    path_uy: float,
    px: float | None,
    py: float | None,
    vx: float,
    vy: float,
) -> tuple[float, float, float, float | None, float | None, float] | None:
    """Walker direction and where the two straight paths cross.

    ``signed`` is the robot's distance to the walker's line, positive on the
    left of their velocity. ``along`` is metres for the robot to the crossing,
    ``ahead`` metres for the walker. Both are None when the paths are parallel.
    """
    speed = math.hypot(vx, vy)
    if px is None or py is None or speed < 0.05:
        return None
    hx, hy = vx / speed, vy / speed
    signed = (x - px) * hy - (y - py) * hx
    denom = path_ux * hy - path_uy * hx
    dx, dy = px - x, py - y
    along = ahead = None
    if abs(denom) >= 0.20:
        along = (dx * hy - dy * hx) / denom
        ahead = (dx * path_uy - dy * path_ux) / denom
    return hx, hy, signed, along, ahead, speed


def _line_reaches_chassis(
    x: float,
    y: float,
    path_ux: float,
    path_uy: float,
    px: float | None,
    py: float | None,
    vx: float,
    vy: float,
) -> bool:
    """True while the walker has not yet gone past a line that clips the chassis."""
    info = _walker_line(x, y, path_ux, path_uy, px, py, vx, vy)
    if info is None:
        return False
    _hx, _hy, signed, _along, ahead, _speed = info
    if abs(signed) >= PERSON_RADIUS + CORNER_REACH + YIELD_GAP:
        return False
    if ahead is None:
        along_path = (px - x) * path_ux + (py - y) * path_uy
        return along_path > -0.35
    return ahead > -0.35


def _aim_stays_right_and_beyond(
    x: float,
    y: float,
    path_ux: float,
    path_uy: float,
    dest_x: float,
    dest_y: float,
    px: float | None,
    py: float | None,
    vx: float,
    vy: float,
) -> bool:
    """The goal is past the walker and the path stays on their right.

    ``dest`` is the goal, not the short path carrot. A walker beside that
    line, coming toward the robot, is passed by driving the path. A line that
    crosses the path and still clips the chassis stays the dash or the square
    stop. Drawing level is part of this pass, so being abreast is not a reason
    to freeze. Crossing their front and rejoining from the left is the long way.
    """
    if px is None or py is None or math.hypot(vx, vy) < 0.12:
        return False
    # A line that crosses the path and still clips the chassis is the dash
    # or the square stop. A parallel line meets the flat face, so the side
    # clearance below decides it.
    info = _walker_line(x, y, path_ux, path_uy, px, py, vx, vy)
    if (
        info is not None
        and info[3] is not None
        and _line_reaches_chassis(x, y, path_ux, path_uy, px, py, vx, vy)
    ):
        return False
    ped_along = (px - x) * path_ux + (py - y) * path_uy
    dest_along = dest_x * path_ux + dest_y * path_uy
    if dest_along < ped_along + 0.30:
        return False
    heading = math.atan2(path_uy, path_ux)
    left_x, left_y = -math.sin(heading), math.cos(heading)
    lat = (px - x) * left_x + (py - y) * left_y
    # Flat face 0.15 m plus the cylinder 0.25 m, and room for a small yaw.
    # Closer than this, staying on the path puts the side into the walker.
    if lat < PERSON_RADIUS + CHASSIS_HALF + 0.20:
        return False
    sight = math.atan2(py - y, px - x)
    if math.cos(sight) * dest_y - math.sin(sight) * dest_x > 0.08:
        return False
    cruise = 0.28
    span = max(dest_along, 0.0)
    limit = min(8.0, span / cruise + 0.2)
    t = 0.0
    while t < limit:
        t += 0.2
        moved = min(cruise * t, span)
        rx = x + moved * path_ux
        ry = y + moved * path_uy
        qx = px + vx * t
        qy = py + vy * t
        dx, dy = qx - rx, qy - ry
        c, s = math.cos(heading), math.sin(heading)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        nearest_x = min(CHASSIS_HALF, max(-CHASSIS_HALF, lx))
        nearest_y = min(CHASSIS_HALF, max(-CHASSIS_HALF, ly))
        gap = math.hypot(lx - nearest_x, ly - nearest_y) - PERSON_RADIUS
        if gap < 0.12:
            return False
    return limit > 0.0


def _dash_reaches(
    along: float | None,
    ahead: float | None,
    closing_speed: float,
) -> bool:
    """True when a sprint still clears the walker's body.

    Their cylinder meets the chassis about 0.40 m before either centre
    reaches the crossing, and the rear face is still on their line after
    our centre has passed it. Stopping on that line is the collision. The
    sprint has to start early enough and keep going until the rear is off.
    """
    if along is None or ahead is None:
        return False
    body = PERSON_RADIUS + CHASSIS_HALF
    end = -(body + YIELD_GAP)
    if along <= end or ahead <= body:
        return False
    remain = along - end
    their = ahead - body
    return remain / YIELD_DASH <= their / max(closing_speed, 0.18) + 0.05


def _yield_command(
    x: float,
    y: float,
    yaw: float,
    path_ux: float,
    path_uy: float,
    px: float | None,
    py: float | None,
    vx: float,
    vy: float,
    ped_speed: float,
) -> tuple[float, float]:
    """Cross in front of a walker, or stop square and clear of their line.

    A hard stop on the line leaves the 0.21 m corner in the 0.25 m cylinder
    whenever the chassis is near 45° to that line. If the walker is still far
    enough along it, accelerating along the path clears the crossing first.
    """
    info = _walker_line(x, y, path_ux, path_uy, px, py, vx, vy)
    if info is None:
        return (0.0, 0.0)
    hx, hy, signed, along, ahead, speed = info
    if speed < 0.12:
        return (0.0, 0.0)
    # A short-window spike is the hit sliding around the cylinder, not a
    # faster walker. The track itself is the pace the sprint was planned on.
    closing_speed = max(speed, 0.18)
    if 0.12 <= ped_speed <= 0.24:
        closing_speed = max(closing_speed, ped_speed)
    if _dash_reaches(along, ahead, closing_speed):
        error = _wrap(math.atan2(path_uy, path_ux) - yaw)
        linear = YIELD_DASH if abs(error) < 0.50 else 0.0
        return linear, max(-1.0, min(1.0, 1.6 * error))

    # Far from the line, the corner cannot reach the cylinder. Yawing here
    # would turn a robot-vs-robot stop into a spin.
    dist = abs(signed)
    if dist >= PERSON_RADIUS + CORNER_REACH + 0.35:
        return (0.0, 0.0)
    s = 1.0 if signed >= 0.0 else -1.0
    ax, ay = hy * s, -hx * s
    extent = _chassis_extent(yaw, -ax, -ay)
    flat_need = PERSON_RADIUS + CHASSIS_HALF + YIELD_GAP
    now_need = PERSON_RADIUS + extent + YIELD_GAP
    # A square's flat face is every quarter turn. Prefer the one already
    # along the path: turning 90° toward the walker is the cut across their
    # front that the straight right-side course was meant to avoid.
    alpha = math.atan2(hy, hx)
    path_h = math.atan2(path_uy, path_ux)
    target = min(
        (alpha + k * math.pi / 2.0 for k in range(4)),
        key=lambda heading: (
            0.0 if abs(_wrap(heading - path_h)) < 0.45 else 1.0,
            abs(_wrap(heading - yaw)),
        ),
    )
    face = _wrap(target - yaw)
    if dist >= max(flat_need, now_need):
        if abs(face) < 0.12:
            return (0.0, 0.0)
        return (0.0, max(-1.0, min(1.0, 1.4 * face)))
    away = math.atan2(ay, ax)
    err_fwd = _wrap(away - yaw)
    err_rev = _wrap(away + math.pi - yaw)
    if abs(err_rev) < abs(err_fwd):
        escape = err_rev
        linear = -0.18 if abs(escape) < 0.45 else 0.0
    else:
        escape = err_fwd
        linear = 0.18 if abs(escape) < 0.45 else 0.0
    turn = face if abs(_wrap(face - escape)) < 0.50 else escape
    return linear, max(-1.0, min(1.0, 1.6 * turn))


def plan_reached(
    x: float,
    y: float,
    goal: Tuple[float, float],
    path: Sequence[Tuple[float, float]],
    tolerance: float,
) -> bool:
    """True when the robot is on the goal, or on the last point the map allowed.

    A click inside a wall snaps to the last free cell. That cell can sit well
    outside ``tolerance`` of the click. Spinning there to face a point under
    the chassis walks the corner into a robot the detour had already cleared.
    """
    if math.hypot(goal[0] - x, goal[1] - y) < tolerance:
        return True
    if not path:
        return False
    end_x, end_y = path[-1]
    if math.hypot(end_x - x, end_y - y) >= tolerance:
        return False
    return math.hypot(goal[0] - end_x, goal[1] - end_y) >= tolerance


def peers_the_path_clears(
    path: Sequence[Tuple[float, float]],
    peers: Sequence[Tuple[float, float]],
    margin: float,
) -> Tuple[Tuple[float, float], ...]:
    """Parked bases the whole path stays at least ``margin`` metres from.

    One peer the path still clips clears the list: that robot is still the
    reactive pass. An empty path or an empty fleet is the ordinary plan.
    """
    if len(path) < 2 or margin <= 0.0 or not peers:
        return ()
    kept: List[Tuple[float, float]] = []
    for mate_x, mate_y in peers:
        nearest = min(
            math.hypot(point_x - mate_x, point_y - mate_y) for point_x, point_y in path
        )
        if nearest < margin:
            return ()
        kept.append((mate_x, mate_y))
    return tuple(kept)


def _segment_clears_disc(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    centre_x: float,
    centre_y: float,
    radius: float,
) -> bool:
    dx = x1 - x0
    dy = y1 - y0
    length2 = dx * dx + dy * dy
    if length2 < 1e-8:
        return math.hypot(x0 - centre_x, y0 - centre_y) >= radius
    blend = ((centre_x - x0) * dx + (centre_y - y0) * dy) / length2
    blend = max(0.0, min(1.0, blend))
    nearest_x = x0 + blend * dx
    nearest_y = y0 + blend * dy
    return math.hypot(nearest_x - centre_x, nearest_y - centre_y) >= radius


def aim_clear_of_peers(
    x: float,
    y: float,
    path: Sequence[Tuple[float, float]],
    index: int,
    peers: Sequence[Tuple[float, float]],
    margin: float,
) -> Tuple[float, float] | None:
    """Point on the detour whose straight aim does not enter a parked robot.

    The usual 0.80 m merge carrot is a chord. Around a robot that chord cuts
    the inside of the circle the planner just left clear. A robot already
    inside that circle is sent at the nearest path point, back out.
    """
    if not peers or len(path) < 2 or margin <= 0.0:
        return None
    index = max(0, min(index, len(path) - 1))
    inside = any(math.hypot(x - mate_x, y - mate_y) < margin for mate_x, mate_y in peers)
    if inside:
        window = path[max(0, index - 2):min(len(path), index + 6)]
        return min(window, key=lambda point: math.hypot(point[0] - x, point[1] - y))
    chosen = path[index]
    for lead in (0.20, 0.35, 0.50, 0.70):
        point = path_lookahead(path, index, x, y, lead)
        if point is None:
            continue
        if all(
            _segment_clears_disc(x, y, point[0], point[1], mate_x, mate_y, margin)
            for mate_x, mate_y in peers
        ):
            chosen = point
        else:
            break
    return chosen


def body_is_cleared_peer(
    x: float,
    y: float,
    yaw: float,
    track_x: float | None,
    track_y: float | None,
    dist: float,
    bearing: float,
    peers: Sequence[Tuple[float, float]],
    reach: float = 0.50,
) -> bool:
    """True when the tracked return is a robot the path already goes around.

    The centre estimate is the same 0.18 m step past the hit that the tracker
    publishes. A crossing robot is not in ``peers``.
    """
    if not peers:
        return False
    points: List[Tuple[float, float]] = []
    if track_x is not None and track_y is not None:
        points.append((track_x, track_y))
    if dist < 900.0:
        reach_m = dist + 0.18
        points.append((
            x + reach_m * math.cos(yaw + bearing),
            y + reach_m * math.sin(yaw + bearing),
        ))
    for point_x, point_y in points:
        for mate_x, mate_y in peers:
            if math.hypot(point_x - mate_x, point_y - mate_y) <= reach:
                return True
    return False


def ranges_behind_cleared_peers(
    x: float,
    y: float,
    yaw: float,
    front: float,
    left: float,
    right: float,
    front_bearing: float,
    left_bearing: float,
    right_bearing: float,
    peers: Sequence[Tuple[float, float]],
    floor: float = 0.22,
    opened: float = 0.95,
) -> Tuple[float, float, float]:
    """Hide a parked robot the path already clears, unless the face is in it.

    Pursuit treats a short flank beside a short front as a corner and yaws
    off the detour. The detour is the clearance. A return under ``floor``
    stays, so a chassis that has left the path still stops.
    """
    def lift(dist: float, bearing: float) -> float:
        if dist >= 900.0 or dist < floor or not peers:
            return dist
        if body_is_cleared_peer(x, y, yaw, None, None, dist, bearing, peers):
            return max(dist, opened)
        return dist

    return lift(front, front_bearing), lift(left, left_bearing), lift(right, right_bearing)


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
    left_bearing: float | None = None,
    right_bearing: float | None = None,
    dest_x: float | None = None,
    dest_y: float | None = None,
    open_front: float | None = None,
    open_bearing: float | None = None,
    open_left: float | None = None,
    open_right: float | None = None,
    open_left_bearing: float | None = None,
    open_right_bearing: float | None = None,
    cleared_peers: Sequence[Tuple[float, float]] = (),
) -> Tuple[Tuple[float, float] | None, AvoidanceState]:
    """Pass another robot, or ``None`` to follow the path.

    The trigger is the first project's: the forward return is closing inside
    1.80 m. A wall or shelf already on the map matches ``map_range`` and does
    not count, including while the nose is still turning onto the line. The
    side and the speeds are the first project's, but the offset stays on a
    line parallel to the path so the pass cannot walk into the flank wall.
    It ends once the other robot has left the forward sector and the trailing
    flank, the chosen flank drops under 0.50 m, or both flanks are a corridor.
    A walker coming along the path is not gone just because that yaw carried
    them out of those sectors: the offset stays until they are behind the chassis.
    A moving robot met on its right is not passed on our right: that cut
    crosses its bow. We stop when our trajectory stays to its right, and pass
    behind it on our left when our trajectory goes to its left. A walker on
    that line is crossed in front when there is time; otherwise the stop is
    backed off the line and turned until a flat face meets them. A walker
    coming along the path, head-on or just off it, keeps the side offset until
    the chassis is clear of them. When the goal is already past that walker
    and they are far enough to the side that the straight run stays clear, the
    robot follows the path.
    """
    if path_x is None:
        path_x = x
    if path_y is None:
        path_y = y
    if dest_x is None:
        dest_x = to_x
    if dest_y is None:
        dest_y = to_y
    # ``front`` is the closest return, shelf included. ``open_front`` is the
    # closest return the map does not explain, so a shelf corner in the same
    # sector does not hide a walker who is still further along that sector.
    see_front = front if open_front is None else open_front
    see_bearing = bearing if open_bearing is None else open_bearing
    see_left = left if open_left is None else open_left
    see_right = right if open_right is None else open_right
    see_left_bearing = left_bearing if open_left_bearing is None else open_left_bearing
    see_right_bearing = right_bearing if open_right_bearing is None else open_right_bearing
    pace = min(front, see_front)
    previous = state.last_front
    # An empty sweep is 999 m. Folding that into the rate makes the next
    # real return look like a lunge, and the approach count that starts a
    # pass is wiped. The robot then yields to a chassis that never moved.
    if see_front < 900.0:
        closing, last_front = update_closing_speed(
            state.last_front, see_front, state.closing, dt=dt,
        )
    else:
        closing = state.closing
        last_front = state.last_front
    on_map = front_is_on_the_map(see_front, map_range)
    hard_open = previous is not None and see_front > previous + 0.10
    approaching = (
        previous is not None
        and see_front < previous - 0.004
        and -1.2 < closing < -0.08
        and see_front < 1.80
        and not on_map
    )
    # One empty sweep in the middle of an approach. Zeroing the count
    # there is the pass that never starts. A second empty sweep in a row
    # still drops it: ``missed`` records the kept frame.
    # A wobbling range opens by a few centimetres on the next sample while
    # the smoothed rate is still closing. That open used to zero the count
    # every other frame, so a walker and a parked robot never got a pass.
    if approaching:
        frames = state.approach_frames + 1
    elif (
        see_front >= 900.0
        and 0 < state.approach_frames < 6
        and state.missed == 0
    ):
        frames = state.approach_frames
    elif (
        state.approach_frames > 0
        and not hard_open
        and -1.2 < closing < -0.08
        and see_front < 1.80
        and not on_map
    ):
        frames = state.approach_frames + 1
    else:
        frames = 0
    # A turn onto the leg used to wipe the approach, so a walker already
    # coming along the path never gathered three frames. The wall still
    # wipes it: that return matches the map and has no tracked approach.
    coming_before = (
        not on_map
        and state.track_x is not None
        and _moving_along_path(path_ux, path_uy, state.track_vx, state.track_vy)
        and (state.track_vx * path_ux + state.track_vy * path_uy) < -0.08
    )
    # The nose yaw slides the short speed window off the path. The centre
    # itself stays on the leg, and that is still the walker in front.
    # A crossing robot this close to the line is stopped by the later
    # gates; a mapped wall never enters this.
    centred_ahead = False
    if not on_map and see_front < 1.80 and closing < -0.08 and state.track_x is not None:
        prev_left = (state.track_x - path_x) * (-path_uy) + (state.track_y - path_y) * path_ux
        prev_along = (state.track_x - x) * path_ux + (state.track_y - y) * path_uy
        centred_ahead = abs(prev_left) < 0.25 and 0.15 < prev_along < 2.20
    if abs(heading_error) > 0.30 and not coming_before and not centred_ahead:
        frames = 0

    # The return is a range, not a centre. Push it forward by about a chassis
    # radius so a stationary body stays put while our own pose changes.
    # Closing speed cannot tell that body from one that is crossing.
    track_x = state.track_x
    track_y = state.track_y
    track_vx = state.track_vx
    track_vy = state.track_vy
    other_speed = state.other_speed
    moving_frames = state.moving_frames
    vel_samples = state.vel_samples
    pos_samples = state.pos_samples
    proved_cross = state.proved_cross
    walker_known = state.walker_known
    # Sustained speed over four tenths of a second. None until that window
    # exists. A shorter window follows a real change; this one is what
    # decides that the body is actually a robot.
    long_speed = None
    long_dir = None
    path_heading_rad = math.atan2(path_uy, path_ux)
    selected = _select_body_ray(
        x, y, yaw, see_front, see_bearing, on_map,
        see_left, see_left_bearing, map_left,
        see_right, see_right_bearing, map_right,
        track_x, track_y,
    )
    # One empty sweep before anyone has latched. Wiping the centre there
    # drops the speed and the lateral window, and the next real return
    # looks like a chassis that never moved: the crossing robot is passed
    # and the one that should have waited drives on. A second empty sweep
    # still drops it. The point stays where it was; advancing it would walk
    # a parked hit into the crossing.
    blank_hold = False
    if selected is None:
        # One missed sweep is not "they have gone past". A head-on walker
        # slides out of the 110° sector while still in front, and wiping the
        # track here is what sent the robot back up the centreline into them.
        # The latest velocity is often sideways: the hit walks around the
        # cylinder as the robot yaws. The latched path velocity is the one
        # that still describes them.
        remembered = (
            state.path_hold
            and state.active
            and state.side != 0.0
            and state.track_x is not None
            and math.hypot(state.hold_vx, state.hold_vy) > 0.05
        )
        coming = state.track_vx * path_ux + state.track_vy * path_uy
        coast = (
            state.track_x is not None
            and (
                remembered
                or (
                    state.active
                    and state.side != 0.0
                    and _moving_along_path(path_ux, path_uy, state.track_vx, state.track_vy)
                    and coming < 0.05
                )
                or state.cross_hold != 0.0
                or state.cross_go
            )
        )
        if coast:
            use_vx, use_vy = state.track_vx, state.track_vy
            if remembered:
                use_vx, use_vy = state.hold_vx, state.hold_vy
            track_x = state.track_x + use_vx * dt
            track_y = state.track_y + use_vy * dt
            track_vx = use_vx
            track_vy = use_vy
            other_speed = state.other_speed
            moving_frames = state.moving_frames
        elif state.track_x is not None and state.missed == 0:
            # Freeze the centre for this one empty sweep, whatever its
            # velocity. Wiping an along-track reading here too made the
            # next return look like a robot that had never moved, and the
            # crossing was driven through. A parked chassis is not yielded
            # to from this hold: the pass below keeps going, and a second
            # empty sweep still drops the track.
            blank_hold = True
            track_x = state.track_x
            track_y = state.track_y
            track_vx = state.track_vx
            track_vy = state.track_vy
            other_speed = state.other_speed
            moving_frames = state.moving_frames
        else:
            other_speed = 0.0
            moving_frames = 0
            vel_samples = ()
            pos_samples = ()
            proved_cross = False
            walker_known = False
            track_x = None
            track_y = None
            track_vx = 0.0
            track_vy = 0.0
        body_range, body_bearing = see_front, see_bearing
    else:
        body_range, body_bearing = selected
        ray = yaw + body_bearing
        cx = x + (body_range + 0.18) * math.cos(ray)
        cy = y + (body_range + 0.18) * math.sin(ray)
        accept_point = True
        early = None
        if track_x is not None and dt > 1e-6:
            hop = math.hypot(cx - track_x, cy - track_y) / dt
            # Above 1.5 m/s the centre jumped around a body: the nose yawed
            # and the hit walked a corner. Leave that point out of the
            # position history. The point itself still moves.
            accept_point = hop < 1.5
            if hop < 1.5:
                inst_vx = (cx - track_x) / dt
                inst_vy = (cy - track_y) / dt
                vel_samples = (vel_samples + ((inst_vx, inst_vy),))[-5:]
                # Under 0.42 m/s this step is the body, not range wobble.
                # The position median needs half a second; a walker dash
                # and a close crossing are already over by then.
                if hop < 0.42 and len(pos_samples) < 5:
                    early = (inst_vx, inst_vy, hop)
        if accept_point:
            pos_samples = (pos_samples + ((cx, cy),))[-9:]

            def median_xy(samples):
                xs = sorted(sample[0] for sample in samples)
                ys = sorted(sample[1] for sample in samples)
                mid = len(xs) // 2
                if len(xs) % 2 == 1:
                    return xs[mid], ys[mid]
                return (
                    0.5 * (xs[mid - 1] + xs[mid]),
                    0.5 * (ys[mid - 1] + ys[mid]),
                )

            def window_velocity(newer, older, frames):
                x1, y1 = median_xy(newer)
                x0, y0 = median_xy(older)
                span = frames * dt
                if span <= 1e-6:
                    return None
                return (x1 - x0) / span, (y1 - y0) / span

            # Two tenths of a second. Exact on a clean track and on an
            # alternating wobble, and it follows a real change of speed
            # before the crossing is over.
            mv = None
            if len(pos_samples) >= 5:
                mv = window_velocity(pos_samples[-3:], pos_samples[-5:-2], 2)
            if len(pos_samples) >= 9:
                lasted = window_velocity(pos_samples[-5:], pos_samples[-9:-4], 4)
                if lasted is not None:
                    long_speed = math.hypot(lasted[0], lasted[1])
                    long_dir = lasted
            if mv is None and early is not None:
                mv = (early[0], early[1])
            if mv is not None:
                mvx, mvy = mv
                med = math.hypot(mvx, mvy)
                # Snap up, so cruise is recognised as soon as the windows
                # agree. Ease down, so a brief slow reading still crosses
                # the walker band instead of skipping it.
                if med >= other_speed:
                    other_speed = med
                else:
                    other_speed = 0.5 * other_speed + 0.5 * med
                moving_frames = (
                    moving_frames + 1 if other_speed >= 0.18 else 0
                )
                track_vx, track_vy = mvx, mvy
        track_x, track_y = cx, cy
    tracked_speed = other_speed if moving_frames >= 3 else 0.0
    body_along = None
    if track_x is not None:
        body_along = (track_x - x) * path_ux + (track_y - y) * path_uy
    clean_along = _moving_along_path(path_ux, path_uy, track_vx, track_vy)
    coming_now = track_vx * path_ux + track_vy * path_uy
    if clean_along and coming_now < 0.08:
        hold_vx, hold_vy = track_vx, track_vy
    else:
        hold_vx, hold_vy = state.hold_vx, state.hold_vy
    # Person radius 0.25 m, rear face 0.15 m, and the centre estimate sits
    # short of the real centre. Releasing at -0.45 m left the cylinder on the
    # rear corner. A hit that slides around the cylinder is not them turning
    # across the path: once this pass has seen them walk it, keep the offset
    # until the estimate is actually behind the chassis.
    path_hold = False
    if (
        state.active
        and state.side != 0.0
        and body_along is not None
        and body_along > -0.70
    ):
        path_hold = state.path_hold or (clean_along and coming_now < 0.08)
    walker_still_crossing = (
        state.active
        and state.side != 0.0
        and body_along is not None
        and body_along > -0.70
        and (path_hold or clean_along)
    )
    off_path = bearing_off_path(yaw, body_bearing, path_heading_rad)
    nose_off = bearing_off_path(yaw, see_bearing, path_heading_rad)
    sight = yaw + body_bearing
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
        and see_front < 1.70
        and off_path < -0.20
    )
    if followed:
        decision = "yield"
    # A walker coming along the path is not cutting across it. Yielding here
    # holds the chassis on the line and the walker meets the side. Offset,
    # unless the goal is already past them with the straight run clear.
    along_path = _moving_along_path(path_ux, path_uy, track_vx, track_vy)
    straight = along_path and _aim_stays_right_and_beyond(
        x, y, path_ux, path_uy, dest_x, dest_y,
        track_x, track_y, track_vx, track_vy,
    )
    if decision is not None and along_path and not straight and not followed:
        decision = None
    # They are off to one side and moving across, the way a side offset would
    # chase. The bearing shrinks to zero as they reach the path, which is
    # when the offset used to start. Judge the side from where they are on
    # the path, and once the stop has begun keep it until the crossing is
    # behind. side_arrival did not claim them, so this is not the pass-behind
    # and not a walker yield. The range has to be closing: a shelf that stays
    # at a fixed range while the nose yaws is not another robot. The robot
    # that reaches the crossing first keeps the straight path. Stopping that
    # one leaves it in front of the robot that should have waited.
    body_left = 0.0
    if track_x is not None:
        body_left = (track_x - path_x) * (-path_uy) + (track_y - path_y) * path_ux
    # Four frames of lateral position. Range noise walks back and forth, so
    # one window can close by about 6 cm on a parked chassis. A robot
    # crossing the line closes by more than that, and the next window
    # agrees. The smoothed speed of a parked hit runs past 0.55 m/s and
    # would otherwise hide this crossing.
    side_closing = state.side_closing
    close_hits = state.close_hits
    if track_x is None:
        left_anchor = 0.0
        anchor_age = 0
        side_closing = False
        close_hits = 0
    elif state.anchor_age == 0 and abs(state.left_anchor) < 1e-6:
        left_anchor = body_left
        anchor_age = 0
    else:
        anchor_age = state.anchor_age + 1
        left_anchor = state.left_anchor
        if anchor_age >= 4:
            closed = abs(left_anchor) - abs(body_left)
            reached = left_anchor * body_left < 0.0 and abs(left_anchor) > 0.15
            if reached or closed > 0.10:
                side_closing = True
                close_hits = 2
            elif closed > 0.06:
                # One window keeps the crossing hold. It does not by itself
                # prove a robot: range wobble closes a single window and
                # the next one opens. Two windows, or a 10 cm closure, do.
                side_closing = True
                close_hits = close_hits + 1
            else:
                close_hits = 0
            left_anchor = body_left
            anchor_age = 0
    # A fast robot already out to the side is crossing. One sample that
    # slides along the line must not freeze the side pass: both robots then
    # arc off and meet again on the way back. Someone ahead, only a little
    # off the line, is the pass itself. A walker on the path stays inside
    # this band.
    if (
        path_hold
        and other_speed >= 0.24
        and body_along is not None
        and abs(body_left) > max(0.30, 0.28 * max(body_along, 0.0))
        # Coming at us along the path is the head-on walker, even when the
        # estimate sits half a metre off the line. Clearing the pass there
        # drives the side back into them. A crossing robot's hit slides
        # forward, so this approach speed stays near zero.
        and coming_now > -0.05
    ):
        path_hold = False
    if body_left < -0.20:
        chase_side = -1.0
    elif body_left > 0.20:
        chase_side = 1.0
    elif state.active and state.side == 0.0:
        cross_vel = path_ux * track_vy - path_uy * track_vx
        chase_side = -1.0 if cross_vel > 0.08 else (1.0 if cross_vel < -0.08 else 0.0)
    else:
        chase_side = 0.0
    cross_line = _walker_line(
        x, y, path_ux, path_uy, track_x, track_y, track_vx, track_vy,
    )
    cross_along = cross_line[3] if cross_line is not None else None
    cross_ahead = cross_line[4] if cross_line is not None else None
    still_across = cross_ahead is None or cross_ahead > -0.40
    order = _crossing_order(cross_along, cross_ahead, tracked_speed, body_left < -0.20)
    clearly_first = _clearly_first(cross_along, cross_ahead, tracked_speed)
    body_approaching = False
    hop_fast_across = False
    robot_hop = False
    if state.track_x is not None and track_x is not None and dt > 1e-6:
        prev_d = math.hypot(state.track_x - x, state.track_y - y)
        new_d = math.hypot(track_x - x, track_y - y)
        body_approaching = (new_d - prev_d) / dt < -0.08
        # The smoothed speed lags a robot that is already crossing. One hop
        # at cruise is enough to refuse the side pass; a walker is below 0.24.
        hop_vx = (track_x - state.track_x) / dt
        hop_vy = (track_y - state.track_y) / dt
        hop_speed = math.hypot(hop_vx, hop_vy)
        hop_cross = abs(path_ux * hop_vy - path_uy * hop_vx)
        # A head-on walker slides around the cylinder for one sample. That hop
        # is only partly across. A robot that cuts the path, even with a
        # little forward speed, is almost pure cross-track.
        hop_fast_across = (
            # Above 1.5 m/s the centre estimate jumped: the nose yawed and the
            # hit walked around a stationary body. The speed filter ignores
            # that same hop. It is not a robot crossing.
            0.24 <= hop_speed < 1.5
            and hop_cross >= 0.85 * hop_speed
            and not _moving_along_path(path_ux, path_uy, track_vx, track_vy)
        )
        # Up to the cut-across samples used in tests. Faster than that is
        # the return sliding around a walker, and it must not start a yield.
        # The smoothed speed has to already be at cruise. One hop while it
        # is still in the walker band is our own motion past a parked robot,
        # and yielding on it latches a stop that never ends.
        robot_hop = (
            hop_fast_across
            and hop_speed < 0.70
            and other_speed >= 0.24
        )
    clips = _line_reaches_chassis(
        x, y, path_ux, path_uy, track_x, track_y, track_vx, track_vy,
    )
    along_v = abs(path_ux * track_vx + path_uy * track_vy)
    cross_v = abs(path_ux * track_vy - path_uy * track_vx)
    # A walker coming at us can show one sideways hop. The smoothed velocity
    # is still along the path, and that is not a robot to yield to.
    along_dominates = along_v >= 0.08 and along_v > cross_v
    # The short window follows the hit around the cylinder while the nose
    # yaws. The longer window is still the walker's own motion.
    long_along = False
    if long_dir is not None:
        long_cross = abs(path_ux * long_dir[1] - path_uy * long_dir[0])
        long_along_v = abs(path_ux * long_dir[0] + path_uy * long_dir[1])
        long_coming = long_dir[0] * path_ux + long_dir[1] * path_uy
        long_along = (
            long_along_v >= 0.08
            and long_along_v > long_cross
            and long_coming < -0.08
        )
    # Someone walking toward us along the path. A crossing robot's speed is
    # across the leg, and a parked chassis does not close along it. The
    # speed estimate of this walker climbs past 0.24 m/s while the nose yaws,
    # and that used to be read as a robot already on the line: the offset
    # never started. Half a metre off the leg is still this approach.
    coming_along = (
        not on_map
        and see_front < 1.80
        and ((along_dominates and coming_now < -0.08) or long_along)
        and body_along is not None
        and body_along > -0.10
        and abs(body_left) < 0.55
    )
    # The hit of a body that is not crossing walks forward with us. A robot
    # that is crossing still has a sideways share, even while the range wobbles.
    slide_along = along_dominates and cross_v < 0.08
    # 0.24 m/s sits above a walker (0.20 to 0.22) and below the 0.30 cruise.
    # The third moving frame is the usual gate. Two frames already at 0.24
    # are enough: the left pass used to start on that earlier frame, before
    # the speed was allowed to count, and the later robot had left the line.
    # A walker must keep the dash and the square stop.
    confirmed = tracked_speed
    if moving_frames >= 2 and other_speed >= 0.24:
        confirmed = max(confirmed, other_speed)
    # ``pace`` stays the lidar range. The pass command treats it as metres
    # of clearance; writing the other robot's speed here made every offset creep.
    if track_x is None:
        seen_fast = False
        cross_go = False
    else:
        # Three frames at robot cruise, still under the speed a parked
        # chassis ratchets to. One noisy hop past 0.24 m/s is a walker
        # whose return slid. Past 0.55 m/s before three pure cross-track
        # frames, the smoothed speed has climbed on a body that is not
        # moving, and latching it blocks the pass for good.
        # A parked chassis, and a walker coming along the path, slide forward
        # as we move. That reading stays under a real cross-track share.
        # Calling it cruise blocks the pass and drives straight at them.
        if (
            moving_frames >= 3
            and 0.24 <= other_speed < 0.55
            and not (along_dominates and cross_v < 0.08)
            # The short window spikes on range noise. Four tenths of a
            # second at a walker's pace is not a robot.
            and long_speed is not None
            and long_speed >= 0.24
        ):
            seen_fast = True
        elif long_speed is not None and long_speed < 0.24:
            seen_fast = False
        elif moving_frames >= 3 and other_speed < 0.24 and not (
            # The hit slows once the other robot draws level. They were
            # already at cruise and they are still in front. Forgetting
            # that is what starts the side pass on the next sweep. A walker
            # never sets this flag.
            state.seen_fast
            and body_range < 2.00
            and body_along is not None
            and -0.40 < body_along < 1.80
            and abs(body_left) < 1.40
        ):
            seen_fast = False
        else:
            seen_fast = state.seen_fast
        # Held a walker's pace. The spike that sets ``seen_fast`` is the
        # return walking around the cylinder once the nose yaws beside them,
        # and straight ahead is how that pass was driven through. A robot
        # is already at cruise while the return is still a metre out.
        if (
            not state.seen_fast
            and long_speed is not None
            and 0.12 <= long_speed < 0.24
            and moving_frames >= 6
        ):
            walker_known = True
        elif (
            walker_known
            and long_speed is not None
            and long_speed >= 0.28
            and body_range > 1.00
            and moving_frames >= 3
        ):
            walker_known = False
        cross_go = state.cross_go
    # First at the crossing: stay on the path. An offset here follows the
    # other robot. A line that clips the chassis is still the walker dash
    # or the square stop, and a pass behind is the aim going to their left.
    drop_follow = (
        order == "first"
        and _mostly_across(path_ux, path_uy, track_vx, track_vy)
        and not along_path
        and decision != "behind"
        and not clips
    )
    chase = (
        decision is None
        and chase_side != 0.0
        and not along_path
        and still_across
        and selected is not None
        and body_range < 1.80
        and (
            closing < -0.08
            or body_approaching
            or (state.active and state.side == 0.0)
        )
        and _follows_across(path_ux, path_uy, track_vx, track_vy, chase_side)
        and order == "second"
        # A robot at cruise is the right-of-way rule below, not this chase.
        and confirmed < 0.24
    )

    def parked(
        active: bool, side: float, started: float, until: float, count: int,
        cross_hold: float = 0.0,
    ) -> AvoidanceState:
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
            track_vx=track_vx,
            track_vy=track_vy,
            cross_hold=cross_hold if active else 0.0,
            path_hold=path_hold if active and side != 0.0 else False,
            hold_vx=hold_vx,
            hold_vy=hold_vy,
            seen_fast=seen_fast,
            cross_go=cross_go,
            still_for=still_for,
            missed=missed,
            cross_frames=cross_frames,
            left_anchor=left_anchor,
            anchor_age=anchor_age,
            side_closing=side_closing,
            close_hits=close_hits,
            vel_samples=vel_samples,
            pos_samples=pos_samples,
            proved_cross=proved_cross,
            walker_known=walker_known,
        )

    # Priority to the right. Who is closer is a lidar range, and that estimate
    # flipped between identical runs, so each robot did something else.
    # The other robot's side of our path is the same fact for both of them:
    # the one that has the other on its right stops, the other drives the path.
    # A walker is slower than 0.24 m/s. A latched head-on offset is not this.
    # The range has to be closing before the decision latches: a shelf that
    # stays at a fixed range while the nose yaws looks like a fast robot.
    range_closing = closing < -0.08 or body_approaching
    # Across the path, and not the along-track slide of a body we are
    # driving past. One frame is a corner of a parked chassis.
    still_crossing = (
        cross_v >= 0.12
        and cross_v > along_v + 0.04
        and other_speed >= 0.18
    )
    if still_crossing and along_v <= 0.04 and cross_v >= 0.15:
        cross_frames = state.cross_frames + 1
    else:
        cross_frames = 0
    # A parked chassis can close one four-frame window. After a second of
    # that reading, a yield is a reverse beside a robot that never moved.
    # A crossing has either closed two windows or kept a pure cross-track
    # speed. The first second is left alone: the stop has to start before
    # those windows fill.
    late_parked = (
        moving_frames >= 12
        and close_hits < 2
        and cross_frames < 3
    )
    # Latched. One noisy frame must not forget a crossing that already
    # closed a window or kept a pure cross-track speed. A parked chassis
    # does neither, so a yield that started on the slide can still end.
    proved_cross = proved_cross or cross_frames >= 3 or close_hits >= 2
    # A parked chassis ratchets the smoothed speed past 0.55 m/s while its
    # lateral offset stays put. A crossing robot's first sample is near
    # 0.4 m/s. Once three pure cross-track frames have landed, the higher
    # speed is that robot and the yield already under way must keep it.
    glitch_speed = other_speed >= 0.55 and cross_frames < 3
    # Already beside the line, the meeting is this robot. Farther out, the
    # same speed is a parked chassis whose hit is sliding, unless the
    # lateral window shows them walking onto the path.
    honest_cross = side_closing or cross_frames >= 3 or (
        # Already beside the line and still at cruise. A parked chassis
        # 0.40 m off the path sits in this band too, once its hit has
        # slid. The first samples of a real crossing are still this:
        # the stop has to start before the lateral window fills. A hit
        # that has been "moving" for a second without ever closing on
        # the path is the parked chassis, and the yield there reverses.
        abs(body_left) < 0.50
        and not glitch_speed
        and other_speed >= 0.24
        and moving_frames < 12
        # One hop before the position median exists, and still on the
        # forward ray, is the head-on walker. A body already aside is the
        # crossing and cannot wait for that median.
        and (len(pos_samples) >= 5 or abs(body_left) >= 0.20)
        # The hit of a parked chassis slides along the path. That is not
        # a robot already beside the line.
        and cross_v >= 0.12
    )
    # Already beside us, the smoothed speed is still catching up. The cross
    # component is the robot. A walker this close is the head-on offset,
    # which is latched, or a line that still clips, which is further off
    # the 0.25 m band.
    beside_cross = (
        body_range < 0.55
        and abs(body_left) > 0.25
        and cross_v >= 0.08
        and cross_v > along_v + 0.04
        # Three frames still under 0.24 m/s is a walker, not a late robot sample.
        and not (moving_frames >= 3 and other_speed < 0.24)
    )
    # Their centre already lies on our line, including while they are still
    # on the approach. The 0.15 m side gate stays shut, and a side pass then
    # starts as if they were standing on the path: the yielder yaws off the
    # line and finishes the crossing beside it. Own that robot out to the
    # range where the pass would otherwise begin.
    on_the_line = (
        abs(body_left) <= 0.15
        and body_range < 2.20
        and body_along is not None
        and -0.20 < body_along < 1.20
        and (seen_fast or other_speed >= 0.24 or hop_fast_across)
        and not coming_along
    )
    # A face sliding along the near side looks like motion along our path.
    # That veto is for a walker on the path. A centre 0.35 m off the line
    # is the other robot, and the slide must not cancel the yield.
    clearly_aside = abs(body_left) > 0.35 and (
        seen_fast or other_speed >= 0.24 or hop_fast_across
    )
    # Three slow frames is a walker. A sideways hop must not promote them
    # into the robot rule: that latch is the reverse in front of a person.
    # A crossing robot can read 0.21 m/s, inside that band, for the whole
    # approach. Three pure cross-track frames are that robot, unless the
    # line already clips the chassis: that reading is the pedestrian dash.
    # A line that already clips the chassis is the pedestrian dash, while
    # they are still on our left. Once they have crossed onto our right,
    # the same slow reading is someone we have to yield to.
    crossing_slow = cross_frames >= 3 and (not clips or body_left < -0.15)
    settled_walker = (
        moving_frames >= 3 and other_speed < 0.24 and not crossing_slow
    )
    # Already beside the chassis, the hit slides forward as we move, so the
    # tracked velocity follows our path and never looks like a crossing.
    # The side is still the right of way. A parked robot stays near 0 m/s,
    # so this does not replace the pass around one.
    beside_fast = (
        body_range < 0.55
        and abs(body_left) > 0.25
        and (seen_fast or other_speed >= 0.18)
        and not settled_walker
    )
    # Out to the side, not merely ahead of us. An obstacle in front can sit
    # 0.20 m off the line while the nose is still yawed; the closing range
    # then looks like a robot and a side pass is the right answer. A body
    # this far off the forward ray is crossing. The hit slides along our
    # path, so the velocity is the wrong test. A walker stays under 0.24 m/s
    # and a parked robot stays near 0, so neither one is this.
    crossing_shape = (
        body_along is not None
        and abs(body_left) > 0.20
        and abs(body_left) > 0.28 * max(body_along, 0.0)
        and not path_hold
        and not settled_walker
        and (not glitch_speed or side_closing or cross_frames >= 3)
        and (seen_fast or other_speed >= 0.08 or moving_frames >= 1)
    )
    off_axis = crossing_shape and (seen_fast or other_speed >= 0.24)
    # Still the robot we were crossing, after the speed reading has fallen.
    # The pass must not start; path following is the rest of the answer.
    robot_still_there = (
        seen_fast
        and not glitch_speed
        # A head-on walker and a parked chassis slide along the path.
        # A crossing robot still has a sideways share. Blocking the pass
        # on the along-track reading drives straight at them.
        and cross_v >= 0.12
        and body_range < 2.00
        and body_along is not None
        and body_along > -0.40
        and abs(body_left) < 1.60
        and not path_hold
        and not coming_along
    )
    # Both chassis are already beside each other and stopped. Pursuit sees a
    # clear front and drives into the neighbour during the frames before the
    # speed estimate exists. The side of the path is still the right of way.
    # A side pass already taken around a parked robot is left alone.
    pass_underway = (
        state.active and state.side != 0.0 and state.cross_hold == 0.0
    )
    # Positive side is our own right. The body sitting on the opposite side
    # means the offset is taking us away from them: that is the pass of a
    # robot that is not moving. An offset toward the body is the arc that
    # follows someone across the line, and it still has to stop at once.
    # 0.20 m is outside the on-the-line yield, so a real crossing still
    # cancels an offset that has only just left the axis.
    passing_away = (
        (state.side < 0.0 and body_left < -0.20)
        or (state.side > 0.0 and body_left > 0.20)
    )
    # The on-the-line yield starts at 0.05 m. An offset already leaving that
    # side is still the pass, while the estimate sits inside the 0.20 m band.
    passing_clear = (
        (state.side < 0.0 and body_left < -0.05)
        or (state.side > 0.0 and body_left > 0.05)
    )
    beside_stopped = (
        selected is not None
        and body_range < 0.55
        and body_along is not None
        and -0.20 < body_along < 0.55
        and abs(body_left) > 0.25
        and not settled_walker
        and not (path_hold and state.side != 0.0)
        and not pass_underway
    )
    beside_right = beside_stopped and body_left < -0.15
    beside_left = beside_stopped and body_left > 0.15 and front < 0.45
    # Inside this band the two squares already share the crossing. Waiting
    # for a speed sample leaves the yielder on the other robot's path.
    tight_beside = beside_right and body_left > -0.50
    if beside_right and other_speed < 0.05 and not seen_fast:
        still_for = state.still_for + dt
    else:
        still_for = 0.0
    # Long enough to be a parked robot, and far enough aside that backing
    # up is not what clears the chassis. A body still inside the crossing
    # keeps the yield: releasing it there drives straight back into them.
    parked_beside = still_for > 1.0 and not tight_beside
    crossing_owned = (
        (not along_path or clearly_aside or beside_fast or off_axis)
        and (not along_dominates or clearly_aside or beside_fast or off_axis)
        and body_range < 2.40
        # The centre estimate sits short of the real chassis, so a robot
        # 0.40 m off the line can read as 0.20 m. 0.15 m is that reading.
        and (abs(body_left) > 0.15 or on_the_line)
        and not (path_hold and state.side != 0.0)
        and not settled_walker
        # A remembered cruise with no motion left is a hit that stopped
        # sliding. Owning the crossing from that memory re-latches the yield
        # after it was released and both robots stay put.
        and (
            other_speed >= 0.24
            or beside_cross
            or beside_fast
            or robot_hop
            or crossing_slow
            # Already in the meeting and moving across, before the smoothed
            # speed reaches cruise. A dropped sweep would otherwise look
            # empty and the chassis drives into them. A parked robot is
            # still further ahead than this, or the hit is sliding along
            # the path.
            or (
                other_speed >= 0.18
                and body_range < 0.80
                and body_along is not None
                and body_along > 0.15
                and body_left < -0.15
                and not along_dominates
            )
            # A cruise we already saw, still ahead. The same memory must not
            # steal a side pass of a parked robot, and it must not start once
            # the hit is beside or behind: the parked chassis never crosses,
            # so that stop reverses and then sits there.
            or (
                seen_fast
                and other_speed >= 0.12
                and body_along is not None
                and body_along > 0.05
                and not (
                    state.active
                    and state.side != 0.0
                    and state.cross_hold == 0.0
                )
            )
        )
        and (
            _mostly_across(path_ux, path_uy, track_vx, track_vy)
            or hop_fast_across
            or beside_cross
            or beside_fast
            or on_the_line
            or off_axis
        )
    )
    lateral_mover = crossing_owned or (
        hop_fast_across
        and not along_path
        and not along_dominates
        and not settled_walker
        and abs(body_left) > 0.15
        and body_range < 2.40
        and not (path_hold and state.side != 0.0)
    )
    latched_yield = state.cross_hold != 0.0
    latched_go = state.cross_go and not latched_yield
    # They came from our right (hold < 0) and their centre is past our path.
    # 0.50 m on the short estimate is about 0.70 m on the real centre, which
    # clears the chassis. Waiting for 0.70 m kept the yield latched, and the
    # along-track projection stayed inside the reverse window.
    passed_yield = (
        (state.cross_hold < 0.0 and body_left > 0.50)
        or (state.cross_hold > 0.0 and body_left < -0.50)
    )
    # An empty sector is not a range. The coasted track is where they were.
    # Treating the empty reading as "far" dropped the yield for one frame
    # and the next frame started a side pass.
    if selected is None and track_x is not None:
        range_now = math.hypot(track_x - x, track_y - y)
    else:
        range_now = body_range
    clear_of_us = (
        track_x is None
        or range_now > 2.10
        or (body_along is not None and body_along < -0.80)
    )
    # Give up a latched priority only when they have actually crossed onto
    # our right and the meeting is still in front. A noisy sample does not.
    # Onto our right, and still moving that way. A single sample on the wrong
    # side, without that direction, is not this cut.
    cutting_right = path_ux * track_vy - path_uy * track_vx < -0.05
    # Only a cut that is still ahead of the chassis. The perpendicular robot
    # passes beside the nose; treating that as a new yield made the priority
    # robot reverse after the crossing.
    cut_over = (
        latched_go
        and selected is not None
        and body_left < -0.15
        and cutting_right
        and body_along is not None
        and body_along > 0.40
    )
    # Closing range: they are still coming in. A robot already beside the
    # chassis holds a short range. A robot cutting across can open the range
    # while it changes side. A shelf that only yaws keeps a fixed range, so
    # ``closing`` stays near zero and this does not claim it.
    engaging = range_closing or body_range < 1.05 or closing > 0.04
    # The stop is where the lidar places the other robot, not the fact that
    # the two paths cross somewhere. More than about 1.7 m off our line means
    # they are still far up their aisle. Stopping there freezes the robot on
    # the left while the crossing is empty. Once our own estimate is already
    # on their line, stopping would plant the chassis in the crossing, so
    # that approach is driven through.
    # ``cross_ahead`` < 0 means they have already gone past the point where
    # the two lines meet. Waiting after that only freezes the robot that
    # still had the crossing in front, while the other drives away.
    # A fresh yield is refused once they are 0.45 m past that point. The
    # centre stays on our right and still ahead along the aisle, so the
    # crossing rule used to take the stop back on the next frame and hold
    # it until they left the lidar.
    # A wobbling sweep locks the smoothed velocity onto the wrong direction,
    # and the crossing then looks already behind for the rest of the approach.
    # The lateral window is the position: two closures mean they are still
    # walking onto the line, on the side we have to yield for.
    already_through = (
        cross_ahead is not None
        and cross_ahead < -0.45
        and not (
            close_hits >= 2
            and body_left < -0.15
            and body_along is not None
            and body_along > -0.10
        )
    )
    # A parked chassis wobbles along the lidar ray. The smoothed velocity
    # is then mostly along our path, with a small cross-track part, and
    # that used to start a yield. A robot that is actually crossing is
    # already near the line, and the cross-track part is a real share of
    # the motion. Farther out, or mostly along the path, it is the slide.
    noisy_along = along_dominates and not (
        cross_v >= 0.12
        and cross_v >= 0.30 * max(along_v, 1e-6)
        and abs(body_left) < 0.45
        and body_range < 1.05
        and other_speed < 0.55
    )
    # ``body_along`` is where their centre sits on our path. The near face
    # reads as abreast while they are still short of the line, and the stop
    # then never starts. Their own distance to the crossing is the other
    # lidar fact, but only once they are already within about a metre of
    # our line. Further out, that distance is just the aisle.
    still_short = body_along is not None and body_along > 0.05
    still_coming = (
        cross_ahead is not None
        and cross_ahead > 0.20
        and abs(body_left) < 1.05
    )
    yield_seen = (
        selected is not None
        and abs(body_left) < 1.70
        and (still_short or still_coming)
    )
    role = None
    # A missed sweep reports 999 m. The releases below would read that as
    # "far" and drop a yield that the coasted track still has in front.
    if selected is None and track_x is not None:
        body_range = math.hypot(track_x - x, track_y - y)
    # The sweep missed them. Coasting a crossing track after that kept a
    # phantom in front, and the yield flipped to a straight command and back
    # every frame: one robot reversed forever after the other had left.
    if (latched_yield or latched_go) and selected is None:
        # One missed sweep. Gazebo drops a robot between sectors for a frame
        # and the next sample used to pick a side pass instead of the yield.
        held_near = (
            track_x is not None
            and body_along is not None
            and body_along > -0.50
            and math.hypot(track_x - x, track_y - y) < 1.80
            and state.missed < 3
        )
        if held_near:
            missed = state.missed + 1
        else:
            role = "release"
            missed = 0
    else:
        # The approach count was kept across one empty sweep. Remember that,
        # so the next empty sweep drops it instead of growing a pass toward
        # a robot that has left the scan.
        missed = (
            1
            if (
                selected is None
                and (
                    blank_hold
                    or (
                        frames > 0
                        and frames == state.approach_frames
                    )
                )
            )
            else 0
        )
    # The path already stays outside this chassis. A side pass or a stop
    # here leaves that line and the next command drives back into the robot.
    # Someone who is crossing is not in this list.
    if role is None and body_is_cleared_peer(
        x, y, yaw, track_x, track_y, see_front, see_bearing, cleared_peers,
    ):
        return None, parked(False, 0.0, state.started, state.until, frames)
    if (
        role is None
        and latched_yield
        and settled_walker
        and not seen_fast
        and not still_crossing
        # Still on our right and not past the chassis. The speed estimate
        # fell into the walker band before three cruise frames. Releasing
        # that stop is the offset through the crossing. A person on the
        # path is not this far off the line.
        and not (
            body_range < 1.05
            and body_along is not None
            and -0.15 < body_along < 0.80
            and abs(body_left) < 1.20
        )
    ):
        # A person coming at us. One sideways sample latched the robot
        # stop, and the reverse then held for as long as they walked.
        # A robot already seen at cruise keeps the stop: the hit slows
        # as it slides, and releasing it there starts the side pass.
        # A robot whose hit never reaches 0.24 m/s is still crossing:
        # ``still_crossing`` is that reading, and the stop stays.
        role = "release"
    elif (
        latched_go
        and settled_walker
        and seen_fast
        and not clear_of_us
        and not (state.active and state.side != 0.0)
        # A parked chassis and a head-on walker also slow into this band.
        # Their hit slides along the path, with nothing left across it.
        and not slide_along
    ):
        # The hit slowed into the walker band. This robot was already at
        # cruise and is still in front. Path following stops on that return:
        # it is inside the forward sector and one flank, so both robots sit
        # short of the intersection. Stay on the line through the dip. A
        # walker never sets ``seen_fast``. An offset already under way is
        # left to the release below, and a robot that has crossed onto our
        # right still reaches ``cut_over`` once the reading is moving again.
        role = "go"
    elif latched_go and settled_walker:
        role = "release"
    elif latched_go and slide_along and cross_frames < 3 and close_hits < 2:
        # Priority was latched on a hit that only slides forward. That is a
        # walker or a robot standing on the path, and straight ahead meets them.
        role = "release"
    elif latched_go and long_speed is not None and long_speed < 0.24:
        # The short window called this a robot. The longer window is a
        # walker, and straight ahead is how that pass was driven through.
        role = "release"
    elif latched_go and walker_known:
        # Already watched at a walker's pace. The spike that latched this
        # go is the hit sliding around the cylinder.
        role = "release"
    elif (
        latched_yield
        and (along_path or along_dominates)
        and not clearly_aside
        and not beside_fast
        and not beside_right
        and not off_axis
        and body_range >= 0.55
        # Still on the side we yielded for, and still coming across. The
        # along-track part is the hit sliding as we move, or the range
        # wobbling along the ray. Once they are on the forward ray that
        # wobble is almost pure along-track, so the cross-track speed falls
        # under 0.12 m/s while their centre is still on the line. The lateral
        # window already saw them walk onto it. Dropping the stop there is
        # the robot that drives through the crossing. A walker along the path
        # never sets that window, so that yield still ends.
        and not (
            body_along is not None
            and body_along > -0.10
            and (
                cross_v >= 0.12
                or (side_closing and body_range < 1.20)
            )
            and (
                (state.cross_hold < 0.0 and body_left < 0.20)
                or (state.cross_hold > 0.0 and body_left > -0.20)
            )
        )
    ):
        role = "release"
    elif latched_yield and parked_beside:
        role = "release"
    elif latched_yield and already_through and body_range > 0.90:
        role = "release"
    elif (
        latched_yield
        and not seen_fast
        and other_speed < 0.12
        and not beside_right
        # The reading dies once we have stopped for them. They are still
        # in the meeting. A parked robot a metre ahead is not: that stop
        # was one sliding sample and it has to end.
        and not (
            body_range < 1.05
            and body_along is not None
            and -0.15 < body_along < 0.80
            and abs(body_left) < 1.20
        )
    ):
        # A parked robot off the line. One sliding sample latched the stop,
        # then holding still brought the reading to zero. A robot that is
        # actually crossing is already at cruise. One beside the chassis
        # keeps the yield through ``beside_right``.
        role = "release"
    elif (
        latched_yield
        and not proved_cross
        and moving_frames >= 12
        and not tight_beside
    ):
        # Watched for a second and never actually crossed. The stop was the
        # slide past a robot that is standing still.
        role = "release"
    elif latched_yield and not passed_yield and not clear_of_us:
        role = "yield"
    elif latched_yield and (passed_yield or clear_of_us):
        # Past the crossing, or far enough behind: follow the path again.
        # Staying in the straight command here was the phantom that reversed.
        role = "release" if clear_of_us else "go"
    elif (
        cut_over
        and selected is not None
        and (range_closing or latched_go)
        and yield_seen
        and not already_through
    ):
        role = "yield"
    elif latched_go and not clear_of_us:
        role = "go"
    elif latched_go and clear_of_us:
        role = "release"
    elif (
        crossing_owned
        and engaging
        and body_left < -0.15
        and not already_through
        # The hit of a parked robot slides forward as we drive. That reading
        # is along our path. Starting the stop there never ends, because the
        # parked robot does not cross the line. A real crossing that is
        # already near the line still has a cross-track share of the motion
        # when the range wobbles. A yield already latched is held above.
        and not noisy_along
        and honest_cross
        and not late_parked
        and (not glitch_speed or side_closing or cross_frames >= 3)
        # A pass already taken around a parked robot. Yawing walks the hit
        # across the ray and the speed climbs, but they are not stepping
        # onto the path. The lateral window also closes while we approach
        # a robot that never moved, so that latch is not the crossing.
        # Replacing the offset with a yield is the stop beside them.
        and not (
            pass_underway
            and passing_away
            and cross_frames < 3
            and not seen_fast
        )
        # Already past their centre. Starting a yield here is the stop after
        # a pass of a parked robot, once the smoothed speed dips back under
        # 0.55 m/s. A crossing that is still in front sits further ahead.
        and not (body_along is not None and body_along <= -0.10)
        and (
            yield_seen
            or (
                # The along-track reading collapses once the faces are in
                # the crossing. A robot still a metre to the side, or
                # already behind, is not that: stopping there freezes the
                # one that had already gone through.
                body_range < 1.05
                and abs(body_left) < 0.70
                and body_along is not None
                and body_along > -0.10
            )
        )
    ):
        role = "yield"
    elif (
        crossing_owned
        and engaging
        and body_left > 0.15
        # A walker at 0.20 m/s crosses this band for the whole approach.
        # Straight ahead is the robot that has priority, and the walker
        # does not stop. A robot already seen at cruise still goes.
        # 0.28 sits above a walker whose return slid a little, and at the
        # cruise a robot actually holds. 0.24 made the pedestrian the robot
        # that has priority.
        and seen_fast
        and not walker_known
        and not slide_along
    ):
        role = "go"
    elif (
        crossing_owned
        and engaging
        and on_the_line
        and body_left < -0.05
        and yield_seen
        and not already_through
        and not noisy_along
        and honest_cross
        and not late_parked
        and (not glitch_speed or side_closing or cross_frames >= 3)
        and not (
            pass_underway
            and passing_clear
            and cross_frames < 3
            and not seen_fast
        )
        and not (body_along is not None and body_along <= -0.10)
    ):
        role = "yield"
    elif (
        crossing_owned
        and engaging
        and on_the_line
        and seen_fast
        and not walker_known
        and not slide_along
        # On the centreline with nothing across it, this is a walker or a
        # robot standing on the path. Straight ahead meets them.
        and cross_v >= 0.08
    ):
        role = "go"
    elif (
        beside_right
        and not parked_beside
        and not already_through
        and not glitch_speed
        # A passer abreast of a parked robot also reads as beside. The
        # smoothed speed of that hit climbs past 0.55 m/s without the body
        # ever crossing, and yielding there is the stutter. Behind the
        # centre, the pass is already done. The pair that starts inside the
        # crossing is still ahead of this, stopped, with a short along gap.
        and body_along is not None
        and body_along > -0.05
        and not late_parked
        and (
            # A parked chassis keeps the cruise flag after the hit stops
            # sliding. That memory, with no cross-track speed left, is not
            # a robot beside us.
            (seen_fast and (other_speed >= 0.12 or side_closing or cross_frames >= 3))
            or other_speed >= 0.12
            or cross_v >= 0.08
            or (body_along < 0.30 and (not seen_fast or other_speed >= 0.12))
        )
    ):
        role = "yield"
    elif beside_left and not seen_fast and other_speed < 0.24:
        # The neighbour fills the front sector. Path following yaws in
        # place on their path. Straight on, without latching, until the
        # speed says whether this is a robot. A parked robot still ahead
        # on this leg is not that neighbour: straight on meets the chassis.
        into_parked = False
        if (
            track_x is not None
            and body_range < 0.45
            and body_along is not None
            and body_along > 0.05
            and 0.06 <= other_speed < 0.20
        ):
            span = math.hypot(track_x - x, track_y - y)
            into_parked = (
                span > 1e-6
                and ((track_x - x) * path_ux + (track_y - y) * path_uy) / span > 0.40
            )
        if not into_parked:
            role = "nudge"
    # The detour already clears every parked robot and the mapped shelves.
    # A return that is not actually moving is the shelf the plan went around,
    # or the hit sliding along it. Yielding or passing there leaves the
    # detour and the stop never ends. A robot at cruise still owns the rule.
    if (
        cleared_peers
        and not path_hold
        and not walker_known
        and not seen_fast
        and cross_frames < 3
        and other_speed < 0.55
        # The detour cleared a parked robot somewhere else. The walker in
        # front of this leg is not that robot and not the shelf.
        and not coming_along
    ):
        return None, parked(False, 0.0, state.started, state.until, frames)
    if role == "nudge":
        return _straight_along_path(yaw, path_ux, path_uy), parked(
            False, 0.0, state.started, state.until, frames,
        )
    if role == "release":
        cross_go = False
        return None, parked(False, 0.0, state.started, state.until, 0)
    if role == "yield":
        cross_go = False
        latch = state.cross_hold if state.cross_hold != 0.0 else (
            -1.0 if body_left <= 0.0 else 1.0
        )
        started = state.started if state.active and state.side == 0.0 else now
        # Two squares meet when the centres are under about 0.50 m. A hard
        # stop on the other's line is a collision once they are beside the
        # chassis. Back up only then, and only until the short centre estimate
        # is 0.50 m ahead. Reversing while they are still on the approach is
        # the backup at the start of the mission. Once they have crossed onto
        # the far side, backing up only grows after they have already passed.
        still_near = (
            (latch < 0.0 and body_left < 0.15)
            or (latch > 0.0 and body_left > -0.15)
        )
        blocking = (
            selected is not None
            and still_near
            # A metre off the line, the other robot has already left the
            # crossing. Backing up there is the late reverse.
            and abs(body_left) < 0.70
            and body_range < 1.05
            and body_along is not None
            and -0.15 < body_along < 0.50
            # Under 0.55 m the faces are already beside each other. Waiting
            # for three cruise frames lets the priority robot close the gap.
            # A body that has not moved at all is a stop, not a reverse,
            # unless the two squares already share the crossing.
            and (
                seen_fast
                or other_speed >= 0.24
                or (body_range < 0.55 and other_speed >= 0.08)
                or tight_beside
            )
            and not settled_walker
        )
        command = (
            _reverse_along_path(yaw, path_ux, path_uy) if blocking else (0.0, 0.0)
        )
        return command, parked(True, 0.0, started, state.until, frames, latch)
    if role == "go":
        cross_go = True
        started = state.started if state.active else now
        return _straight_along_path(yaw, path_ux, path_uy), parked(
            True, 0.0, started, state.until, frames, 0.0,
        )
    if (
        lateral_mover
        and state.active
        and state.side != 0.0
        # One frame at 0.24 m/s is the return walking around a square as the
        # nose yaws. Three frames, or a cruise already remembered, is a robot
        # actually crossing onto this offset. Past 0.55 m/s with no
        # cross-track frame, that walk is a stopped chassis, not the crossing.
        and (seen_fast or cross_frames >= 3 or (other_speed >= 0.24 and moving_frames >= 3))
        and not glitch_speed
        and not along_dominates
        and (
            not passing_away
            or (side_closing and seen_fast)
            or cross_frames >= 3
        )
    ):
        # An arc started before the speed was confirmed. Stop on the line
        # once they are on the side we are offsetting toward. Falling through
        # to path following lets the flank read as a corner and the chassis
        # pivots off the line. Yawing away from a parked chassis walks the
        # hit the same way, and the lateral window closes because we
        # approached. That latch is not them stepping across: ``passing_away``
        # keeps the offset unless we already saw cruise.
        cross_go = False
        return (0.0, 0.0), parked(True, 0.0, state.started, state.until, frames)

    # Confirmed motion: hold still. One or two frames are not enough to call
    # them a crossing robot, but they are enough to refuse the chasing offset.
    # A later sweep can lose the speed estimate as they draw level. The stop
    # stays until they are 0.40 m past the path, on the far side. A hold that
    # was started on a bad estimate is dropped once this robot is clearly the
    # one that arrives first. A small dip in their speed is not that.
    if state.cross_hold != 0.0:
        hold_side = state.cross_hold
    elif chase and tracked_speed >= 0.18:
        hold_side = chase_side
    else:
        hold_side = 0.0
    if clearly_first or settled_walker:
        # A person is not the robot crossing stop. Holding that latch, then
        # dropping it, cancelled the square backoff in front of them.
        hold_side = 0.0
    crossed_past = (
        (hold_side < 0.0 and body_left > 0.40)
        or (hold_side > 0.0 and body_left < -0.40)
    )
    watch_range = body_range
    if selected is None and track_x is not None:
        watch_range = math.hypot(track_x - x, track_y - y)
    # A latched head-on offset is not this stop. One sideways sample used to
    # read as a crossing, freeze the robot on the line, and the walker met
    # the corner when the offset started again.
    if (
        hold_side != 0.0
        and not crossed_past
        and watch_range < 1.80
        and not (path_hold and state.side != 0.0)
    ):
        started = state.started if state.active and state.side == 0.0 else now
        return (0.0, 0.0), parked(True, 0.0, started, state.until, frames, hold_side)

    # The first robot does not sit stopped in the crossing. An offset already
    # taken for someone walking along the path is not this stop: one noisy
    # lateral sample must not throw that pass away.
    # One noisy frame says this robot is first and the line misses the
    # chassis. That frame used to throw away a square stop that was already
    # holding for a walker, and path following then drove into them.
    if drop_follow and state.active and state.side == 0.0 and not walker_known:
        return None, parked(False, 0.0, state.started, state.until, 0)
    if drop_follow:
        decision = None

    def crossing_now() -> bool:
        return (
            decision is not None
            and selected is not None
            and body_range < 1.80
            and abs(heading_error) <= 0.80
        )

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
            and not lateral_mover
            and not crossing_shape
            and not robot_still_there
        ):
            return _evasion_command(
                x, y, yaw, path_ux, path_uy, -1.0, left, right, path_x, path_y, pace,
            ), parked(True, -1.0, now, now, frames)
        # Coming along the path: the stop leaves the side in their way.
        # Take the open side and finish the offset. A crossing robot whose
        # return slid along the ray for one sweep is still off the line.
        if (
            along_path
            and not straight
            and not followed
            and not robot_still_there
            and not (abs(body_left) > 0.20 and other_speed >= 0.24)
        ):
            side = choose_pass_side(
                path_ux, path_uy, to_x, to_y, left, right, bearing=off_path,
            )
            if side is not None:
                return _evasion_command(
                    x, y, yaw, path_ux, path_uy, side, left, right, path_x, path_y, pace,
                ), parked(True, side, now, now, frames)
        # Goal still on their right and already past them: drive the path.
        if decision == "yield" and _aim_stays_right_and_beyond(
            x, y, path_ux, path_uy, dest_x, dest_y,
            track_x, track_y, track_vx, track_vy,
        ):
            return None, parked(False, 0.0, state.started, state.until, 0)
        # Range opening at 1.20 m is not "past": that is the moment the other
        # robot is crossing the line. Wait until the return is far, is the
        # map again, or the other robot has stopped. A walker whose line still
        # clips the chassis is not gone just because the nose looks at a wall.
        command = _yield_command(
            x, y, yaw, path_ux, path_uy,
            track_x, track_y, track_vx, track_vy, other_speed,
        )
        clips = _line_reaches_chassis(
            x, y, path_ux, path_uy, track_x, track_y, track_vx, track_vy,
        )
        info = _walker_line(
            x, y, path_ux, path_uy, track_x, track_y, track_vx, track_vy,
        )
        crossed = (
            info is not None
            and info[3] is not None
            and info[3] < -0.15
            and not clips
        )
        watch_range = body_range if selected is not None else see_front
        watch_map = on_map if selected is None else False
        # Range and the two-second hold used to drop a sprint that still
        # clears the walker. The cruise that replaced it met the cylinder.
        # A square stop is not that sprint, and it still ends here.
        released = (not clips) and command[0] < 0.35 and (
            crossed
            or held >= 2.5
            or watch_map
            or watch_range >= 1.70
            or other_speed < 0.12
        )
        if released:
            return None, parked(False, 0.0, state.started, state.until, 0)
        return command, parked(True, 0.0, state.started, state.until, frames)

    # A robot crossing ahead still projects onto our line, so the side gate
    # stays shut and a pass starts as if they were parked on the path. That
    # yaw is the offset through the crossing. Stay on the line instead.
    # A walker along the path, and a stationary robot, keep the pass.
    crossing_body = (
        body_range < 2.40
        and not along_path
        and not along_dominates
        and not path_hold
        and not settled_walker
        and (not glitch_speed or side_closing or cross_frames >= 3)
        and (hop_fast_across or other_speed >= 0.24 or seen_fast)
    )

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
    # The offset pose is clear of a walker who is still ahead. Following the
    # centreline from there is the early return into them.
    if take_crossing and walker_still_crossing:
        take_crossing = False
    # A robot already crossing is the right-of-way rule, not a left arc.
    if take_crossing and (lateral_mover or crossing_body or off_axis):
        take_crossing = False
    if take_crossing:
        if decision == "yield":
            if _aim_stays_right_and_beyond(
                x, y, path_ux, path_uy, dest_x, dest_y,
                track_x, track_y, track_vx, track_vy,
            ):
                return None, parked(False, 0.0, now, now, frames)
            return _yield_command(
                x, y, yaw, path_ux, path_uy,
                track_x, track_y, track_vx, track_vy, other_speed,
            ), parked(True, 0.0, now, now, frames)
        return _evasion_command(
            x, y, yaw, path_ux, path_uy, -1.0, left, right, path_x, path_y, pace,
        ), parked(True, -1.0, now, now, frames)

    if (
        state.active
        and state.side != 0.0
        and crossing_body
        and (seen_fast or cross_frames >= 3 or (other_speed >= 0.24 and moving_frames >= 3))
        and not glitch_speed
        and (
            not passing_away
            or (side_closing and seen_fast)
            or cross_frames >= 3
        )
    ):
        # The offset is already under way against a robot that is crossing
        # onto that side. Following the path from here turns back into them.
        # Stop on the line. A pass that still has them on the other side is
        # the parked robot. The lateral window also closes while we approach
        # one that never moved, and that latch must not freeze the yaw.
        return (0.0, 0.0), parked(True, 0.0, state.started, state.until, 0)

    if state.active:
        clearance = right if state.side > 0.0 else left
        # Steering-side clearance is the wall. The other robot swings through
        # the trailing flank; a shelf there matches the map and must not hold
        # the pass open, or the robot drives parallel into the next wall.
        if state.side > 0.0:
            trailing, trailing_map = see_left, map_left
        else:
            trailing, trailing_map = see_right, map_right
        # 1.80 m, the same range that starts a pass. 1.20 m dropped the pass
        # once the nose yawed an oncoming walker out of the forward cone and
        # into the flank, before the chassis had moved aside.
        still_there = trailing < 1.80 and not front_is_on_the_map(trailing, trailing_map)
        # Under 0.50 m the chassis no longer fits on that side. Both flanks
        # under 1 m is the shelf-wall gap when those returns are the map.
        # The robot being passed also reads under 1 m: cancelling then drives
        # straight back into it. 0.90 m used to cancel a real pass beside a
        # wall and the robot then drove into the one it was approaching.
        wall = clearance < 0.50 or (gap and not still_there)
        # An empty sweep reads as the wall. The robot being passed is still
        # the coasted centre; ending the offset on that frame turns back
        # into them, and the next pass starts the stutter.
        sweep_missed = (
            see_front >= 900.0
            and track_x is not None
            and (body_along is None or body_along > -0.50)
        )
        passed = (
            on_map
            and not still_there
            and not sweep_missed
            and (now - state.started) >= 0.8
            and see_front >= 0.80
            and not walker_still_crossing
        )
        if wall or passed:
            # The flank is a shelf, or the walker has left the sector. Neither
            # one means they are behind the chassis. Rejoining here is the yaw
            # back onto the line, and the next offset swings the rear corner
            # into them. Creep along the path heading and keep the offset.
            if path_hold and body_along is not None and body_along > -0.70:
                path_h = math.atan2(path_uy, path_ux)
                err = _wrap(path_h - yaw)
                return (0.20, max(-0.5, min(0.5, 1.2 * err))), parked(
                    True, state.side, state.started, state.until, frames,
                )
            return None, parked(False, state.side, state.started, state.until, 0)
        # The latched side is left or right of the leg the pass started on.
        # Once that leg has turned away from the goal, the same side points
        # at a robot that never moved. A straight pass still aims with the
        # goal, so yawing off the centreline is the offset itself and must
        # keep going. A head-on hold aims at the goal too.
        if (
            not seen_fast
            and cross_frames < 3
            and track_x is not None
            and body_range < 1.80
        ):
            toward_x = track_x - x
            toward_y = track_y - y
            span = math.hypot(toward_x, toward_y)
            heading = math.atan2(path_uy, path_ux)
            right_x = math.sin(heading)
            right_y = -math.cos(heading)
            off_x = right_x if state.side > 0.0 else -right_x
            off_y = right_y if state.side > 0.0 else -right_y
            aimed = (
                span > 1e-6
                and (toward_x * off_x + toward_y * off_y) / span > 0.55
            )
            goal_span = math.hypot(dest_x, dest_y)
            leg_reversed = (
                goal_span > 0.40
                and (path_ux * dest_x + path_uy * dest_y) / goal_span < -0.15
            )
            # Level with a robot that never moved, and the offset now points
            # at them. Drop the pass and follow the path. A yawed nose also
            # parks the estimate on the offset while they are still a metre
            # ahead: that one keeps the pass, or the chassis never moves aside.
            if (
                aimed
                and not path_hold
                and not leg_reversed
                and body_along is not None
                and body_along < 0.40
                and body_range < 0.80
            ):
                return None, parked(False, state.side, state.started, state.until, 0)
            # A yawed nose parks the centre estimate on the offset side, so
            # the same dot is true of a straight pass that is working. Only
            # a leg that has turned away from the goal is the drive back in.
            if leg_reversed and span > 1e-6:
                gux = dest_x / goal_span
                guy = dest_y / goal_span
                # Goal still in front of the chassis: keep the same side on
                # that heading. Goal also toward the body: hold still. Path
                # following from here drives along the reversed leg.
                if (toward_x * gux + toward_y * guy) / span > 0.35 and body_range < 1.20:
                    return (0.0, 0.0), parked(
                        True, state.side, state.started, state.until, frames,
                    )
                return _evasion_command(
                    x, y, yaw, gux, guy, state.side, left, right, x, y, pace,
                ), parked(True, state.side, state.started, state.until, frames)
        return _evasion_command(
            x, y, yaw, path_ux, path_uy, state.side, left, right, path_x, path_y, pace,
        ), parked(True, state.side, state.started, state.until, frames)

    if (
        not chase
        and not drop_follow
        # Still gathering the position window. A crossing robot's first
        # samples are range wobble, so the speed is not in yet, and a side
        # pass in those frames leaves the line. A body on the path is the
        # ordinary pass and does not wait.
        and not (
            len(pos_samples) < 9
            and track_x is not None
            and abs(body_left) > 0.20
            and body_range < 2.40
            and not along_path
        )
        # A parked chassis whose smoothed speed has run away still looks
        # like a crossing robot. The pass around it has to start. A body
        # that is actually crossing is not this: three cross-track frames,
        # or a speed still under 0.55 m/s, keep the block.
        and not (lateral_mover and (not glitch_speed or side_closing or cross_frames >= 3))
        and not on_the_line
        and not crossing_body
        and not off_axis
        # Beside the path and already moving: a crossing robot, including
        # the frames before the speed is clearly cruise. An obstacle that
        # is still mostly ahead keeps the ordinary pass.
        and not crossing_shape
        and not robot_still_there
        # They have been walking onto the line. A dropped sweep leaves the
        # speed in the walker band, and a side pass then leaves the lane.
        # Someone walking along the path is the offset, not this. A chassis
        # that is standing still closes the same window, because the path
        # heading changes as we approach: that one is the pass.
        and not (
            side_closing
            and not along_path
            and body_along is not None
            and body_along > -0.10
            and (
                seen_fast
                or other_speed >= 0.12
                or moving_frames >= 3
                or cross_frames >= 1
                # The speed window is not in yet. A robot already crossing
                # looks stopped for those frames, and a side pass leaves
                # the line. A body that is still mostly ahead is not this
                # veto: ``not along_path`` is false once the slide follows
                # the leg, which is the parked chassis.
                or len(pos_samples) < 5
            )
        )
        # On our line, with the lateral window already closing, this is the
        # other robot stepping across. The speed estimate is still near zero
        # on that frame, and the centre can sit just past the 0.20 m gate.
        # A chassis standing on the path does not close the window, so the
        # pass around it still starts. 0.35 m is still inside the crossing.
        and not (
            side_closing
            and abs(body_left) <= 0.35
            and body_along is not None
            and body_along > -0.10
            and not coming_along
        )
        and frames >= 3
        and (abs(heading_error) <= 0.30 or coming_along)
        and not on_map
        and is_oncoming(see_front, closing)
    ):
        # The scan bearing is relative to the nose. The pass has to use the
        # path: the right-hand trajectory stays on the right when that flank
        # is open, even if the nose is still left of the other robot.
        # A leg aimed at a robot already standing to one side reports that
        # return on the path heading. The head-on side is then their side.
        pass_bearing = nose_off
        if abs(body_left) > 0.20:
            pass_bearing = 0.20 if body_left > 0.0 else -0.20
        elif coming_along:
            # Still yawed onto the leg. The nose bearing is not the side.
            pass_bearing = 0.0
        side = choose_pass_side(
            path_ux, path_uy, to_x, to_y, left, right, bearing=pass_bearing,
        )
        if side is not None:
            if coming_along:
                path_hold = True
            return _evasion_command(
                x, y, yaw, path_ux, path_uy, side, left, right, path_x, path_y, pace,
            ), parked(True, side, now, now + 2.2, frames)
    # The leg still runs inside the chassis of a robot that has not moved,
    # and that robot is still ahead. Offset now. Waiting until the range
    # stops closing leaves the swerve for the moment of contact. A body
    # already out at the planned margin, or one that is crossing, is not this.
    if (
        track_x is not None
        and body_along is not None
        and long_speed is not None
        and long_speed < 0.18
        and 0.55 < body_along < 1.70
        and 0.15 < abs(body_left) < 0.55
        and body_range < 1.80
        and moving_frames == 0
        and not on_map
        and not path_hold
        and not seen_fast
        and not walker_known
        and cross_frames < 3
        and not glitch_speed
        and not (
            side_closing
            and abs(body_left) <= 0.35
            and body_along > -0.10
        )
    ):
        if body_left > 0.0:
            flank, flank_map = see_left, map_left
        else:
            flank, flank_map = see_right, map_right
        if flank < 1.00 and not front_is_on_the_map(flank, flank_map):
            pass_bearing = 0.20 if body_left > 0.0 else -0.20
            side = choose_pass_side(
                path_ux, path_uy, to_x, to_y, left, right, bearing=pass_bearing,
            )
            away = side is not None and (
                (body_left > 0.0 and side > 0.0) or (body_left < 0.0 and side < 0.0)
            )
            if away:
                return _evasion_command(
                    x, y, yaw, path_ux, path_uy, side, left, right, path_x, path_y, pace,
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
