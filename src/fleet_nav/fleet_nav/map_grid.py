"""Load the shared warehouse occupancy grid. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml
from PIL import Image

# Centre of a planned path stays at least this far from a parked robot.
# Each chassis reaches about 0.22 m, so the footprints still have a gap
# when the path runs past at this radius.
FLEET_PEER_RADIUS = 0.68

# Below this, a peer is not driving. The plan may treat them as a wall.
PEER_STILL_SPEED = 0.08


def peer_motion_speed(reported: float, displacement_m: float, dt: float) -> float:
    """Speed used to decide that a peer is standing still.

    ``reported`` is the odometry twist. That field stays at zero on some
    bridges while the pose itself is moving, and a robot crossing the path
    was then planned around. Once the window is long enough, the pose change
    is the speed. A step above 1.5 m/s is a fresh sample, not motion.
    """
    if dt < 0.15:
        return reported
    derived = displacement_m / dt
    if derived > 1.5:
        return reported
    return max(reported, derived)


def peer_is_planning_obstacle(speed: float) -> bool:
    """True when the plan should route around this peer."""
    return speed < PEER_STILL_SPEED


def hold_crossing_path(active: bool, side: float, cross_hold: float) -> bool:
    """True while this robot is stopped so another one can cross.

    The path already goes through the meeting point. Replacing it sends
    this robot around the one that has the right of way.
    """
    return active and side == 0.0 and cross_hold != 0.0

# warehouse.sdf shelves, axis-aligned (x0, x1, y0, y1). The saved image
# leaves these boxes unknown or free, so a lidar hit on the face is shorter
# than the grid ray and the contact walks along the face like a walker.
WAREHOUSE_SHELVES = (
    (1.5, 2.5, 0.5, 3.5),
    (-2.5, -1.5, -3.5, -0.5),
)

from fleet_nav.navigation_geometry import (
    astar_grid,
    inflate_occupancy,
    nearest_free_cell,
    path_lookahead,
    simplify_path,
)


def _shelf_range(x: float, y: float, angle: float, max_range: float) -> float:
    """Distance to the first warehouse shelf face, or ``max_range``."""
    dx = math.cos(angle)
    dy = math.sin(angle)
    best = max_range
    for x0, x1, y0, y1 in WAREHOUSE_SHELVES:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return 0.0
        if abs(dx) > 1e-9:
            for edge in (x0, x1):
                travel = (edge - x) / dx
                if 0.0 < travel < best:
                    crossed = y + travel * dy
                    if y0 <= crossed <= y1:
                        best = travel
        if abs(dy) > 1e-9:
            for edge in (y0, y1):
                travel = (edge - y) / dy
                if 0.0 < travel < best:
                    crossed = x + travel * dx
                    if x0 <= crossed <= x1:
                        best = travel
    return best


@dataclass
class OccupancyMap:
    grid: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    display: np.ndarray
    penalty: np.ndarray

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        height, width = self.grid.shape
        return max(0, min(height - 1, row)), max(0, min(width - 1, col))

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def range_to_mapped(
        self,
        x: float,
        y: float,
        yaw: float,
        half_angle: float = 0.61,
        max_range: float = 3.0,
        samples: int = 9,
        lidar_x: float | None = None,
        lidar_y: float | None = None,
    ) -> float:
        """Nearest mapped wall or shelf along ``yaw``, or ``max_range``.

        With ``samples`` above 1, the ray is repeated across ±``half_angle``.
        Unknown cells and the edge of the grid are not walls: a grey cell was
        stopping the ray and every lidar return then looked like the map.
        The shelf boxes are the exception. They are solid in the world and
        missing from the image, and a ray that passes through them reports
        the face. ``lidar_x`` and ``lidar_y`` are the real scan origin when
        ``x, y`` is the point 0.10 m ahead of it: a grazing ray from that
        forward point misses the corner the scan hits.
        """
        nearest = max_range
        if samples <= 1:
            return self._ray_to_mapped(x, y, yaw, max_range, lidar_x, lidar_y)
        for index in range(samples):
            angle = yaw - half_angle + (2.0 * half_angle) * index / (samples - 1)
            nearest = min(nearest, self._ray_to_mapped(x, y, angle, max_range, lidar_x, lidar_y))
        return nearest

    def _ray_to_mapped(
        self,
        x: float,
        y: float,
        angle: float,
        max_range: float,
        lidar_x: float | None = None,
        lidar_y: float | None = None,
    ) -> float:
        step = max(self.resolution, 0.02)
        height, width = self.display.shape
        limit = _shelf_range(x, y, angle, max_range)
        if lidar_x is not None and lidar_y is not None:
            limit = min(limit, _shelf_range(lidar_x, lidar_y, angle, max_range))
        travelled = 0.0
        while travelled < limit:
            travelled += step
            px = x + math.cos(angle) * travelled
            py = y + math.sin(angle) * travelled
            col = int((px - self.origin_x) / self.resolution)
            row = int((py - self.origin_y) / self.resolution)
            if row < 0 or col < 0 or row >= height or col >= width:
                # Past the saved map is not a wall. Stopping here made every
                # return look explained and the robot never passed anyone.
                # A shelf beyond the image is still the face the lidar hits.
                if limit >= max_range:
                    return max_range
                continue
            if int(self.display[row, col]) == 100:
                return travelled
        return limit

    def plan(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        escape_m: float = 0.60,
    ) -> list[tuple[float, float]]:
        """Path on the inflated grid. A pose inside that margin snaps to free space."""
        radius = max(1, int(escape_m / self.resolution))
        start = nearest_free_cell(self.grid, *self.world_to_cell(*start_xy), radius)
        goal = nearest_free_cell(self.grid, *self.world_to_cell(*goal_xy), radius)
        if start is None or goal is None:
            return []
        if start == goal:
            return [self.cell_to_world(*goal)]
        cells = astar_grid(self.grid, start, goal, clear_radius=1, penalty=self.penalty)
        path = [self.cell_to_world(row, col) for row, col in cells]
        # The raw grid path is a staircase. Its heading flips 45° on every
        # cell, so the robot stops to turn while passing an edge it could
        # clear. The shortcut keeps the clearance of those cells.
        return simplify_path(
            path,
            self.clearance,
            self.segment_free,
            resolution=self.resolution,
        )

    def plan_around(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        mates: Sequence[tuple[float, float, float]],
        radius: float = FLEET_PEER_RADIUS,
        escape_m: float = 0.60,
    ) -> list[tuple[float, float]]:
        """Plan as usual, with each parked robot blocked out to ``radius``.

        ``mates`` are ``(x, y, yaw)`` of bases that are not going to move.
        The stored map is left alone: lidar rays still match the walls, and
        only this plan treats those robots as obstacles. An empty list is the
        ordinary plan.
        """
        if not mates:
            return self.plan(start_xy, goal_xy, escape_m)
        grid = self.grid.copy()
        height, width = grid.shape
        cells = max(1, int(math.ceil(radius / self.resolution)))
        limit = cells * cells
        for mate_x, mate_y, _mate_yaw in mates:
            row, col = self.world_to_cell(mate_x, mate_y)
            for d_row in range(-cells, cells + 1):
                for d_col in range(-cells, cells + 1):
                    if d_row * d_row + d_col * d_col > limit:
                        continue
                    blocked_row = row + d_row
                    blocked_col = col + d_col
                    if 0 <= blocked_row < height and 0 <= blocked_col < width:
                        grid[blocked_row, blocked_col] = 1
        blocked = OccupancyMap(
            grid,
            self.resolution,
            self.origin_x,
            self.origin_y,
            self.display,
            self.penalty,
        )
        return blocked.plan(start_xy, goal_xy, escape_m)

    def clearance(self, x: float, y: float) -> float:
        """Metres to the nearest known wall. Chebyshev, so it never over-reads."""
        row, col = self.world_to_cell(x, y)
        return float(self._clearance_grid()[row, col])

    def _clearance_grid(self) -> np.ndarray:
        cached = getattr(self, "_clearance_m", None)
        if cached is not None:
            return cached
        occupied = self.display == 100
        dist = np.full(occupied.shape, 10_000, dtype=np.int16)
        dist[occupied] = 0
        for _ in range(120):
            nxt = dist.copy()
            nxt[1:, :] = np.minimum(nxt[1:, :], dist[:-1, :] + 1)
            nxt[:-1, :] = np.minimum(nxt[:-1, :], dist[1:, :] + 1)
            nxt[:, 1:] = np.minimum(nxt[:, 1:], dist[:, :-1] + 1)
            nxt[:, :-1] = np.minimum(nxt[:, :-1], dist[:, 1:] + 1)
            nxt[1:, 1:] = np.minimum(nxt[1:, 1:], dist[:-1, :-1] + 1)
            nxt[1:, :-1] = np.minimum(nxt[1:, :-1], dist[:-1, 1:] + 1)
            nxt[:-1, 1:] = np.minimum(nxt[:-1, 1:], dist[1:, :-1] + 1)
            nxt[:-1, :-1] = np.minimum(nxt[:-1, :-1], dist[1:, 1:] + 1)
            if np.array_equal(nxt, dist):
                break
            dist = nxt
        self._clearance_m = dist.astype(np.float32) * np.float32(self.resolution)
        return self._clearance_m

    def segment_free(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """True when the straight line stays on free cells. Endpoints are included."""
        row, col = self.world_to_cell(x0, y0)
        row1, col1 = self.world_to_cell(x1, y1)
        drow = abs(row1 - row)
        dcol = abs(col1 - col)
        step_row = 1 if row < row1 else -1
        step_col = 1 if col < col1 else -1
        err = drow - dcol
        while True:
            if self.grid[row, col] != 0:
                return False
            if row == row1 and col == col1:
                return True
            doubled = 2 * err
            if doubled > -dcol:
                err -= dcol
                row += step_row
            if doubled < drow:
                err += drow
                col += step_col

    def visible_lookahead(
        self,
        path: list[tuple[float, float]],
        index: int,
        x: float,
        y: float,
        distance: float,
    ) -> tuple[float, float] | None:
        """Lookahead along the path, or None when every chord crosses an obstacle.

        A pose already inside the inflated margin keeps the geometric lookahead:
        every chord starts on a blocked cell, and the planned point is what
        leads back out.
        """
        target = path_lookahead(path, index, x, y, distance)
        if target is None:
            return None
        row, col = self.world_to_cell(x, y)
        if self.grid[row, col] != 0 or self.segment_free(x, y, target[0], target[1]):
            return target
        index = max(0, min(index, len(path) - 1))
        chosen = None
        traveled = 0.0
        prev_x, prev_y = x, y
        horizon = max(distance * 2.0, distance + 1.0)
        for point in path[index:]:
            traveled += math.hypot(point[0] - prev_x, point[1] - prev_y)
            prev_x, prev_y = point
            if self.segment_free(x, y, point[0], point[1]):
                chosen = point
            if traveled >= distance and chosen is not None:
                break
            if traveled >= horizon:
                break
        return chosen


def load_occupancy(yaml_path: str | Path, inflation_m: float = 0.35) -> OccupancyMap:
    yaml_path = Path(yaml_path)
    with yaml_path.open() as handle:
        meta = yaml.safe_load(handle)
    image_path = yaml_path.parent / meta["image"]
    resolution = float(meta["resolution"])
    origin_x = float(meta["origin"][0])
    origin_y = float(meta["origin"][1])
    raw = np.flipud(np.array(Image.open(image_path), dtype=np.uint8))
    walls = raw <= 50
    inflation_cells = max(1, int(inflation_m / resolution))
    grid = inflate_occupancy(walls, inflation_cells)
    unknown = (raw > 50) & (raw < 220)
    grid[unknown] = 1
    display = np.full(raw.shape, -1, dtype=np.int8)
    display[raw >= 220] = 0
    display[raw <= 50] = 100
    # Soft cost out to 0.45 m so a gap between a shelf and a wall is crossed
    # in the middle. The lethal inflation above stays the hard limit.
    penalty = clearance_penalty(grid, resolution, influence_m=0.45, gain=6.0)
    return OccupancyMap(grid, resolution, origin_x, origin_y, display, penalty)


def clearance_penalty(
    grid: np.ndarray,
    resolution: float,
    influence_m: float = 0.45,
    gain: float = 6.0,
) -> np.ndarray:
    """Extra traversal cost for free cells, highest against an obstacle."""
    limit = max(1, int(round(influence_m / resolution)))
    dist = np.full(grid.shape, limit, dtype=np.int16)
    dist[grid != 0] = 0
    for _ in range(limit):
        nxt = dist.copy()
        nxt[1:, :] = np.minimum(nxt[1:, :], dist[:-1, :] + 1)
        nxt[:-1, :] = np.minimum(nxt[:-1, :], dist[1:, :] + 1)
        nxt[:, 1:] = np.minimum(nxt[:, 1:], dist[:, :-1] + 1)
        nxt[:, :-1] = np.minimum(nxt[:, :-1], dist[:, 1:] + 1)
        dist = nxt
    penalty = np.zeros(grid.shape, dtype=np.float32)
    free_near = (grid == 0) & (dist < limit)
    penalty[free_near] = gain * (limit - dist[free_near]) / limit
    return penalty
