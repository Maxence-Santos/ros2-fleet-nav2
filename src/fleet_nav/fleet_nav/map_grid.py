"""Load the shared warehouse occupancy grid. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from fleet_nav.navigation_geometry import (
    astar_grid,
    inflate_occupancy,
    nearest_free_cell,
    path_lookahead,
    simplify_path,
)


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
    ) -> float:
        """Nearest known occupied cell along ``yaw``, or ``max_range``.

        With ``samples`` above 1, the ray is repeated across ±``half_angle``.
        Unknown cells and the edge of the grid are not walls: a grey cell was
        stopping the ray and every lidar return then looked like the map.
        """
        nearest = max_range
        if samples <= 1:
            return self._ray_to_mapped(x, y, yaw, max_range)
        for index in range(samples):
            angle = yaw - half_angle + (2.0 * half_angle) * index / (samples - 1)
            nearest = min(nearest, self._ray_to_mapped(x, y, angle, max_range))
        return nearest

    def _ray_to_mapped(self, x: float, y: float, angle: float, max_range: float) -> float:
        step = max(self.resolution, 0.02)
        height, width = self.display.shape
        travelled = 0.0
        while travelled < max_range:
            travelled += step
            px = x + math.cos(angle) * travelled
            py = y + math.sin(angle) * travelled
            col = int((px - self.origin_x) / self.resolution)
            row = int((py - self.origin_y) / self.resolution)
            if row < 0 or col < 0 or row >= height or col >= width:
                # Past the saved map is not a wall. Stopping here made every
                # return look explained and the robot never passed anyone.
                return max_range
            if int(self.display[row, col]) == 100:
                return travelled
        return max_range

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
