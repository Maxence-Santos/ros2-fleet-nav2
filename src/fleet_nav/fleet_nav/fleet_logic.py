"""Assign warehouse goals to the nearest idle robot, and requeue stalls."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

IDLE = "idle"
NAVIGATING = "navigating"
SUCCEEDED = "succeeded"
BLOCKED = "blocked"


@dataclass
class Assignment:
    robot: str
    mission_id: int
    x: float
    y: float


@dataclass
class Mission:
    mission_id: int
    x: float
    y: float
    excluded: set[str] = field(default_factory=set)
    requeue: bool = True


@dataclass
class RobotState:
    name: str
    x: float = 0.0
    y: float = 0.0
    status: str = IDLE
    mission_id: int | None = None
    goal_x: float | None = None
    goal_y: float | None = None
    assigned_at: float = 0.0
    best_distance: float = math.inf
    cooldown_until: float = 0.0
    pose_known: bool = False
    seen_navigating: bool = False


class FleetBook:
    """In-memory fleet. ``tick`` is the only place a mission changes hands."""

    def __init__(
        self,
        names: list[str],
        timeout_s: float = 20.0,
        progress_m: float = 0.3,
        cooldown_s: float = 8.0,
    ):
        self.robots = {name: RobotState(name=name) for name in names}
        self.timeout_s = timeout_s
        self.progress_m = progress_m
        self.cooldown_s = cooldown_s
        self.queue: list[Mission] = []
        self.missions: dict[int, Mission] = {}
        self._cancels: list[str] = []
        self._seq = 1

    def set_pose(self, name: str, x: float, y: float, now: float) -> None:
        robot = self.robots[name]
        robot.x = x
        robot.y = y
        robot.pose_known = True
        if robot.status != NAVIGATING or robot.goal_x is None or robot.goal_y is None:
            return
        distance = math.hypot(robot.goal_x - x, robot.goal_y - y)
        # Anchor stays put until the robot has closed progress_m in total.
        # Updating it on every odom sample would hide a stall.
        if robot.best_distance - distance >= self.progress_m:
            robot.best_distance = distance
            robot.assigned_at = now

    def note_status(self, name: str, status: str, now: float) -> None:
        robot = self.robots[name]
        if robot.mission_id is None:
            return
        if status == NAVIGATING:
            robot.seen_navigating = True
            return
        # A block published for the previous goal must not cancel the mission
        # that was just given. The navigator announces navigating first.
        if not robot.seen_navigating:
            return
        if status == SUCCEEDED:
            self._clear_goal(robot)
            robot.status = IDLE
        elif status == BLOCKED:
            self._release(robot, now)

    def submit(self, x: float, y: float, requeue: bool = True) -> int:
        mission_id = self._seq
        self._seq += 1
        mission = Mission(mission_id, x, y, requeue=requeue)
        self.missions[mission_id] = mission
        self.queue.append(mission)
        return mission_id

    def tick(self, now: float) -> list[Assignment]:
        for robot in self.robots.values():
            if robot.status == NAVIGATING and now - robot.assigned_at >= self.timeout_s:
                self._release(robot, now)
        for robot in self.robots.values():
            if robot.status == BLOCKED and now >= robot.cooldown_until:
                robot.status = IDLE

        assigned: list[Assignment] = []
        waiting: list[Mission] = []
        names = set(self.robots)
        for mission in self.queue:
            if mission.excluded >= names:
                mission.excluded.clear()
            choice = self._nearest_idle(mission)
            if choice is None:
                waiting.append(mission)
                continue
            self._give(choice, mission, now)
            assigned.append(Assignment(choice.name, mission.mission_id, mission.x, mission.y))
        self.queue = waiting
        return assigned

    def take_cancels(self) -> list[str]:
        names = self._cancels
        self._cancels = []
        return names

    def command(self, name: str, x: float, y: float, now: float) -> Assignment:
        """Send one robot to a point, replacing the mission it already has."""
        robot = self.robots[name]
        if robot.mission_id is not None:
            mission = self.missions.pop(robot.mission_id, None)
            if mission is not None and mission in self.queue:
                self.queue.remove(mission)
            self._clear_goal(robot)
            self._cancels.append(robot.name)
        robot.status = IDLE
        robot.cooldown_until = 0.0
        mission_id = self._seq
        self._seq += 1
        mission = Mission(mission_id, x, y, requeue=False)
        self.missions[mission_id] = mission
        self._give(robot, mission, now)
        return Assignment(robot.name, mission_id, x, y)

    def _nearest_idle(self, mission: Mission) -> RobotState | None:
        best: RobotState | None = None
        best_distance = math.inf
        for robot in self.robots.values():
            if robot.status != IDLE or robot.name in mission.excluded:
                continue
            distance = math.hypot(robot.x - mission.x, robot.y - mission.y)
            if distance < best_distance:
                best = robot
                best_distance = distance
        return best

    def _give(self, robot: RobotState, mission: Mission, now: float) -> None:
        robot.status = NAVIGATING
        robot.mission_id = mission.mission_id
        robot.goal_x = mission.x
        robot.goal_y = mission.y
        robot.assigned_at = now
        robot.best_distance = math.hypot(robot.x - mission.x, robot.y - mission.y)
        robot.seen_navigating = False

    def _release(self, robot: RobotState, now: float) -> None:
        mission = self.missions.get(robot.mission_id) if robot.mission_id is not None else None
        self._clear_goal(robot)
        robot.status = BLOCKED
        robot.cooldown_until = now + self.cooldown_s
        self._cancels.append(robot.name)
        if mission is None or not mission.requeue:
            return
        mission.excluded.add(robot.name)
        if mission not in self.queue:
            self.queue.append(mission)

    def _clear_goal(self, robot: RobotState) -> None:
        robot.mission_id = None
        robot.goal_x = None
        robot.goal_y = None


def click_action(
    poses: dict[str, tuple[float, float]],
    x: float,
    y: float,
    selected: str | None,
    radius: float,
) -> tuple[str | None, tuple[float, float] | None]:
    """A click on a robot selects it. The next click elsewhere is that robot's goal."""
    nearest: str | None = None
    nearest_distance = radius
    for name, (robot_x, robot_y) in poses.items():
        distance = math.hypot(robot_x - x, robot_y - y)
        if distance <= nearest_distance:
            nearest = name
            nearest_distance = distance
    if nearest is not None:
        return nearest, None
    if selected is None:
        return None, None
    return selected, (x, y)
