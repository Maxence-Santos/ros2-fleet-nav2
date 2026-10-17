"""Back-and-forth walkers, copied from ros2-slam-nav2 dynamic_actors.

Gazebo VelocityControl applies linear.x in the body frame. The sign flips
with the same square wave as the first project, so each person returns to
the spawn. The routes stay in the open aisles: clear of the shelves at
(2, 2) and (-2, -2), of the walls, and of the three robot spawns.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Walker:
    name: str
    x: float
    y: float
    yaw: float
    period: float
    rgba: str
    phase: str = "sin"
    speed: float = 0.20


PEOPLE = (
    Walker("person_1", -1.5, 1.2, 0.0, 20.0, "0.9 0.1 0.1 1", "sin"),
    Walker("person_2", -3.6, -1.5, math.pi / 2, 24.0, "0.1 0.2 0.9 1", "sin"),
    Walker("person_3", 1.5, -2.0, math.pi / 2, 20.0, "0.1 0.9 0.2 1", "cos"),
)


def linear_speed(walker: Walker, t: float) -> float:
    """Body-frame forward speed. Zero of the wave counts as the reverse half."""
    angle = 2.0 * math.pi * t / walker.period
    wave = math.sin(angle) if walker.phase == "sin" else math.cos(angle)
    return walker.speed if wave > 0.0 else -walker.speed


def pose_at(walker: Walker, t: float, dt: float = 0.05) -> tuple[float, float]:
    """Pose after ``t`` seconds, stepping the same 20 Hz clock as the node."""
    x, y = walker.x, walker.y
    if t <= 0.0:
        return x, y
    forward_x = math.cos(walker.yaw)
    forward_y = math.sin(walker.yaw)
    steps = int(round(t / dt))
    for index in range(steps):
        speed = linear_speed(walker, (index + 1) * dt)
        x += speed * forward_x * dt
        y += speed * forward_y * dt
    return x, y
