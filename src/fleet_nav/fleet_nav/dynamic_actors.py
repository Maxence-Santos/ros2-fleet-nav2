#!/usr/bin/env python3
"""Publish the walker speeds on /model/person_N/cmd_vel."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from fleet_nav.walkers import PEOPLE, linear_speed


class DynamicActorsNode(Node):
    def __init__(self):
        super().__init__(
            "dynamic_actors",
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True,
            )],
        )
        self._pubs = [
            self.create_publisher(Twist, f"/model/{person.name}/cmd_vel", 10)
            for person in PEOPLE
        ]
        self._t = 0.0
        self.create_timer(0.05, self._update)
        names = ", ".join(person.name for person in PEOPLE)
        self.get_logger().info(f"Walking {names}")

    def _update(self):
        self._t += 0.05
        for person, pub in zip(PEOPLE, self._pubs):
            command = Twist()
            command.linear.x = linear_speed(person, self._t)
            pub.publish(command)


def main():
    rclpy.init()
    node = DynamicActorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
