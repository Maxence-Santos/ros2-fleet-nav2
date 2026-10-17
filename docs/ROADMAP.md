# Roadmap

## v1 — in place

- [x] Three AMRs, namespaces `/robot_1` `/robot_2` `/robot_3`, separate TF, global `/tf`.
- [x] One shared warehouse map, a single `/map` publisher.
- [x] A* per robot. The path stays 0.25 m from obstacles. Tracking sticks to the line. A pivot happens only when one side is a wall (under 0.50 m) and the other leaves 0.80 m.
- [x] `fleet_manager`: mission to the nearest idle robot.
- [x] Recovery: timeout without progress, exclusion, cooldown. A dropped goal is not handed to another robot. The library can still requeue when `requeue` is true; the demo never sets that.
- [x] Unit tests for assignment and for a path on the real map.
- [x] One-robot mapping (`start_slam_demo.sh`) and `save_map.py`, as in project 1.
- [x] The demo loads `src/fleet_nav/maps/warehouse_map`.
- [x] RViz click: select the robot, then the goal (`/fleet/direct`).
- [x] Lidar avoidance of robots and people: 0.75 m parallel, each robot's own right head-on, stop when a moving robot cuts the trajectory from the right.
- [x] Three people in the aisles, absent from the mapping world.

## Outside this repository

- [ ] Multi-instance Nav2 instead of the custom A*.
- [ ] Open-RMF.
- [ ] Distributed SLAM, one map per robot.
- [x] GitHub publication: [Maxence-Santos/ros2-fleet-nav2](https://github.com/Maxence-Santos/ros2-fleet-nav2).
