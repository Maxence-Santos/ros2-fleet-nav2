"""Three AMRs in the shared warehouse. Headless unless headless:=false."""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge

from fleet_nav.walkers import PEOPLE


def generate_launch_description():
    share = get_package_share_directory("fleet_nav")
    fleet_yaml = os.path.join(share, "config", "fleet.yaml")
    world = os.path.join(share, "worlds", "warehouse.sdf")
    xacro_file = os.path.join(share, "urdf", "robot.urdf.xacro")
    person_xacro = os.path.join(share, "urdf", "person.sdf.xacro")
    with open(fleet_yaml, encoding="utf-8") as handle:
        fleet = yaml.safe_load(handle)

    headless = LaunchConfiguration("headless")
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run gz sim -s. Pass false to open the Gazebo window.",
    )
    map_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value=os.path.join(share, "maps", "warehouse_map.yaml"),
        description="Saved occupancy map. The demo reads this file, as in ros2-slam-nav2.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Open RViz. Use Publish Point: click a robot, then click its goal.",
    )
    map_yaml = LaunchConfiguration("map_yaml")

    gazebo = [
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", world],
            condition=IfCondition(headless),
            output="screen",
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            condition=UnlessCondition(headless),
            output="screen",
        ),
    ]
    bridge = RosGzBridge(
        bridge_name="fleet_bridge",
        config_file=os.path.join(share, "config", "bridge.yaml"),
        use_composition=False,
        bridge_params=[{"use_sim_time": True}],
    )
    manager = Node(
        package="fleet_nav",
        executable="fleet_manager",
        name="fleet_manager",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "fleet_yaml": fleet_yaml,
            "map_yaml": map_yaml,
            "demo": False,
        }],
    )

    robot_actions = []
    for index, (name, spec) in enumerate(fleet["robots"].items()):
        description = xacro.process_file(
            xacro_file,
            mappings={"robot_name": name, "chassis_rgba": str(spec["rgba"])},
        ).toxml()
        robot_actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=name,
            output="screen",
            remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
            parameters=[{
                "use_sim_time": True,
                "robot_description": description,
                "frame_prefix": f"{name}/",
            }],
        ))
        robot_actions.append(Node(
            package="fleet_nav",
            executable="fleet_navigator",
            namespace=name,
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "spawn_x": float(spec["x"]),
                "spawn_y": float(spec["y"]),
                "map_yaml": map_yaml,
                "peer_names": [other for other in fleet["robots"] if other != name],
            }],
        ))
        robot_actions.append(TimerAction(
            period=3.0 + 4.0 * index,
            actions=[Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{name}",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "-world", "warehouse",
                    "-topic", f"/{name}/robot_description",
                    "-name", name,
                    "-x", str(spec["x"]),
                    "-y", str(spec["y"]),
                    "-z", "0.1",
                ],
            )],
        ))

    # People are spawned only for the fleet demo. Mapping keeps the static world,
    # so a later SLAM run does not bake a walker into the occupancy grid.
    people = [Node(
        package="fleet_nav",
        executable="dynamic_actors",
        name="dynamic_actors",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )]
    for person in PEOPLE:
        description = xacro.process_file(
            person_xacro,
            mappings={"body_rgba": person.rgba},
        ).toxml()
        people.append(TimerAction(
            period=2.0,
            actions=[Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{person.name}",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "-world", "warehouse",
                    "-string", description,
                    "-name", person.name,
                    "-x", str(person.x),
                    "-y", str(person.y),
                    "-z", "0.75",
                    "-Y", str(person.yaw),
                ],
            )],
        ))

    click = Node(
        package="fleet_nav",
        executable="click_goal",
        name="click_goal",
        output="screen",
        parameters=[{"use_sim_time": True, "fleet_yaml": fleet_yaml}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(share, "rviz", "fleet.rviz")],
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([
        headless_arg, map_arg, rviz_arg, *gazebo, bridge, manager, click, rviz,
        *people, *robot_actions,
    ])
