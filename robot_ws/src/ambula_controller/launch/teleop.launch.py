import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = False

    teleop_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_controller"), "config", "teleop.yaml"]
    )

    ld = LaunchDescription()
    node_joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,   # ทำให้ cmd_vel ออกต่อเนื่องตอนค้างคันโยก
        }],
        output="screen",
    )
    node_teleop_twist = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[{teleop_config_path}],
        output="screen",
    )

    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist)
    return ld