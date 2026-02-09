import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = False

    balance_control_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_controller"), "config", "balance_control.yaml"]
    )

    ld = LaunchDescription()
    node_position_control = Node(
        package="ambula_controller",
        executable="position_control.py",
        name="go_to_goal_controller",
        parameters=[{balance_control_config_path}]
    )

    ld.add_action(node_position_control)
    return ld