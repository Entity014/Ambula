import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = False

    rl_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_controller"), "config", "rl.yaml"]
    )

    ld = LaunchDescription()
    node_reinforcement = Node(
        package="ambula_controller",
        executable="balance_control.py",
        name="balance_node",
        parameters=[{rl_config_path}]
    )

    ld.add_action(node_reinforcement)
    return ld