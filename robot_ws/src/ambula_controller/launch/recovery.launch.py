import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = False

    recovery_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_controller"), "config", "recovery.yaml"]
    )

    ld = LaunchDescription()
    node_stand_feedforward = Node(
        package="ambula_controller",
        executable="stand_feedforward.py",
        name="stand_feedforward",
        parameters=[{recovery_config_path}]
    )

    ld.add_action(node_stand_feedforward)
    return ld