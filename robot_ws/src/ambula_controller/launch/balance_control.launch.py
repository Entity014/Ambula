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
    node_wbc_node = Node(
        package="ambula_controller",
        executable="wbc_node.py",
        name="wbc_node",
        parameters=[{balance_control_config_path}]
    )

    ld.add_action(node_wbc_node)
    return ld