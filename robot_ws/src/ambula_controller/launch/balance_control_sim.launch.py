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
        [FindPackageShare("ambula_controller"), "config", "balance_control_sim.yaml"]
    )

    ld = LaunchDescription()
    node_pd_balance = Node(
        package="ambula_controller",
        executable="pd_balance.py",
        parameters=[{balance_control_config_path}]
    )

    ld.add_action(node_pd_balance)
    return ld