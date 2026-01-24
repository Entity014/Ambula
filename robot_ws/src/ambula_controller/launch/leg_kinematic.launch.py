import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = False

    leg_kinematic_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_controller"), "config", "leg_kinematic.yaml"]
    )

    ld = LaunchDescription()
    node_joint_sim2real = Node(
        package="ambula_controller",
        executable="joint_sim2real.py",
        parameters=[{leg_kinematic_config_path}]
    )

    ld.add_action(node_joint_sim2real)
    return ld