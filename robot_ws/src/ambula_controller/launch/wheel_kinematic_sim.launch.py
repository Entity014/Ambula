import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = False

    wheel_kinematic_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_controller"), "config", "wheel_kinematic_sim.yaml"]
    )

    ld = LaunchDescription()
    node_wheel_cmd = Node(
        package="ambula_controller",
        executable="wheel_cmd.py",
        name="wheel_cmd_vel",
        parameters=[wheel_kinematic_config_path],
    )

    ld.add_action(node_wheel_cmd)
    return ld