import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = False

    state_machine_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_state_machine"), "config", "state_machine.yaml"]
    )

    ld = LaunchDescription()
    node_autonomy = Node(
        package="ambula_state_machine",
        executable="autonomy_node.py",
        name="autonomy_node",
        parameters=[{state_machine_config_path}]
    )

    ld.add_action(node_autonomy)
    return ld