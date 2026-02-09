from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import sys
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = False

    state_estimator_pkg_path = get_package_share_directory("ambula_state_estimator")

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([state_estimator_pkg_path, "launch", "localization.launch.py"])]
        )
    )


    ld = LaunchDescription()
    ld.add_action(localization_launch)
    return ld