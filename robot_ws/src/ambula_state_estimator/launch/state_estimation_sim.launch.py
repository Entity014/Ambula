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

    drive_kinematic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([state_estimator_pkg_path, "launch", "drive_kinematics_sim.launch.py"])]
        )
    )

    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([state_estimator_pkg_path, "launch", "imu_filters_sim.launch.py"])]
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([state_estimator_pkg_path, "launch", "localization_sim.launch.py"])]
        )
    )

    state_vector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([state_estimator_pkg_path, "launch", "state_vector_sim.launch.py"])]
        )
    )

    ld = LaunchDescription()
    ld.add_action(drive_kinematic_launch)
    ld.add_action(imu_filter_launch)
    # ld.add_action(localization_launch)
    ld.add_action(state_vector_launch)
    return ld