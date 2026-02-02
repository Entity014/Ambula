import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


import sys
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = False

    bringup_pkg_path = get_package_share_directory("ambula_bringup")
    controller_pkg_path = get_package_share_directory("ambula_controller")
    state_estimator_pkg_path = get_package_share_directory("ambula_state_estimator")
    state_machine_pkg_path = get_package_share_directory("ambula_state_machine")

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([bringup_pkg_path, "launch", "description.launch.py"])]
        ),
    )

    leg_kinematic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([controller_pkg_path, "launch", "leg_kinematic.launch.py"])]
        )
    )

    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([state_estimator_pkg_path, "launch", "state_estimation.launch.py"])]
        )
    )

    state_machine_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([state_machine_pkg_path, "launch", "state_machine.launch.py"])]
        )
    )
    

    ld = LaunchDescription()
    ld.add_action(description_launch)
    ld.add_action(leg_kinematic_launch)
    ld.add_action(state_estimation_launch)
    # ld.add_action(state_machine_launch)
    return ld