from launch_ros.substitutions import FindPackageShare
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

import sys
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory("ambula_bringup")
    gz_worlds_path = os.path.join(get_package_share_directory("ambula_world"))
    world_path = os.path.join(gz_worlds_path, "worlds", "plane.sdf")
    gui = "-r"
    for arg in sys.argv:
        if arg.startswith("gui:="):
            gui = str(arg.split(":=")[1])
    
    controller_pkg_path = get_package_share_directory("ambula_controller")
    state_estimator_pkg_path = get_package_share_directory("ambula_state_estimator")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={"gz_args": f"{gui} {world_path}"}.items(),
    )

    spawn_ambula_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_path, "launch", "summon_ambula.launch.py"])]
        )
    )

    leg_kinematic_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([controller_pkg_path, "launch", "leg_kinematic_sim.launch.py"])]
        )
    )

    wheel_kinematic_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([controller_pkg_path, "launch", "wheel_kinematic_sim.launch.py"])]
        )
    )
    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([state_estimator_pkg_path, "launch", "state_estimation_sim.launch.py"])]
        )
    )

    return LaunchDescription(
        [
            gazebo_launch,
            spawn_ambula_launch,
            leg_kinematic_sim_launch,
            wheel_kinematic_sim_launch,
            state_estimation_launch,
        ]
    )
