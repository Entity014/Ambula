import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import launch

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory("ambula_description"))
    ambula_desc_path = os.path.join(get_package_share_directory("ambula_description"))

    bridge_path = os.path.join(pkg_path, "config", "gazebo_bridge.yaml")

    xacro_file = os.path.join(ambula_desc_path, "urdf/ambula_oa/", "ambula.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {
        "robot_description": robot_description_config.toxml(),
        "use_sim_time": use_sim_time,
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Spawn
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "ambula",
            # '-x', '0',
            # "-z",
            # "1.0",
            # '-y', '0',
            "-P",
            "0.0",
            "-topic",
            "/robot_description",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="controller_bridge",
        parameters=[
            {
                "config_file": bridge_path,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_slider_position_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "slider_position_controller",
        ],
        output="screen",
    )

    launch.actions.DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Flag to enable use_sim_time",
    )

    # Launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use sim time if true"
            ),
            robot_state_publisher,
            bridge,
            spawn,
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=spawn,
            #         on_exit=[
            #             load_joint_state_controller,
            #         ],
            #     )
            # ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=load_joint_state_controller,
            #         on_exit=[
            #             load_slider_position_controller,
            #         ],
            #     )
            # ),
        ]
    )
