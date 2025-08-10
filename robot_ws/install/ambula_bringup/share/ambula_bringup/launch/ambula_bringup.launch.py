import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
        ),
        launch_arguments=[("gz_args", ["-r -v 4 empty.sdf"])],
    )

    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_cart_pole",
        arguments=[
            "-name",
            "ambula_bot",
            "-topic",
            "robot_description",
            "-z",
            "0.45",
            # "-P",
            # "-0.1",
        ],
        output="screen",
    )

    xacro_file = os.path.join(
        get_package_share_directory("ambula_description"),
        "urdf/ambula_bot",
        "ambula.urdf.xacro",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": Command(["xacro ", xacro_file])},
            {"use_sim_time": True},
        ],
    )

    bridge_config = os.path.join(
        get_package_share_directory("ambula_description"),
        "config",
        "gazebo_bridge.yaml",
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        ros_arguments=["-p", f"config_file:={bridge_config}"],
        output="screen",
    )

    simulation_control = Node(
        package="simulation_control",
        executable="simulation_control_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    simulation_observation = Node(
        package="ambula_observation",
        executable="robot_observation_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
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

    load_wheel_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_velocity_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            gazebo_spawn_robot,
            robot_state_publisher,
            gz_bridge,
            simulation_control,
            simulation_observation,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gazebo_spawn_robot,
                    on_exit=[
                        load_joint_state_controller,
                    ],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[
                        load_wheel_velocity_controller,
                    ],
                )
            ),
        ]
    )
