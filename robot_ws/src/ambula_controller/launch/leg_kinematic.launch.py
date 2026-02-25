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
    node_height_adjust = Node(
        package="ambula_controller",
        executable="height_adjust",
        name="height_adjust",
        parameters=[{leg_kinematic_config_path}]
    )
    node_left_leg_inverse_kinematic = Node(
        package="ambula_controller",
        executable="inverse_kinematic",
        name="left_leg_inverse_kinematic",
        parameters=[{leg_kinematic_config_path}]
    )
    node_right_leg_inverse_kinematic = Node(
        package="ambula_controller",
        executable="inverse_kinematic",
        name="right_leg_inverse_kinematic",
        parameters=[{leg_kinematic_config_path}]
    )
    node_left_leg_joint_smoother = Node(
        package="ambula_controller",
        executable="joint_smoother_cpp",
        name="left_leg_joint_smoother",
        parameters=[{leg_kinematic_config_path}]
    )
    node_right_leg_joint_smoother = Node(
        package="ambula_controller",
        executable="joint_smoother_cpp",
        name="right_leg_joint_smoother",
        parameters=[{leg_kinematic_config_path}]
    )
    node_convert_to_left_motor = Node(
        package="ambula_controller",
        executable="convert_to_motor",
        name="convert_to_left_motor",
        parameters=[{leg_kinematic_config_path}]
    )
    node_convert_to_right_motor = Node(
        package="ambula_controller",
        executable="convert_to_motor",
        name="convert_to_right_motor",
        parameters=[{leg_kinematic_config_path}]
    )

    ld.add_action(node_height_adjust)
    ld.add_action(node_left_leg_inverse_kinematic)
    ld.add_action(node_right_leg_inverse_kinematic)
    # ld.add_action(node_left_leg_joint_smoother)
    # ld.add_action(node_right_leg_joint_smoother)
    # ld.add_action(node_convert_to_left_motor)
    # ld.add_action(node_convert_to_right_motor)
    return ld