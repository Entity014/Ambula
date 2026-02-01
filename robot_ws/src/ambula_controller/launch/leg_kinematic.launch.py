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
    node_joint_converter = Node(
        package="ambula_controller",
        executable="joint_converter.py",
        name="joint_converter",
        parameters=[{leg_kinematic_config_path}]
    )
    node_fk_left = Node(
        package="ambula_controller",
        executable="forward_kinematic.py",
        name="left_leg_forward_kinematics",
        parameters=[leg_kinematic_config_path],
    )
    node_fk_right = Node(
        package="ambula_controller",
        executable="forward_kinematic.py",
        name="right_leg_forward_kinematics",
        parameters=[leg_kinematic_config_path],
    )

    ld.add_action(node_joint_converter)
    ld.add_action(node_fk_left)
    ld.add_action(node_fk_right)
    return ld