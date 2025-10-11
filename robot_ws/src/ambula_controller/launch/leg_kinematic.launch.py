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
    node_leg_fk = Node(
        package="ambula_controller",
        executable="leg_fk.py",
        parameters=[{leg_kinematic_config_path}]
    )
    node_leg_ik = Node(
        package="ambula_controller",
        executable="leg_ik.py",
        parameters=[{leg_kinematic_config_path}]
    )
    node_joint_merge = Node(
        package="ambula_controller",
        executable="joint_state_command_merge.py",
        parameters=[{leg_kinematic_config_path}]
    )
    node_height_to_pose = Node(
        package="ambula_controller",
        executable="height_to_point.py",
        parameters=[{leg_kinematic_config_path}]
    )
    node_s_curve_command = Node(
        package="ambula_controller",
        executable="s_curve_command.py",
        parameters=[{leg_kinematic_config_path}]
    )
    
    ld.add_action(node_leg_fk)
    ld.add_action(node_leg_ik)
    ld.add_action(node_joint_merge)
    ld.add_action(node_height_to_pose)
    ld.add_action(node_s_curve_command)

    return ld