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
        [FindPackageShare("ambula_controller"), "config", "leg_kinematic_sim.yaml"]
    )

    ld = LaunchDescription()
    node_joint_sim2real = Node(
        package="ambula_controller",
        executable="joint_sim2real.py",
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
    node_ik_left = Node(
        package="ambula_controller",
        executable="inverse_kinematic.py",
        name="left_leg_inverse_kinematics",
        parameters=[leg_kinematic_config_path],
    )
    node_ik_right = Node(
        package="ambula_controller",
        executable="inverse_kinematic.py",
        name="right_leg_inverse_kinematics",
        parameters=[leg_kinematic_config_path],
    )
    node_left_joint_smoother = Node(
        package="ambula_controller",
        executable="joint_smoother.py",
        name="left_leg_joint_smoother",
        parameters=[leg_kinematic_config_path],
    )
    node_right_joint_smoother = Node(
        package="ambula_controller",
        executable="joint_smoother.py",
        name="right_leg_joint_smoother",
        parameters=[leg_kinematic_config_path],
    )
    node_left_leg_joint_cmd_bridge = Node(
        package="ambula_controller",
        executable="leg_joint_cmd.py",
        name="left_leg_joint_cmd_bridge",
        parameters=[leg_kinematic_config_path],
    )
    node_right_leg_joint_cmd_bridge = Node(
        package="ambula_controller",
        executable="leg_joint_cmd.py",
        name="right_leg_joint_cmd_bridge",
        parameters=[leg_kinematic_config_path],
    )

    ld.add_action(node_joint_sim2real)
    ld.add_action(node_fk_left)
    ld.add_action(node_fk_right)
    ld.add_action(node_ik_left)
    ld.add_action(node_ik_right)
    ld.add_action(node_left_joint_smoother)
    ld.add_action(node_right_joint_smoother)
    ld.add_action(node_left_leg_joint_cmd_bridge)
    ld.add_action(node_right_leg_joint_cmd_bridge)
    return ld