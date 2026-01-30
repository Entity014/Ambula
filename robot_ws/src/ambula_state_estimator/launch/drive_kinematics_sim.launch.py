from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = False

    drive_kinematic_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_state_estimator"), "config", "drive_kinematics_sim.yaml"]
    )

    ld = LaunchDescription()
    node_drive_kinematics = Node(
        package="ambula_state_estimator",
        executable="diff_drive_kinematics.py",
        name="diff_drive_kinematics",
        parameters=[drive_kinematic_config_path],
    )
    node_height_estimator = Node(
        package="ambula_state_estimator",
        executable="height_estimator.py",
        name="height_estimator",
        parameters=[drive_kinematic_config_path],
    )

    ld.add_action(node_drive_kinematics)
    ld.add_action(node_height_estimator)
    return ld