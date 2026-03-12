from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = False

    state_estimator_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_state_estimator"), "config", "robot_estimator.yaml"]
    )

    ld = LaunchDescription()
    node_ekf = Node(
        package="ambula_state_estimator",
        executable="state_estimator.py",
        name="ambula_state_estimator",
        parameters=[state_estimator_config_path],
    )

    ld.add_action(node_ekf)
    return ld