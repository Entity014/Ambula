from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = False

    state_vector_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_state_estimator"), "config", "state_vector_sim.yaml"]
    )

    ld = LaunchDescription()
    node_state_vector = Node(
        package="ambula_state_estimator",
        executable="state_vector.py",
        name="state_vector",
        parameters=[state_vector_config_path],
    )

    ld.add_action(node_state_vector)
    return ld