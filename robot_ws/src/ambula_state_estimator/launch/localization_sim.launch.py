from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = False

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_state_estimator"), "config", "ekf_sim.yaml"]
    )

    ld = LaunchDescription()
    node_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path],
        remappings=[("odometry/filtered", "odom")],
    )

    ld.add_action(node_ekf)
    return ld