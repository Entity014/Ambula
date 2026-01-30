from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --------- launch args ----------
    use_madgwick = LaunchConfiguration("use_madgwick")
    use_complementary = LaunchConfiguration("use_complementary")

    imu_raw_topic = LaunchConfiguration("imu_raw_topic")
    madgwick_out_topic = LaunchConfiguration("madgwick_out_topic")
    complementary_out_topic = LaunchConfiguration("complementary_out_topic")

    world_frame = LaunchConfiguration("world_frame") # e.g. odom (หรือ map)

    imu_sensor_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_state_estimator"), "config", "imu_filters_sim.yaml"]
    )

    # --------- nodes ----------
    # 1) Madgwick filter
    madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        condition=IfCondition(use_madgwick),
        parameters=[{
            # topic/frame
            "use_mag": False,          # ถ้ามี mag ค่อยเปิด
            "publish_tf": False,       # ถ้าใช้ EKF/robot_localization แนะนำปิด
            "world_frame": world_frame,  # บางเวอร์ชันใช้ fixed_frame/world_frame
            "fixed_frame": world_frame,
            # tuning
            "gain": 1.0,               # ค่าเริ่มต้นที่นิยมลอง (ต้องจูนตามจริง)
        }],
        remappings=[
            ("imu/data_raw", imu_raw_topic),
            ("imu/data", madgwick_out_topic),
        ],
    )

    # 2) Complementary filter
    complementary_node = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        name="imu_complementary_filter",
        output="screen",
        condition=IfCondition(use_complementary),
        parameters=[{
            "use_mag": False,
            "publish_tf": False,
            "fixed_frame": world_frame,
            # options (บางเวอร์ชันมี)
            "do_bias_estimation": True,
            "do_adaptive_gain": True,
            "gain_acc": 0.01,
        }],
        remappings=[
            ("imu/data_raw", imu_raw_topic),
            ("imu/data", complementary_out_topic),
        ],
    )

    node_orientation_to_euler_raw = Node(
        package="ambula_state_estimator",
        executable="imu_orientation_to_euler.py",
        name="imu_orientation_to_euler_raw",
        parameters=[imu_sensor_config_path],
    )

    node_orientation_to_euler_complementary = Node(
        package="ambula_state_estimator",
        executable="imu_orientation_to_euler.py",
        name="imu_orientation_to_euler_complementary",
        parameters=[imu_sensor_config_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_madgwick", default_value="false"),
        DeclareLaunchArgument("use_complementary", default_value="false"),

        DeclareLaunchArgument("imu_raw_topic", default_value="/imu/data_raw"),
        DeclareLaunchArgument("madgwick_out_topic", default_value="/imu/madgwick"),
        DeclareLaunchArgument("complementary_out_topic", default_value="/imu/complementary"),

        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("imu_frame", default_value="imu_link"),
        DeclareLaunchArgument("world_frame", default_value="odom"),

        madgwick_node,
        complementary_node,
        node_orientation_to_euler_raw,
        # node_orientation_to_euler_complementary,
    ])
