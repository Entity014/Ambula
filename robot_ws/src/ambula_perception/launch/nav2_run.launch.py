import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory('ambula_perception')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_local_params.yaml')
    realsense_dir = get_package_share_directory('realsense2_camera')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    detection_config_path = PathJoinSubstitution(
        [FindPackageShare("ambula_perception"), "config", "detection.yaml"]
    )

    # 1. RealSense
    included_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': 'camera',
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'false',
            'enable_color': 'true',        
            'align_depth.enable': 'false', 
            'enable_infra1': 'false',       
            'enable_infra2': 'false',     
            'pointcloud.enable': 'true',   
            'ordered_pc': 'false',
            'depth_module.depth_profile': '640x480x15',    # ลด fps 30→15 ช่วย CPU มาก
            'rgb_camera.color_profile': '640x480x15',
            'infra_module.profile': '640x480x15',
            'decimation_filter.enable': 'true',
            'decimation_filter.filter_magnitude': '4',  # ลด 4x, ใส่ 4 ลดได้ 16x
            'publish_tf': 'true',
        }.items()
    )

    # 2. IMU Filter
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/camera/camera/imu/filtered')
        ]
    )

    # 3. Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )

    # 4. RTAB-Map
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'subscribe_imu': True,
            'wait_imu_to_init': True,
            'approx_sync': True,
            'wait_for_transform': 0.5,
        }],
        remappings=[
            ('rgb/image', '/camera/camera/infra1/image_rect_raw'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
            ('rgb/camera_info', '/camera/camera/infra1/camera_info'),
            ('imu', '/camera/camera/imu/filtered'),
        ]
    )

    nav2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    depth2laser_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            'scan_height': 10,
            'output_frame': 'camera_link',
            'range_min': 0.3,
            'range_max': 3.0
        }]
    )

    # 5. หน่วงเวลา 5 วิ รอ realsense node ขึ้นก่อน แล้วค่อย set param
    #    เพราะ launch_arguments ไม่สามารถส่งชื่อที่มี __ ได้ใน ROS2
    set_pointcloud_enable = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/camera/camera',
             'pointcloud__neon_.enable', 'true'],
        output='screen'
    )

    set_pointcloud_ordered = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/camera/camera',
             'pointcloud__neon_.ordered_pc', 'false'],
        output='screen'
    )

    delayed_pointcloud_params = TimerAction(
        period=10.0,
        actions=[set_pointcloud_enable, set_pointcloud_ordered]
    )

    node_detection = Node(
        package="ambula_perception",
        executable="opening_detector",
        name="opening_detector",
        parameters=[{detection_config_path}],
        output="screen",
    )

    return LaunchDescription([
        included_realsense,
        imu_filter_node,
        static_tf,
        delayed_pointcloud_params,
        node_detection,
        # depth2laser_node,
        # rtabmap_odom_node,
        # nav2_nodes
    ])