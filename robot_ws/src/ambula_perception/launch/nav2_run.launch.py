import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ambula_perception')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_local_params.yaml')
    realsense_dir = get_package_share_directory('realsense2_camera')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

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
            'enable_sync': 'true',
            'enable_color': 'true',         # เพิ่ม: เพื่อใช้ในการ Label และ Debug
            'align_depth.enable': 'true',   # เพิ่ม: หัวใจสำคัญของการทำ Person Detection คู่กับ Depth
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'ordered_pc': 'true',
            'pointcloud.enable': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_module.profile': '640x480x30',   # เพิ่ม: กำหนดขนาดภาพสีให้เท่ากับ Depth
            'infra_module.profile': '640x480x30',
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

    # 3. Static TF (หัวใจสำคัญ: เชื่อม base_link เข้ากับโครงสร้างกล้อง)
    # จากภาพของคุณ odom และ base_link แยกกันอยู่ เราต้องจับ base_link ไปต่อกับ camera_link
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
            'wait_for_transform': 0.5, # เพิ่มเผื่อไว้กัน Error TF ที่เจอเมื่อกี้
        }],
        remappings=[
            # เปลี่ยนจาก image_raw เป็น image_rect_raw ตามรูป
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
            'scan_height': 10, # ใช้กี่แถวของภาพมาสร้าง scan
            'output_frame': 'camera_link',
            'range_min': 0.3,
            'range_max': 3.0
        }]
    )

    return LaunchDescription([
        included_realsense,
        imu_filter_node,
        static_tf,
        depth2laser_node,
        # rtabmap_odom_node,
        # nav2_nodes
    ])