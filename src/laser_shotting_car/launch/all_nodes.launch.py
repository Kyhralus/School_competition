import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 bluesea2 包的共享目录
    bluesea2_share_dir = get_package_share_directory('bluesea2')
    # 包含 uart_lidar.launch
    uart_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bluesea2_share_dir, 'launch', 'uart_lidar.launch.py')
        )
    )

    # 定义要启动的节点
    main_controller_node = Node(
        package='laser_shotting_car',
        executable='main_controller',
        name='main_controller',
        output='screen'
    )

    lidar_processor_node = Node(
        package='laser_shotting_car',
        executable='lidar_processor',
        name='lidar_processor',
        output='screen'
    )

    camera_publisher_node = Node(
        package='laser_shotting_car',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen'
    )

    yolov8_detector_node = Node(
        package='laser_shotting_car',
        executable='yolov8_detector',
        name='yolov8_detector',
        output='screen'
    )

    circle_laser_detector_node = Node(
        package='laser_shotting_car',
        executable='circle_laser_detector',
        name='circle_laser_detector',
        output='screen'
    )

    return LaunchDescription([
        uart_lidar_launch,
        main_controller_node,
        lidar_processor_node,
        camera_publisher_node,
        yolov8_detector_node,
        circle_laser_detector_node
    ])