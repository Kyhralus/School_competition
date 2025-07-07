"""
    启动 t265
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        
        # T265相机
        Node(
            package='car_t265',
            executable='t265_publisher',
            name='t265_publisher',
            output='screen',
            parameters=[
                {'frame_id': 't265_frame'},
                # {'publish_tf': True},
            ]
        ),
        
    ])