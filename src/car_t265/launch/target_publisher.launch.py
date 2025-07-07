from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='car_t265',
            executable='target_publisher',
            name='target_publisher',
            output='screen'
        ),
        
    ])