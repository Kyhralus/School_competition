from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        
        # 电机控制
        Node(
            package='car_t265',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),
        
        # 导航规划
        Node(
            package='car_t265',
            executable='navigation_controller',
            name='navigation_controller',
            output='screen'
        ),
        
    ])