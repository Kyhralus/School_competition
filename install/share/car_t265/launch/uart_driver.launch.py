from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明 串口 启动参数
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyS0',
            description='Serial port device'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial port baudrate'
        ),
       
        # 启动UART驱动
        Node(
            package='car_t265',
            executable='uart_driver',
            name='uart_driver',
            parameters=[
                {'port': LaunchConfiguration('serial_port')},
                {'baudrate': LaunchConfiguration('baudrate')}
            ],
            output='screen'
        ),
        
    ])