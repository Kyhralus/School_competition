# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():
#     pkg_share = get_package_share_directory('laser_shotting_car')
#     # config_file = os.path.join(pkg_share, 'config', 'uart_config.yaml')

#     uart0_send_node = Node(
#         package='laser_shotting_car',
#         executable='uart0_sender',
#         name='uart0_send_node',
#         output='screen',
#     )
#     uart0_recv_node = Node(
#         package='laser_shotting_car',
#         executable='uart0_receiver',
#         name='uart0_recv_node',
#         output='screen',
#     )
#     uart1_send_node = Node(
#         package='laser_shotting_car',
#         executable='uart1_sender',
#         name='uart1_send_node',
#         output='screen',
#     )
#     uart1_recv_node = Node(
#         package='laser_shotting_car',
#         executable='uart1_receiver',
#         name='uart1_recv_node',
#         output='screen',
#     )

#     return LaunchDescription([
#         uart0_send_node,
#         uart0_recv_node,
#         uart1_send_node,
#         uart1_recv_node,
#     ])


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('laser_shotting_car')

    # 定义串口管理节点
    uart_manager_node = Node(
        package='laser_shotting_car',
        executable='uart_manager',
        name='uart_manager',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyS1',
            'baudrate': 115200
        }]
    )

    # 定义串口发送节点
    uart_sender_node = Node(
        package='laser_shotting_car',
        executable='uart_sender',
        name='uart0_sender',
        output='screen'
    )

    # 定义串口接收节点
    uart_receiver_node = Node(
        package='laser_shotting_car',
        executable='uart_receiver',
        name='uart0_receiver',
        output='screen'
    )

    return LaunchDescription([
        uart_manager_node,
        uart_sender_node,
        uart_receiver_node
    ])