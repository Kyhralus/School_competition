"""
    功能：
        启动car_t265包的节点
        1. motor_controller_node：电机控制节点
        2. uart_driver_node：串口驱动节点
        3. target_publisher_node：目标发布节点
        4. navigation_controller_node：导航控制节点
        5. t265_publisher_node：T265定位发布节点
    作者： Kyhralus <alineyiee@shu.edu.cn>
    时间： 2025.06.26
    版本： 1.0.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_controller_node = Node(
        package='car_t265',
        executable='motor_controller',
        name='motor_controller'
    )
    uart_driver_node = Node(
        package='car_t265',
        executable='uart_driver',
        name='uart_driver'
    )
    # target_publisher_node = Node(
    #     package='car_t265',
    #     executable='target_publisher',
    #     name='target_publisher'
    # )
    navigation_controller_node = Node(
        package='car_t265',
        executable='navigation_controller',
        name='navigation_controller'
    )
    t265_publisher_node = Node(
        package='car_t265',
        executable='t265_publisher',
        name='t265_publisher'
    )

    return LaunchDescription([
        motor_controller_node,
        uart_driver_node,
        # target_publisher_node,
        navigation_controller_node,
        t265_publisher_node
    ])