import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UartReceiver(Node):
    def __init__(self, name, topic_stack=10):
        super().__init__(name)
        self.name = name
        self.topic_stack = topic_stack

        # 创建订阅者，接收串口数据
        self.sub_receive = self.create_subscription(
            String, "uart_manager_received_data", self.receive_data, topic_stack)

    def receive_data(self, msg):
        self.get_logger().info(f"接收到串口数据: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    uart_receiver = UartReceiver("uart0_receiver")
    rclpy.spin(uart_receiver)
    uart_receiver.destroy_node()
    rclpy.shutdown()