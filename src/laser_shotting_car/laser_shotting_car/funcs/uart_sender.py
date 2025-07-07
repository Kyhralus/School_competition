import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UartSender(Node):
    def __init__(self, name, topic_stack=10):
        super().__init__(name)
        self.name = name
        self.topic_stack = topic_stack

        # 创建发布者，发布要发送的数据
        self.pub_send = self.create_publisher(
            String, "uart_manager_send_data", topic_stack)

    def send_data(self, data):
        msg = String()
        msg.data = data
        self.pub_send.publish(msg)
        self.get_logger().info(f"已发布要发送的数据: {data}")


def main(args=None):
    rclpy.init(args=args)
    uart_sender = UartSender("uart0_sender")

    try:
        while rclpy.ok():
            input_data = input("请输入要发送的数据（输入 'q' 退出）: ")
            if input_data.lower() == 'q':
                break
            uart_sender.send_data(input_data)
            rclpy.spin_once(uart_sender, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    uart_sender.destroy_node()
    rclpy.shutdown()