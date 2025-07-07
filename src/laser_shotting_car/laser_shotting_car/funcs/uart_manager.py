import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String


class UartManager(Node):
    def __init__(self, name, serial_port, baudrate, topic_stack=10):
        super().__init__(name)
        self.name = name
        self.topic_stack = topic_stack

        # 创建订阅者，接收要发送的数据
        self.sub_send = self.create_subscription(
            String, f"{name}_send_data", self.send_uart_data, topic_stack)

        # 创建发布者，发布接收到的串口数据
        self.pub_receive = self.create_publisher(
            String, f"{name}_received_data", topic_stack)

        self.serial_port = serial_port
        self.baudrate = baudrate
        try:
            # 初始化串口
            self.sink_serial = serial.Serial(self.serial_port, self.baudrate)
            if not self.sink_serial.isOpen():
                self.sink_serial.open()
            self.get_logger().info(
                f"串口 {self.serial_port} 打开成功，波特率: {self.baudrate}")
        except Exception as e:
            self.get_logger().error(
                f"无法打开串口 {self.serial_port}: {str(e)}")
            return

        # 使用定时器定期检查串口数据
        self.timer = self.create_timer(0.01, self.check_serial_data)

    def check_serial_data(self):
        try:
            if self.sink_serial.isOpen():
                serial_data_length = self.sink_serial.inWaiting()
                if serial_data_length:
                    serial_data = self.sink_serial.read(serial_data_length)
                    # 使用 decode 方法转换字节数据为字符串
                    serial_data = serial_data.decode('utf-8', errors='ignore')
                    self.publish_received_data(serial_data)
        except Exception as e:
            self.get_logger().error(f"读取串口数据出错: {str(e)}")

    def publish_received_data(self, data):
        msg = String()
        msg.data = data
        self.pub_receive.publish(msg)
        self.get_logger().info(f'接收数据发布成功 | 数据：{msg.data}')

    def send_uart_data(self, msg):
        self.get_logger().info(f"准备发送数据: {msg.data}")
        if self.sink_serial.isOpen():
            try:
                # 将字符串转换为字节并发送
                send_data = msg.data.encode('utf-8')
                send_count = self.sink_serial.write(send_data)
                if send_count == len(send_data):
                    self.get_logger().info(
                        f'数据发送成功，发送字节数: {send_count}')
                    return True
                else:
                    self.get_logger().warning(
                        f'部分数据发送失败，期望发送 {len(send_data)} 字节，实际发送 {send_count} 字节')
            except Exception as e:
                self.get_logger().error(f'数据发送出错: {str(e)}')
        return False


def main(args=None):
    rclpy.init(args=args)
    uart_manager = UartManager("uart_manager", "/dev/ttyS1", 115200)
    rclpy.spin(uart_manager)
    uart_manager.destroy_node()
    rclpy.shutdown()