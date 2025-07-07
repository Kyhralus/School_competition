import rclpy                                      
from rclpy.node   import Node 
import serial
from std_msgs.msg import String


class UartReceiver(Node):

    def __init__(self, name, serial_port, baudrate, topic_stack=10):
        super().__init__(name)
        self.name = name
        self.topic_stack = topic_stack  # 修正拼写错误

        self.pub = self.create_publisher(
            String, name + "_data_topic",  topic_stack)   
        
        self.serial_port = serial_port
        self.baudrate = baudrate
        try:
            self.sink_serial = serial.Serial(self.serial_port, self.baudrate)
            if not self.sink_serial.isOpen():
                self.sink_serial.open()
            self.get_logger().info(f"串口 {self.serial_port} 打开成功")
        except Exception as e:
            self.get_logger().error(f"无法打开串口 {self.serial_port}: {str(e)}")
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
                    self.publish(serial_data)
        except Exception as e:
            self.get_logger().error(str(e))

    def publish(self, data):
        msg = String()                                            
        msg.data = data                                    
        self.pub.publish(msg)                                    
        self.get_logger().info(f'接收数据发布成功 | 数据：{msg.data}')


    def uart_receiver1_init(self, topic_stack=10):
        self.sub_uart_receiver0 = self.create_subscription(String, "uart_receiver1_data_topic",
                                                        self.uart_receiver1_callback, topic_stack)


def main(args=None):
    rclpy.init(args=args)
    uartReceiver = UartReceiver("uart_receiver1", "/dev/ttyS1", 115200)
    uartReceiver.get_logger().info(f"uart_receiver1 @ {uartReceiver.serial_port} Buad: {uartReceiver.baudrate} init success")
    rclpy.spin(uartReceiver)
    uartReceiver.destroy_node()
    rclpy.shutdown()