import rclpy                                      
from rclpy.node   import Node 
import serial
from std_msgs.msg import String


class UartReceiver(Node):

    def __init__(self, name, serial_port, baudrate, topic_statck=10):
        super().__init__(name)                                      

        self.name = name
        self.topic_stack = topic_statck

        self.pub = self.create_publisher(
            String, name + "_data_topic",  topic_statck)   
        
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.sink_serial = serial.Serial(self.serial_port, self.baudrate)         
          
    def run(self):
        self.get_logger().warning(f'启动运行')  
        while rclpy.ok():
            try:
                if self.sink_serial.isOpen():
                    serial_data_length = self.sink_serial.inWaiting()	
                    if serial_data_length:
                        serial_data = self.sink_serial.read(serial_data_length)
                        #源数据是以字节形式存在，用str转成字符串，去掉首字符b
                        serial_data = str(str(serial_data)[1:].replace("\'",""))
                        self.publish(serial_data)
            except Exception as e:
                self.get_logger().error(str(e))

    def publish(self, data):
        msg = String()                                            
        msg.data = data                                    
        self.pub.publish(msg)                                    
        self.get_logger().info(f'接收数据发布成功 | 数据：{msg.data}')


    def uart_receiver0_init(self, topic_stack=10):
        self.sub_uart_receiver0 = self.create_subscription(String, "uart_receiver0_data_topic",
                                                        self.uart_receiver1_callback, topic_stack)


def main(args=None):
    rclpy.init(args=args)
    uartReceiver = UartReceiver("uart_receiver0", "/dev/ttyS0", 115200)
    uartReceiver.get_logger().info(f"uart_receiver0 @ {uartReceiver.serial_port} Buad: {uartReceiver.baudrate} init success")
    uartReceiver.run()
    rclpy.spin(uartReceiver)
    uartReceiver.destroy_node()
    rclpy.shutdown()