import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from typing import Optional

# Define serial parameter mapping dictionaries
DATA_BITS_MAP = {8: serial.EIGHTBITS}
PARITY_MAP = {'none': serial.PARITY_NONE, 'even': serial.PARITY_EVEN, 'odd': serial.PARITY_ODD}
STOP_BITS_MAP = {1: serial.STOPBITS_ONE, 2: serial.STOPBITS_TWO}


class UARTNode(Node):
    def __init__(self):
        super().__init__('uart_node')
        node_name = self.get_name()
        # Declare and get parameters
        self.device_path = self._declare_and_get_param(node_name, 'device_path', '/dev/ttyS0')
        self.baud_rate = self._declare_and_get_param(node_name, 'baud_rate', 115200)
        self.data_bits = self._declare_and_get_param(node_name, 'data_bits', 8)
        self.parity = self._declare_and_get_param(node_name, 'parity', 'none')
        self.stop_bits = self._declare_and_get_param(node_name, 'stop_bits', 1)
        self.timeout_ms = self._declare_and_get_param(node_name, 'timeout_ms', 100)
        self.receive_format = self._declare_and_get_param(node_name, 'receive_format', 'ascii')
        self.send_format = self._declare_and_get_param(node_name, 'send_format', 'ascii')
        self.frame_delimiter = self._declare_and_get_param(node_name, 'frame_delimiter', '\r\n')
        self.encoding = self._declare_and_get_param(node_name, 'encoding', 'utf-8')
        self.enabled = self._declare_and_get_param(node_name, 'enabled', True)

        self.ser: Optional[serial.Serial] = None
        self.buffer = bytearray()

        if not self.enabled:
            self.get_logger().info("UART device is disabled")
            return

        if not self.device_path:
            self.get_logger().error("device_path not set")
            return

        self._init_serial_port()
        self._create_publisher_and_subscriber()
        self._create_read_timer()

    def _declare_and_get_param(self, node_name: str, param_name: str, default_value):
        """Declare and get parameter values"""
        full_param_name = f"{node_name}.{param_name}"
        self.declare_parameter(full_param_name, default_value)
        return self.get_parameter(full_param_name).value

    def _init_serial_port(self):
        """Initialize the serial port"""
        try:
            self.ser = serial.Serial(
                port=self.device_path,
                baudrate=self.baud_rate,
                bytesize=DATA_BITS_MAP.get(self.data_bits, serial.EIGHTBITS),
                parity=PARITY_MAP.get(self.parity, serial.PARITY_NONE),
                stopbits=STOP_BITS_MAP.get(self.stop_bits, serial.STOPBITS_ONE),
                timeout=self.timeout_ms / 1000.0
            )
            self.get_logger().info(
                f"UART initialized: {self.device_path}, Baud: {self.baud_rate}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize UART: {e}")

    def _create_publisher_and_subscriber(self):
        """Create publishers and subscribers"""
        self.publisher_ = self.create_publisher(
            String, '/uart/recv_data', 10
        )
        self.subscription = self.create_subscription(
            String, '/uart/send_data',
            self.send_data_callback, 10
        )
        self.subscription  # Suppress unused variable warning

    def _create_read_timer(self):
        """Create a read timer"""
        self.timer = self.create_timer(0.01, self.read_uart_data)

    def read_uart_data(self):
        """Read serial data"""
        if not self.ser or not self.ser.is_open:
            return

        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                return

            self.buffer.extend(data)
            self._process_received_data()
        except Exception as e:
            self.get_logger().error(f"Error reading UART: {e}")
            self.buffer = bytearray()

    def _process_received_data(self):
        """Process received data"""
        if self.receive_format == 'ascii':
            delimiter = self.frame_delimiter.encode(self.encoding)
            while delimiter in self.buffer:
                frame, self.buffer = self.buffer.split(delimiter, 1)
                self._publish_ascii_data(frame)
        else:  # Binary mode
            delimiter = bytes([int(x, 16) for x in self.frame_delimiter.split()])
            while delimiter in self.buffer:
                frame, self.buffer = self.buffer.split(delimiter, 1)
                self._publish_binary_data(frame)

    def _publish_ascii_data(self, frame: bytes):
        """Publish ASCII - formatted data"""
        try:
            message = frame.decode(self.encoding)
            msg = String()
            msg.data = message
            self.publisher_.publish(msg)
            self.get_logger().debug(f"Received ASCII: {message}")
        except UnicodeDecodeError as e:
            self.get_logger().error(f"Error decoding ASCII data: {e}")

    def _publish_binary_data(self, frame: bytes):
        """Publish binary - formatted data"""
        msg = String()
        msg.data = frame.hex()
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Received Binary: {msg.data}")

    def send_data_callback(self, msg: String):
        """Callback function for processing sent data"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warning("UART not open, cannot send data")
            return

        try:
            if self.send_format == 'ascii':
                self._send_ascii_data(msg.data)
            else:  # Binary
                self._send_binary_data(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")

    def _send_ascii_data(self, data: str):
        """Send ASCII - formatted data"""
        self.ser.write(data.encode(self.encoding))
        self.get_logger().debug(f"Sent ASCII: {data}")

    def _send_binary_data(self, data: str):
        """Send binary - formatted data"""
        try:
            binary_data = bytes.fromhex(data)
            self.ser.write(binary_data)
            self.get_logger().debug(f"Sent Binary: {data}")
        except ValueError:
            self.get_logger().error(f"Invalid binary data: {data}")


def main(args: Optional[str] = None):
    rclpy.init(args=args)
    node = UARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()