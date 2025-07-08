import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import math
from tf_transformations import euler_from_quaternion
import time
from periphery import PWM

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        # 初始化串口通信
        self.ser = serial.Serial('/dev/ttyS1', 115200, timeout=1)
        self.ser_lock = threading.Lock()

        # 订阅各模块结果话题
        self.lidar_sub = self.create_subscription(String, 'lidar_result', self.lidar_result_callback, 10)
        self.yolov8_sub = self.create_subscription(String, 'yolov8_result', self.yolov8_result_callback, 10)
        # self.line_tracking_sub = self.create_subscription(String, 'line_tracking_result', self.line_tracking_result_callback, 10)
        self.circle_laser_sub = self.create_subscription(String, 'circle_laser_result', self.circle_laser_result_callback, 10)
        self.t265_sub = self.create_subscription(PoseStamped, 't265_publisher/pose', self.t265_pose_callback, 10)
        # 新增：订阅 target_position 话题
        self.target_sub = self.create_subscription(PoseStamped, 'target_position', self.target_callback, 10)
        # 存储当前等待的指令
        self.current_command = None

        # 创建串口数据接收发布者
        self.serial_receive_pub = self.create_publisher(String, 'serial_receive', 10)
        # 创建串口数据发送订阅者
        self.serial_send_sub = self.create_subscription(String, 'serial_send', self.serial_send_callback, 10)

        # 启动串口接收线程
        self.serial_thread = threading.Thread(target=self.receive_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.task_lists = ['00','r01','r02','05','r11','12','r15','r21','r25','r31','r35','r41','r45']

        # 坐标系
        self.origin = (None,None, None) # 全局坐标系
        self.cup_lidar = (None,None) # 杯子坐标系
        self.cup_map = (None,None)  # 杯子全局坐标系
        self.target = (None,None) # 目标坐标
        self.roads = [(100,0),(100,-100),(0,-100),(0,0)] # 循环测试
        self.target_index = 0

    def receive_serial_data(self):
        while rclpy.ok():
            try:
                with self.ser_lock:
                    if self.ser.in_waiting:
                        # 读取数据直到遇到 \r\n 结束符
                        command = self.ser.read_until(b'\r\n').decode('utf-8', errors='ignore')
                        # 去除 \r\n 及首尾空白字符
                        command = command.strip('\r\n').strip() 
                        self.get_logger().info(f"收到单片机指令: {command}")
                        self.current_command = command
                        # 打印 command 原始值和长度
                        self.get_logger().info(f"收到单片机指令原始值: '{command}', 长度: {len(command)}")
                        # 发布接收到的串口数据
                        msg = String()
                        msg.data = command
                        self.serial_receive_pub.publish(msg)
                        # 对于持续请求，可添加定时器等逻辑，此处简单处理单次请求
                        if command not in self.task_lists: # 不在任务序列里
                            self.handle_special_command(command)
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {e}")

    def serial_send_callback(self, msg):
        try:
            with self.ser_lock:
                self.ser.write(msg.data.encode('utf-8'))
            # if msg.data[-1] != '\r':
            #     self.ser.write('\r'.encode('utf-8'))
            # if msg.data[1] != '3':
            self.get_logger().info(f"发送数据到串口: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"串口发送错误: {e}")

    # ========== 任务一 =========
    def lidar_result_callback(self, msg):
        if self.current_command == 'r11':  # 只发一次
            self.get_logger().info(f"------ 任务一：发送杯子位置 ------")
            self.get_logger().info(f"杯子的位置: {msg.data}")
            parts = msg.data.split(',')
            # 检查分割后的列表长度，避免索引越界
            if len(parts) >= 5:
                try:
                    self.cup_lidar = (int(parts[4]), int(parts[3]))
                    self.get_logger().info(f"杯子雷达位置: {self.cup_lidar}")
                    self.cup_map = (int(self.cup_lidar[0]), int(self.cup_lidar[1]))
                    self.get_logger().info(f"杯子地图位置: {self.cup_map}")
                    # 确保分割后的列表有足够元素
                    if len(parts) >= 2:
                        send_msg = String()
                        send_msg.data = f"@1,{parts[1]},{parts[2]}\r"
                        self.serial_send_callback(send_msg)
                    else:
                        self.get_logger().error("消息分割后元素不足，无法发送数据")
                except (IndexError, ValueError):
                    self.get_logger().error("解析杯子位置数据失败，可能包含非数字内容")
            else:
                self.get_logger().error("消息分割后元素不足，无法获取杯子位置")
            self.current_command = None

        if self.current_command == 'r12':  # 只发一次
            send_msg = String()
            send_msg.data = '@2,' + f"{self.cup_map[0]}," + f"{self.cup_map[1]}" + '\r'
            self.get_logger().info(f"发送杯子地图位置: {send_msg.data}")
            self.serial_send_callback(send_msg)
            self.current_command = None
        if self.current_command == 'r05\rr12':  # 只发一次
            send_msg = String()
            send_msg.data = '@2,' + f"{self.cup_map[0]}," + f"{self.cup_map[1]}" + '\r'
            self.get_logger().info(f"发送杯子地图位置: {send_msg.data}")
            self.serial_send_callback(send_msg)
            time.sleep(0.01)
            # 重新发目标
            send_msg = String()
            send_msg.data = '@3,' + f"{self.origin[0]}," + f"{self.origin[1]}," + f'{self.origin[2]}' + '\r'
            self.get_logger().info(f"发送目标位置: {send_msg.data}")
            self.serial_send_callback(send_msg)
            self.current_command = None

        elif self.current_command == 'r15': # 停止发
            self.get_logger().info(f"结束杯子识别")
            self.current_command = None

    # ========== 任务二 识别数字 ========
    def yolov8_result_callback(self, msg):
        if self.current_command == 'r21':  # 识别数字
            self.get_logger().info(f"------ 任务二：识别数字 ------")
            self.get_logger().info(f"识别的数字结果: {msg.data}")
            send_msg = String()
            send_msg.data = msg.data
            self.serial_send_callback(send_msg)
            self.current_command = None
        elif self.current_command == 'r25': #
            self.get_logger().info(f"结束数字识别")
            self.current_command = None

    # ========== 任务三 ========


    def circle_laser_result_callback(self, msg):
        if self.current_command == 'r41':  # 识别数字

    def handle_special_command(self, command):
            self.get_logger().warn("未知指令")

    def t265_pose_callback(self, msg: PoseStamped):
        # 保存坐标
        # 重组数据
        send_msg = String()
        # 换算 yaw 值
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        # 调用函数解算欧拉角
        yaw = euler_from_quaternion(quat)[2]
        # 将弧度转换为角度
        yaw_deg = math.degrees(yaw)
        send_msg.data = f"@3,{int(msg.pose.position.x*100)},{int(msg.pose.position.y*100)},{yaw_deg:.2f}\r"
        self.serial_send_callback(send_msg)
        if self.current_command == 'r0': # 保存 C 点坐标
            # 重组数据
            send_msg = String()
            # 换算 yaw 值
            quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            # 调用函数解算欧拉角
            yaw = euler_from_quaternion(quat)[2]
            # 将弧度转换为角度
            yaw_deg = math.degrees(yaw)
            self.origin = (int(100*msg.pose.position.x),int(100*msg.pose.position.y),yaw_deg)
            self.get_logger().info(f"保存 C 点坐标: {self.origin}")
            # send_msg = String()
            # send_msg.data = '@3,' + f"{self.origin[0]}," + f"{self.origin[1]}" + '\r'
            # self.serial_send_callback(send_msg)
            self.current_command = None
        if self.current_command == 'r01':
            msg = String()
            msg.data = '@3,'+ f"{self.origin[0]}," + f"{self.origin[1]}" + '\r'
            self.get_logger().info(f"发送 C 点坐标: {msg.data}")
            self.serial_send_callback(msg)
            self.current_command = None
        if self.current_command == 'r05':
            # 重组数据
            send_msg = String()
            # 换算 yaw 值
            quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            # 调用函数解算欧拉角
            yaw = euler_from_quaternion(quat)[2]
            # 将弧度转换为角度
            yaw_deg = math.degrees(yaw)
            send_msg.data = f"@3,{int(msg.pose.position.x*100)},{int(msg.pose.position.y*100)},{yaw_deg:.2f}\r"
            self.get_logger().info("t265:",send_msg.data)
        elif self.current_command == 'r01': # 保存 A 点坐标
            self.cup = (msg.pose.position.x,msg.pose.position.y)
            self.get_logger().info(f"保存 cup 点坐标: {self.cup}")

    # 新增：处理目标点的回调函数
    def target_callback(self, msg: PoseStamped):
        """
        处理接收到的目标点信息，将目标点信息通过串口发送。
        Args:
            msg (PoseStamped): 包含目标点位置和姿态信息的消息。
        """
        # 先处理指令相关逻辑
        if self.current_command == 'r01':  # 请求目标点
            self.target = (None, None)
            self.get_logger().info("重发目标点！")
            if msg:  # 确保 msg 存在
                send_msg = String()
                send_msg.data = f"@2,{msg.pose.position.x:.2f},{msg.pose.position.y:.2f}\r"
                self.serial_send_callback(send_msg)
            self.current_command = None
        elif self.current_command == 'r02':
            self.target_index += 1
            if self.target_index >= len(self.roads):
                self.target_index = 0
            self.target = self.roads[self.target_index]
            send_msg = String()
            send_msg.data = f"@2,{self.target[0]:.2f},{self.target[1]:.2f}\r"
            self.get_logger().info(f"切换下一个目标点{self.target_index}: {send_msg.data}")
            self.serial_send_callback(send_msg)
            self.current_command = None

        # 再处理目标点相同只发一次的逻辑
        # if self.target != (msg.pose.position.x, msg.pose.position.y):
        #     self.target = (msg.pose.position.x, msg.pose.position.y)
        #     send_msg = String()
        #     send_msg.data = f"@2,{msg.pose.position.x:.2f},{msg.pose.position.y:.2f}\r"
        #     self.serial_send_callback(send_msg)



    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    main_controller = MainController()
    main_controller.get_logger().info("主控制器节点已启动!")
    main_controller.get_logger().info("启动串口 @/dev/ttyS1 115200!")
    try:
        rclpy.spin(main_controller)
    except KeyboardInterrupt:
        pass
    finally:
        main_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()