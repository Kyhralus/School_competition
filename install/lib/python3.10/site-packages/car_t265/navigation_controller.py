# """
#     功能：
#         导航控制节点：
#         1. 订阅T265的坐标数据和目标位置数据
#         2. 计算控制指令并发布
#         3. 实现简单的路径导航功能：
#             3.1 到达目标位置：
#                 3.1.1 距离目标位置小于阈值时，停止运动
#             3.2 未到达目标位置：
#                 3.2.1 计算目标位置与当前位置的角度误差
#                 3.2.2 根据角度误差调整控制指令
#                 3.2.3 发布控制指令
#                 3.2.4 记录日志 （可选）
#     作者： Kyhralus <alineyiee@shu.edu.cn>
#     时间： 2025-06-26
#     版本： 1.0
# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, Twist
# import math
# from tf_transformations import euler_from_quaternion

# class NavigationController(Node):
#     def __init__(self):
#         super().__init__('navigation_controller')
        
#         # 订阅T265的坐标数据
#         self.pose_sub = self.create_subscription(
#             PoseStamped,
#             '/t265_publisher/pose',
#             self.pose_callback,
#             10    # 队列长度
#         )
        
#         # 订阅目标位置
#         self.target_sub = self.create_subscription(
#             PoseStamped,
#             '/target_position',
#             self.target_callback,
#             10
#         )
        
#         # 发布Twist消息
#         self.cmd_pub = self.create_publisher(
#             Twist,
#             '/cmd_vel',
#             10
#         )
        
#         # 初始化变量
#         self.current_pose = None
#         self.target_pose = None
        
#         # 控制参数
#         self.distance_threshold = 0.05  # 距离阈值（米）
#         self.angle_threshold_min = 0.1     # 角度阈值（弧度）, 0.1pi -> 0.1*pi * 180°/pi = 18°
#         self.angle_threshold_mid = 0.25     # 角度阈值（弧度）, 0.25pi -> 0.25*pi * 180°/pi = 45°
#         self.angle_threshold_max = 0.67     # 角度阈值（弧度）, 0.677pi -> 0.67*pi * 180°/pi == 120°
        
#         self.get_logger().info('导航控制节点初始化成功！')

#         # self.log_file = open('nav_controller_log.txt', 'a')
#         # self.log_file.write('timestamp x y target_x target_y target_angle current_angle angle_error distance\n')
    
#     def pose_callback(self, msg):
#         """处理T265的坐标数据"""
#         self.current_pose = msg
#         if self.target_pose is not None:
#             self.calculate_control()
    
#     def target_callback(self, msg):
#         """处理目标位置数据"""
#         self.target_pose = msg
#         self.get_logger().info(f"收到新目标点: x={msg.pose.position.x}, y={msg.pose.position.y}")
    
#     def calculate_control(self):
#         """简化版：直接给定固定速度，主控已做PID闭环"""
#         if self.current_pose is None or self.target_pose is None:
#             if self.current_pose is None:
#                 self.get_logger().info("t265位姿未接收到！")
#             if self.target_pose is None:
#                 self.get_logger().info("目标点未接收到！")
#             return
#         # 获取当前位置和目标位置
#         current_x = self.current_pose.pose.position.x
#         current_y = self.current_pose.pose.position.y
#         target_x = self.target_pose.pose.position.x
#         target_y = self.target_pose.pose.position.y
#         # 计算与目标点的距离
#         dx = target_x - current_x
#         dy = target_y - current_y
#         distance = math.sqrt(dx**2 + dy**2)
#         # 计算目标角度
#         target_angle = math.atan2(dy, dx)
#         # 获取当前朝向（四元数转欧拉角）
#         q = self.current_pose.pose.orientation
#         quat = [q.x, q.y, q.z, q.w]
#         roll, pitch, yaw = euler_from_quaternion(quat)  # 用pitch轴，是实际yaw轴。得到弧度

#         # siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         # cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         # current_angle = math.atan2(siny_cosp, cosy_cosp)
#         current_angle = yaw

#         # 计算角度误差
#         angle_error = normalize_angle(target_angle - current_angle)  # 换算成 -pi - pi
#         # 创建控制消息
#         # 角速度固定 0.5 , -0.5 rad/s
#         # 线速度 0.2 m/s
#         cmd = Twist()
#         if distance < self.distance_threshold:
#             cmd.linear.x = 0.0
#             cmd.angular.z = 0.0
#             self.get_logger().info("到达目标位置!")
#         else:
#             if abs(angle_error) > self.angle_threshold_max:   # 转弯幅度大于120°
#                 cmd.linear.x = 0.0
#                 cmd.angular.z = 0.5 if angle_error > 0 else -0.5
#             elif abs(angle_error) > self.angle_threshold_mid:  # 转弯幅度 [45°, 120°)
#                 cmd.linear.x = 0.1
#                 cmd.angular.z = 0.2 if angle_error > 0 else -0.2
#             elif abs(angle_error) > self.angle_threshold_min:  #  转弯幅度 [18°, 45°)
#                 cmd.linear.x = 0.2
#                 cmd.angular.z = 0.1 if angle_error > 0 else -0.1
#             else:  # 没有角度误差就直走
#                 cmd.linear.x = 0.2
#                 cmd.angular.z = 0.0
#         # 打印最终使用的角度（保留两位小数）
#         self.get_logger().info(
#             f"控制计算: "
#             f"目标角度={math.degrees(target_angle):.2f}°, "
#             f"当前朝向={math.degrees(current_angle):.2f}°, "
#             f"角度误差={angle_error:.2f} pi, "
#             f"距离={distance:.2f}m"
#             f"发送线速度：{cmd.linear.x}角速度：{cmd.angular.z}"
#         )
#         self.cmd_pub.publish(cmd)
#         # 日志
#         # self.log_file.write(
#         #     f"{self.get_clock().now().to_msg().sec}.{self.get_clock().now().to_msg().nanosec:09d} "
#         #     f"{current_x:.4f} {current_y:.4f} {target_x:.4f} {target_y:.4f} "
#         #     f"{math.degrees(target_angle):.2f} {math.degrees(current_angle):.2f} {math.degrees(angle_error):.2f} {distance:.4f}\n"
#         # )
#         # if distance < self.distance_threshold:
#         #     self.log_file.write("到达目标点")
#         # self.log_file.flush()

# def normalize_angle(angle):
#     """将任意角度标准化到 (-pi, pi] 区间"""
#     return (angle + math.pi) % (2 * math.pi) - math.pi


# def main(args=None):
#     rclpy.init(args=args)
#     node = NavigationController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         # 日志
#         # if hasattr(node, 'log_file') and node.log_file:
#         #     node.log_file.close()   
#         if rclpy.ok():  # 检查上下文是否仍处于活动状态，防止多个节点同时关闭rclpy
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math
from tf_transformations import euler_from_quaternion

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # 订阅T265的坐标数据
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/t265_publisher/pose',
            self.pose_callback,
            10
        )
        
        # 订阅目标位置（String类型）
        self.target_sub = self.create_subscription(
            String,
            '/target_position',
            self.target_callback,
            10
        )
        
        # 发布Twist消息
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 初始化变量
        self.current_pose = None
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.reached_position = False  # 是否已到达目标点
        
        # 控制参数
        self.distance_threshold = 0.05
        self.angle_threshold_min = 0.314     # 角度阈值（弧度）, 0.1pi -> 0.1*pi * 180°/pi = 18°
        # self.angle_threshold_mid = 0.25     # 角度阈值（弧度）, 0.25pi -> 0.25*pi * 180°/pi = 45°
        self.angle_threshold_max = 2.10    # 角度阈值（弧度）, 0.677pi -> 0.67*pi * 180°/pi == 120°  
        self.yaw_threshold = 0.1  # 到点后调整朝向的阈值（弧度）

        self.get_logger().info('导航控制节点初始化成功！')
    
    def pose_callback(self, msg):
        self.current_pose = msg
        if self.target_x is not None and self.target_y is not None and self.target_yaw is not None:
            self.calculate_control()
    
    def target_callback(self, msg):
        try:
            # 解析字符串，格式如 "x:1.0,y:2.0,yaw:0.0"
            data = dict(item.split(":") for item in msg.data.split(","))
            self.target_x = float(data.get("x", 0.0))
            self.target_y = float(data.get("y", 0.0))
            self.target_yaw = float(data.get("yaw", 0.0))  # 角度 --> 弧度
            self.reached_position = False  # 新目标点，重置标志
            self.get_logger().info(f"收到新目标点: x={self.target_x}, y={self.target_y}, yaw={self.target_yaw:.2f}")
            self.target_yaw = math.radians(float(data.get("yaw", 0.0)))  # 角度 --> 弧度
        except Exception as e:
            self.get_logger().error(f"目标点字符串解析失败: {e}")
    
    def calculate_control(self):
        if self.current_pose is None or self.target_x is None or self.target_y is None or self.target_yaw is None:
            self.get_logger().debug("没有收到t265或目标信息！")
            return
        # 获取当前位置和目标位置
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        # 计算与目标点的距离
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        # 获取当前朝向
        q = self.current_pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        current_angle = yaw

        cmd = Twist()
        if not self.reached_position:
            # 还未到达目标点，先走到目标点
            target_angle = math.atan2(dy, dx)
            angle_error = normalize_angle(target_angle - current_angle)
            if distance < self.distance_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.reached_position = True
                self.get_logger().info("到达目标位置，准备调整朝向！")
            else:
                # self.get_logger().info(f"angle_error: { angle_error }")
                if abs(angle_error) > self.angle_threshold_max:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5 if angle_error > 0 else -0.5
                # elif abs(angle_error) > self.angle_threshold_mid:
                #     cmd.linear.x = 0.1
                #     cmd.angular.z = 0.35 if angle_error > 0 else -0.35
                elif abs(angle_error) > self.angle_threshold_min:
                    cmd.linear.x = 0.2
                    cmd.angular.z = 0.2 + 0.167*(angle_error - self.angle_threshold_min)  if angle_error > 0 else -0.2 - 0.167*(angle_error + self.angle_threshold_min)  # 线性增加误差
                else:
                    cmd.linear.x = 0.2
                    cmd.angular.z = 0.0
        else:
            # 已到达目标点，调整朝向
            yaw_error = normalize_angle(self.target_yaw - current_angle)
            if abs(yaw_error) < self.yaw_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("目标点朝向调整完成！")
            else:
                if abs(yaw_error) > self.angle_threshold_max:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5 if yaw_error > 0 else -0.5
                # elif abs(angle_error) > self.angle_threshold_mid:
                #     cmd.linear.x = 0.1
                #     cmd.angular.z = 0.35 if angle_error > 0 else -0.35
                elif abs(yaw_error) > self.angle_threshold_min:   # 0.1 < yaw_error <= 0.67
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.2 + 0.167*(yaw_error - self.angle_threshold_min) if yaw_error > 0 else -0.2 - 0.167*(yaw_error + self.angle_threshold_min)  # 线性增加误差
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                # self.get_logger().info(f"原地angle_error: {angle_error:.2f} = {math.degrees(angle_error):.2f} , yaw_error: {yaw_error:.2f} = {math.degree(yaw_error):.2f}°, 角速度：{cmd.angular.z:.2f}")
                self.get_logger().info(f"调整朝向中，yaw误差: {math.degrees(yaw_error):.2f}°, 角速度：{cmd.angular.z:.2f}")
        
        self.cmd_pub.publish(cmd)

def normalize_angle(angle):
    """将任意角度标准化到 (-pi, pi] 区间"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()