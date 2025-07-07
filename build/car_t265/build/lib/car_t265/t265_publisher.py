"""
    功能：
        T265相机位姿发布节点，坐标已经转换（相机朝上）
    作者： Kyhralus <alineyiee@shu.edu.cn>
    时间： 2025-06-26
    版本： 1.0
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import pyrealsense2 as rs
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from tf_transformations import euler_from_quaternion

class T265Publisher(Node):
    def __init__(self):
        super().__init__('t265_publisher')

        # 声明参数
        self.declare_parameter('frame_id', 't265_pose_frame')
        self.pub_count = 0
        # 创建发布者
        self.pose_pub = self.create_publisher(PoseStamped, '~/pose', 10)   # 队列长度
        
        # 配置T265相机
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.pose)

        # 启动相机
        try:
            self.pipeline.start(self.config)
            self.get_logger().info('T265 启动成功！')
        except Exception as e:
            self.get_logger().error(f"相机启动失败: {e}")
        
        self.get_logger().info('T265 发布方初始化！')
        
        # 创建定时器，定期处理相机数据
        self.timer = self.create_timer(0.1, self.process_camera_data)   # 50Hz

    def process_camera_data(self):
        """处理相机数据"""
        try:
            # 等待一帧数据
            frames = self.pipeline.wait_for_frames()
            
            # 处理位姿数据
            pose_frame = frames.get_pose_frame()
            if pose_frame:
                self.publish_pose(pose_frame)
        except Exception as e:
            self.get_logger().error(f"相机错误: {e}")

    def publish_pose(self, pose_frame):
        """发布位姿数据"""
        pose_data = pose_frame.get_pose_data()
        data_confidence = pose_data.tracker_confidence # 消息置信度

        # 创建位姿消息
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        
        # 设置位置
        # @TODO
        # 根据T265的pose坐标系转换成符合习惯的坐标系
        # 转换后的坐标系 以镜头为基准（树立安装）
        # x --- 朝镜头前
        # y --- 朝镜头右 ---》 更正为左
        # z --- 朝上
        msg.pose.position.x = pose_data.translation.z    # z
        msg.pose.position.y = pose_data.translation.x    # x
        msg.pose.position.z = pose_data.translation.y    # y
        
        # 解算欧拉角
        quat = [
            pose_data.rotation.x,
            pose_data.rotation.y,
            pose_data.rotation.z,
            pose_data.rotation.w,
        ]
        quat = convert_quaternion(quat)

        # 设置方向（四元数）
        # @TODO
        # T265 pose
        # 上 --- y
        # 前 --- z
        # 左 --- x
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3] 
        # 调用函数解算欧拉角
        roll, pitch, yaw = euler_from_quaternion(quat)
        # 将弧度转换为角度
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        # 高置信度位姿信息才发布
        if data_confidence >= 2:
            # 发布消息
            self.pose_pub.publish(msg)
            self.pub_count += 1
            if self.pub_count == 10:
                if data_confidence == 2:
                    self.get_logger().warn(
                        f'发布位姿[{data_confidence}]:(x: {msg.pose.position.x:.4f}, y: {msg.pose.position.y:.4f}, z:{msg.pose.position.z:.4f}), '
                        # f'roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}°'
                        f' yaw={yaw_deg:.2f}°'
                    )
                # elif data_confidence == 3:
                #     self.get_logger().info(
                #         f'发布位姿[{data_confidence}]:(x: {msg.pose.position.x:.4f}, y: {msg.pose.position.y:.4f}, z:{msg.pose.position.z:.4f}), '
                #         f'yaw={yaw_deg:.2f}°'
                #     )
                self.pub_count = 0

    def destroy_node(self):
        """节点关闭时清理资源"""
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()

def get_yaw_from_quaternion(q):
    # q: geometry_msgs.msg.Quaternion 或 [x, y, z, w]
    # 只用四元数分量解算yaw（绕z轴旋转）
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    x, y, z, w = q
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw

def convert_quaternion(q1):
    """
    Converts a quaternion representing a rotation in the x-y-z coordinate system to a quaternion
    representing the same rotation in the x-z-(-y) coordinate system.

    Args:
    q1: (x, y, z, w) quaternion representing a rotation in the x-y-z coordinate system.

    Returns:
    q2: (x, y, z, w) quaternion representing the same rotation in the x-z-(-y) coordinate system.
    """

    # Define a rotation that swaps the y and z axes and inverts the y axis
    swap_yz = R.from_euler('yxz', [0, 90, 0], degrees=True)

    # Convert q1 to a rotation object
    r1 = R.from_quat(q1)

    # Apply the coordinate system transformation and convert back to a quaternion
    r2 = swap_yz * r1 * swap_yz.inv()
    q2 = r2.as_quat()

    return q2

def data_process(msg):
    '''
        参考代码【遗留版】
    '''
    # z should be reverse
    msg.pose.position.z = msg.pose.position.z * (-1)
    # swap y and z
    tmp = msg.pose.position.y
    msg.pose.position.y = msg.pose.position.z
    msg.pose.position.z = tmp
    msg.pose.position.x = msg.pose.position.x * 1.07
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = T265Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # 检查上下文是否仍处于活动状态，防止多个节点同时关闭rclpy
            rclpy.shutdown()

if __name__ == '__main__':
    main()