import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import pyrealsense2 as rs


class T265ImuPublisher(Node):
    def __init__(self):
        super().__init__('t265_imu_publisher')
        
        # 声明参数
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('imu_topic', '/imu/data_raw')  # 默认全局话题，便于imu_tools直接用

        # 获取参数
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # 创建发布者
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        
        # 配置T265相机
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.pose)
        self.pipeline.start(self.config)
        
        self.get_logger().info(f'T265 IMU 发布方初始化成功！话题: {imu_topic}，frame_id: {frame_id}')
        
        # 定时器，定期发布IMU数据
        self.timer = self.create_timer(0.1, self.publish_imu_data)
        self.frame_id = frame_id
    
    def publish_imu_data(self):
        """发布IMU数据"""
        try:
            frames = self.pipeline.wait_for_frames()
            pose_frame = frames.get_pose_frame()
            if pose_frame:
                pose_data = pose_frame.get_pose_data()
                imu_msg = Imu()
                imu_msg.header.frame_id = self.frame_id
                imu_msg.header.stamp = self.get_clock().now().to_msg()

                # 坐标系转换
                imu_msg.linear_acceleration.x = pose_data.acceleration.z
                imu_msg.linear_acceleration.y = pose_data.acceleration.x
                imu_msg.linear_acceleration.z = pose_data.acceleration.y

                imu_msg.angular_velocity.x = pose_data.angular_velocity.z
                imu_msg.angular_velocity.y = pose_data.angular_velocity.x
                imu_msg.angular_velocity.z = pose_data.angular_velocity.y

                imu_msg.orientation.x = pose_data.rotation.z
                imu_msg.orientation.y = pose_data.rotation.x
                imu_msg.orientation.z = pose_data.rotation.y
                imu_msg.orientation.w = pose_data.rotation.w

                # 不进行坐标变化
                # imu_msg.linear_acceleration.x = pose_data.acceleration.x
                # imu_msg.linear_acceleration.y = pose_data.acceleration.y
                # imu_msg.linear_acceleration.z = pose_data.acceleration.z

                # imu_msg.angular_velocity.x = pose_data.angular_velocity.x
                # imu_msg.angular_velocity.y = pose_data.angular_velocity.y
                # imu_msg.angular_velocity.z = pose_data.angular_velocity.z

                # imu_msg.orientation.x = pose_data.rotation.x
                # imu_msg.orientation.y = pose_data.rotation.y
                # imu_msg.orientation.z = pose_data.rotation.z
                # imu_msg.orientation.w = pose_data.rotation.w

                self.imu_pub.publish(imu_msg)
                self.get_logger().debug("IMU数据已发布")
        except Exception as e:
            self.get_logger().error(f"IMU数据发布异常: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = T265ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('T265 IMU 发布方已停止')
    finally:
        node.pipeline.stop()
        if rclpy.ok():  # 检查上下文是否仍处于活动状态，防止多个节点同时关闭rclpy
            rclpy.shutdown()

if __name__ == '__main__':
    main()