import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import colorsys
import math


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        # 创建订阅方，订阅 LaserScan 话题
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )
        self.subscription  # 防止未使用变量警告

        # 创建发布方
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.result_pub = self.create_publisher(String, 'lidar_result', 10)
        
        # 检测参数配置
        self.min_distance = 0.1  # 最小检测距离
        self.max_distance = 3.0  # 最大检测距离
        self.min_angle_deg = -80  # 最小角度
        self.max_angle_deg = 80  # 最大角度
        self.cluster_distance_threshold = 0.2  # 相邻点距离阈值
        self.min_cluster_points = 5  # 最小聚类点数
        self.max_cluster_points = 20000  # 最大聚类点数
        self.frame_id = 'laser_link'  # 雷达坐标系

        self.get_logger().info("激光雷达处理器初始化完成")

    def lidar_callback(self, msg):
        # 将角度范围转换为弧度
        min_angle_rad = np.deg2rad(self.min_angle_deg)
        max_angle_rad = np.deg2rad(self.max_angle_deg)

        # 计算角度
        angles_rad = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment   # 角度数组

        # 数据预处理
        valid_mask = (np.array(msg.ranges) >= self.min_distance) & \
                     (np.array(msg.ranges) <= self.max_distance) & \
                     (angles_rad >= min_angle_rad) & \
                     (angles_rad <= max_angle_rad) & \
                     np.isfinite(np.array(msg.ranges))
        valid_angles = angles_rad[valid_mask]                      # 目标角度范围和距离范围内，各点的角度
        valid_ranges = np.array(msg.ranges)[valid_mask]            # 目标角度范围和距离范围内，各点的距离
        valid_intensities = np.array(msg.intensity)[valid_mask]    # 目标角度范围和距离范围内，各点的强度
        
        if not valid_ranges.size:
            self.get_logger().warning("没有有效距离数据")
            return

        # 转换为笛卡尔坐标
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.column_stack((x, y))

        # 简单聚类处理
        clusters = []
        current_cluster = [points[0]]
        for i in range(1, len(points)):
            prev_point = points[i - 1]
            current_point = points[i]
            distance = np.linalg.norm(current_point - prev_point)
            if distance <= self.cluster_distance_threshold:
                current_cluster.append(current_point)
            else:
                if len(current_cluster) >= self.min_cluster_points:
                    clusters.append(np.array(current_cluster))
                current_cluster = [current_point]

        # 处理最后一个聚类
        if len(current_cluster) >= self.min_cluster_points:
            clusters.append(np.array(current_cluster))

        target_distance = None
        target_angle = None
        marker_array = MarkerArray()
        marker_id = 0

        for cluster_index, cluster in enumerate(clusters):
            cluster_point_count = len(cluster)
            # 打印每个聚类的点数
            # self.get_logger().info(f"聚类 {cluster_index} 的点数: {cluster_point_count}")
            if cluster_point_count > 50 and cluster_point_count > 15:
                continue
            if self.min_cluster_points <= cluster_point_count <= self.max_cluster_points:
                cluster_indices = []
                for point in cluster:
                    index = np.where((points == point).all(axis=1))[0][0]
                    cluster_indices.append(index)

                cluster_ranges = valid_ranges[cluster_indices]
                cluster_angles = valid_angles[cluster_indices]

                avg_distance = np.mean(cluster_ranges)
                if target_distance is None or avg_distance < target_distance:
                    target_distance = avg_distance
                    target_angle = np.mean(cluster_angles)


                #  ======= 计算聚类中心 ========= 
                center = np.mean(cluster, axis=0)


                # 创建 Marker 进行可视化
                marker = self.create_marker(cluster, marker_id)
                marker_array.markers.append(marker)
                marker_id += 1
                # self.get_logger().info(f"障碍物 {marker_id} 的点数: {cluster_point_count}")

        # 发布目标信息
        if target_distance is not None and target_angle is not None:
            target_x = int(target_distance * math.sin(target_angle) * 100)
            target_y = int(target_distance * math.cos(target_angle) * 100)
            target_result = f"@1,{target_distance:.2f},{np.rad2deg(target_angle):.2f},{- target_x},{target_y}\r"

            target_msg = String()
            target_msg.data = target_result
            self.result_pub.publish(target_msg)
            # self.get_logger().info(f"目标结果: {target_result}")

        # 发布 MarkerArray
        self.marker_pub.publish(marker_array)

    def create_marker(self, cluster_points, marker_id):
        """创建可视化 Marker"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacles"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # 计算聚类中心
        center = np.mean(cluster_points, axis=0)
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = 0.05  # 圆柱体高度位置

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 设置尺寸（直径和高度）
        marker.scale.x = 0.1  # 圆柱体直径
        marker.scale.y = 0.1
        marker.scale.z = 0.2  # 圆柱体高度

        # 生成不同颜色
        h = (marker_id * 0.1) % 1.0
        r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.8  # 透明度

        marker.lifetime.sec = 1  # 生存时间 1 秒
        return marker


def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    try:
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


