# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from std_msgs.msg import String
# import cv2
# import numpy as np
# # from rknnpool import rknnPoolExecutor
# # from func import myFunc
# # 相对导入
# from .rknnpool import rknnPoolExecutor
# from .func import myFunc

# class Yolov8DetectorNode(Node):
#     def __init__(self):
#         super().__init__('yolov8_detector_node')
#         self.bridge = CvBridge()
        
#         # 订阅图像话题
#         self.subscription = self.create_subscription(
#             Image,
#             'image_raw',
#             self.image_callback,
#             10
#         )
        
#         # 发布识别结果
#         self.result_pub = self.create_publisher(String, 'yolov8_result', 10)
        
#         # 初始化rknn池
#         self.model_path = "/home/orangepi/ros2_workspace/school_competition/src/laser_shotting_car/laser_shotting_car/models/v3.rknn"
#         self.thread_num = 5
#         self.pool = rknnPoolExecutor(
#             rknnModel=self.model_path,
#             TPEs=self.thread_num,
#             func=myFunc
#         )
        
#         self.get_logger().info("Yolov8检测器节点已启动")

#     def image_callback(self, msg):
#         try:
#             # 将ROS图像消息转换为OpenCV图像
#             cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
#             # 放入图像进行推理
#             self.pool.put(cv_image)
            
#             # 获取处理结果
#             result, success = self.pool.get()
            
#             if success:
#                 class_names, confidences = result
#                 if class_names and confidences:
#                     # 构建结果消息
#                     result_msg = String()
#                     result_str = ""
                    
#                     for cls, conf in zip(class_names, confidences):
#                         result_str += f"{cls}:{conf:.2f},"
#                         self.get_logger().info(f"检测到: {cls}, 置信度: {conf:.2f}")
                    
#                     # 发布结果
#                     result_msg.data = result_str.rstrip(',')  # 移除最后的逗号
#                     self.result_pub.publish(result_msg)
#                 else:
#                     self.get_logger().info("未检测到目标")
#             else:
#                 self.get_logger().info("获取处理结果失败")
#                 # 显示处理后的帧
#             cv2.imshow('yolov8', cv_image)
#             cv2.waitKey(1)
                
#         except Exception as e:
#             self.get_logger().error(f"图像处理异常: {str(e)}")

#     def destroy_node(self):
#         self.pool.release()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = Yolov8DetectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
# 相对导入
from .rknnpool import rknnPoolExecutor
from .func import myFunc

class Yolov8DetectorNode(Node):
    def __init__(self):
        super().__init__('yolov8_detector_node')
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        
        # 发布识别结果
        self.result_pub = self.create_publisher(String, 'yolov8_result', 10)
        
        # 初始化rknn池
        self.model_path = "/home/orangepi/ros2_workspace/school_competition/src/laser_shotting_car/laser_shotting_car/models/v10.rknn"
        self.thread_num = 5
        self.pool = rknnPoolExecutor(
            rknnModel=self.model_path,
            TPEs=self.thread_num,
            func=myFunc
        )
        # 初始化识别计数字典
        self.detection_counts = {}
        # 设定触发发布的识别次数阈值
        self.threshold = 10
        
        self.get_logger().info("Yolov8检测器节点已启动")

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 放入图像进行推理
            self.pool.put(cv_image)
            
            # 获取处理结果
            result, success = self.pool.get()
            
            if success:
                class_names, confidences = result
                if class_names and confidences:  # 置信度 大于0.63
                    for cls, conf in zip(class_names, confidences):
                        # 更新识别计数
                        self.detection_counts[cls] = self.detection_counts.get(cls, 0) + 1
                        # self.get_logger().info(f"检测到: {cls}, 置信度: {conf:.2f}, 累计次数: {self.detection_counts[cls]}")
                        # 检查是否达到阈值
                        if self.detection_counts[cls] >= self.threshold:
                            # 构建结果消息
                            result_msg = String()
                            result_msg.data = f"@5,{cls}\r"
                            self.result_pub.publish(result_msg)
                            # self.get_logger().info(f"达到阈值，发布结果: {result_msg.data}")
                            # 重置计数
                            self.detection_counts[cls] = 0
                else:
                    # self.get_logger().info("未检测到目标")
                    pass
            else:
                # self.get_logger().info("获取处理结果失败")
                pass
            # 显示处理后的帧
            cv2.imshow('yolov8', cv_image)
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"图像处理异常: {str(e)}")

    def destroy_node(self):
        self.pool.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Yolov8DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




# @TODO
# 生命周期
# import rclpy
# from rclpy.lifecycle import LifecycleNode
# from rclpy.lifecycle import TransitionCallbackReturn
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from std_msgs.msg import String
# import cv2
# import numpy as np
# # 相对导入
# from .rknnpool import rknnPoolExecutor
# from .func import myFunc

# class Yolov8DetectorNode(LifecycleNode):
#     def __init__(self):
#         super().__init__('yolov8_detector_node')
#         self.bridge = CvBridge()
#         self.model_path = "/home/orangepi/ros2_workspace/school_competition/src/laser_shotting_car/laser_shotting_car/models/v3.rknn"
#         self.thread_num = 5
#         self.pool = None
#         self.detection_counts = {}
#         self.threshold = 10
#         self.subscription = None
#         self.result_pub = None

#     def on_configure(self, state):
#         self.pool = rknnPoolExecutor(
#             rknnModel=self.model_path,
#             TPEs=self.thread_num,
#             func=myFunc
#         )
#         self.subscription = self.create_subscription(
#             Image,
#             'image_raw',
#             self.image_callback,
#             10
#         )
#         self.result_pub = self.create_publisher(String, 'yolov8_result', 10)
#         self.get_logger().info("Yolov8 检测器节点已配置")
#         return TransitionCallbackReturn.SUCCESS

#     def on_activate(self, state):
#         self.get_logger().info("Yolov8 检测器节点已激活")
#         return TransitionCallbackReturn.SUCCESS

#     def on_deactivate(self, state):
#         self.get_logger().info("Yolov8 检测器节点已停用")
#         return TransitionCallbackReturn.SUCCESS

#     def on_cleanup(self, state):
#         if self.pool:
#             self.pool.release()
#         if self.subscription:
#             self.destroy_subscription(self.subscription)
#         if self.result_pub:
#             self.destroy_publisher(self.result_pub)
#         self.get_logger().info("Yolov8 检测器节点已清理")
#         return TransitionCallbackReturn.SUCCESS

#     def on_shutdown(self, state):
#         self.on_cleanup(state)
#         self.get_logger().info("Yolov8 检测器节点已关闭")
#         return TransitionCallbackReturn.SUCCESS

#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#             self.pool.put(cv_image)
#             result, success = self.pool.get()
#             if success:
#                 class_names, confidences = result
#                 if class_names and confidences:
#                     for cls, conf in zip(class_names, confidences):
#                         self.detection_counts[cls] = self.detection_counts.get(cls, 0) + 1
#                         self.get_logger().info(f"检测到: {cls}, 置信度: {conf:.2f}, 累计次数: {self.detection_counts[cls]}")
#                         if self.detection_counts[cls] >= self.threshold:
#                             result_msg = String()
#                             result_msg.data = f"{cls}:{conf:.2f}"
#                             self.result_pub.publish(result_msg)
#                             self.get_logger().info(f"达到阈值，发布结果: {result_msg.data}")
#                             self.detection_counts[cls] = 0
#                 else:
#                     self.get_logger().info("未检测到目标")
#             else:
#                 self.get_logger().info("获取处理结果失败")
#             cv2.imshow('yolov8', cv_image)
#             cv2.waitKey(1)
#         except Exception as e:
#             self.get_logger().error(f"图像处理异常: {str(e)}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = Yolov8DetectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

