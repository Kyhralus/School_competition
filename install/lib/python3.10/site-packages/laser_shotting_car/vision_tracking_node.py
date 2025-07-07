import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import time
from typing import Union, Optional
from periphery import PWM  # 导入periphery的PWM库

class VisionTrackingNode(Node):
    """ 视觉检测节点：订阅图像话题，发布String类型误差 """
    def __init__(self):
        super().__init__('vision_tracking_node')
        
        # 1. 初始化OpenCV桥接器
        self.bridge = CvBridge()
        
        # 2. 订阅图像话题
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',  # 订阅相机发布的图像话题
            self.image_callback,
            10
        )
        
        # 3. 发布误差话题（String类型）
        self.error_pub = self.create_publisher(
            String,
            'gimbal_error',
            10
        )
        
        self.get_logger().info("Vision tracking initialized.")
    
    def detect_deepest_inner_circle(self, frame: np.ndarray) -> tuple[np.ndarray, Optional[tuple[int, int]]]:
        """检测层级最多的轮廓结构中最里层的圆心"""
        candidate_frame = frame.copy()
        hier_frame = frame.copy()
        result_frame = frame.copy()
        
        # 图像预处理
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 使用大津法进行阈值处理
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # 形态学操作优化轮廓
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        
        cv2.imshow("Binary", thresh)
        
        # 获取轮廓和层级信息 (RETR_TREE 模式)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if hierarchy is None or len(hierarchy) == 0 or len(contours) == 0:
            cv2.putText(result_frame, "No contours found", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.imshow("Result", result_frame)
            cv2.waitKey(1)
            return result_frame, None
        
        hierarchy = hierarchy[0]  # 获取层级数组
        
        # ===== 步骤1：保留符合条件的父轮廓下的所有子轮廓，并在子轮廓中筛选圆度 =====
        candidate_circles = []
        valid_parents = set()  # 存储符合条件的父轮廓索引

        # 第一遍遍历：找出所有符合条件的父轮廓
        for i in range(len(contours)):
            contour = contours[i]
            area = cv2.contourArea(contour)
            
            # 检查父轮廓是否符合条件
            if area < 200:  # 面积阈值可调整
                continue
                
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
                
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity >= 0.8:  # 圆度阈值可调整
                valid_parents.add(i)

        # 第二遍遍历：收集符合条件的父轮廓下的所有子轮廓，并筛选满足圆度的子轮廓
        for parent_idx in valid_parents:
            # 父轮廓自身不加入候选（仅处理子轮廓）
            # 递归添加所有满足圆度的子轮廓
            def add_valid_child_contours(current_idx):
                child_idx = hierarchy[current_idx][2]  # 第一个子轮廓
                while child_idx != -1:
                    # 计算子轮廓的圆度
                    area = cv2.contourArea(contours[child_idx])
                    perimeter = cv2.arcLength(contours[child_idx], True)
                    if perimeter == 0:
                        child_idx = hierarchy[child_idx][0]  # 跳过无效轮廓
                        continue
                        
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # 仅当子轮廓满足圆度要求时加入候选
                    if area >= 200 and circularity >= 0.89:
                        candidate_circles.append({
                            'index': child_idx,
                            'contour': contours[child_idx],
                            'area': area,
                            'circularity': circularity,
                            'parent_index': parent_idx  # 记录父轮廓索引
                        })
                    
                    # 递归处理当前子轮廓的子轮廓
                    add_valid_child_contours(child_idx)
                    child_idx = hierarchy[child_idx][0]  # 下一个同级轮廓
                    
            add_valid_child_contours(parent_idx)

        # 绘出候选圆
        for circle in candidate_circles:
            cv2.drawContours(candidate_frame, [circle['contour']], -1, (0, 255, 255), 2)
            # 显示圆度和父轮廓索引
            cv2.putText(candidate_frame, f"C: {circle['circularity']:.2f}, P:{circle['parent_index']}", 
                    (circle['contour'][0][0][0], circle['contour'][0][0][1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.putText(candidate_frame, f"Candidates: {len(candidate_circles)}", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("candidate_frame", candidate_frame)
        cv2.waitKey(1)

        if not candidate_circles:
            print("没找到圆度符合要求的圆")

            return result_frame, None

        
        # ===== 步骤2：计算candidate_circles每个轮廓的层级深度 =====
        circle_levels = {}
        for circle in candidate_circles:
            idx = circle['index']
            level = 0
            next_idx = idx
            while hierarchy[next_idx][3] != -1:  # 向上遍历至最外层轮廓
                next_idx = hierarchy[next_idx][3]
                level += 1
            circle_levels[idx] = level
        # print(f"层级轮廓信息{circle_levels}")

        # ===== 步骤3：计算候选轮廓的平均层级，选择层级大于平均层级的轮廓作为inner_circles =====
        if not circle_levels:
            print("没有层级信息")
            return result_frame, None

        # 计算平均层级
        average_level = sum(circle_levels.values()) / len(circle_levels)
        # if average_level <= 2:
        #     average_level = 0.5

        # print(f"平均层级: {average_level:.2f}")

        # 选择层级大于平均层级的轮廓
        inner_circles = [c for c in candidate_circles if circle_levels[c['index']] >= average_level]

        # 如果没有找到，回退到选择最深层级
        if not inner_circles:
            # max_level = max(circle_levels.values())
            # inner_circles = [c for c in candidate_circles if circle_levels[c['index']] == max_level]
            # print(f"没有找到层级大于平均值的轮廓，回退到最深层级: {max_level}")
            print(f"没有找到层级大于平均值的轮廓")

        if not inner_circles:
            print("没有找到符合条件的内层轮廓")

        # ===== 可视化层级结果 =====
        # 定义层级颜色映射
        color_map = [
            (0, 0, 255),    # 红色 - 层级0 (最外层)
            (0, 255, 0),    # 绿色 - 层级1
            (255, 0, 0),    # 蓝色 - 层级2
            (0, 255, 255),  # 黄色 - 层级3
            (255, 0, 255),  # 紫色 - 层级4
            (255, 255, 0),  # 青色 - 层级5
            (128, 0, 128),  # 深紫 - 层级6
            (0, 128, 128),  # 深绿 - 层级7
        ]

        # 绘制所有候选圆，按层级着色
        for circle in candidate_circles:
            level = circle_levels[circle['index']]
            color = color_map[level % len(color_map)]
            cv2.drawContours(hier_frame, [circle['contour']], -1, color, 2)
            
            # 在轮廓上标注层级信息
            M = cv2.moments(circle['contour'])
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.putText(hier_frame, f"L{level}", (cX, cY),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 用特殊颜色高亮inner_circles
        for circle in inner_circles:
            cv2.drawContours(hier_frame, [circle['contour']], -1, (255, 255, 255), 3)

        # 显示统计信息
        cv2.putText(hier_frame, f"Avg Level: {average_level:.2f}, Inner Circles: {len(inner_circles)}", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("hier_frame", hier_frame)
        cv2.waitKey(1)

        # ===== 步骤4：在inner_circles选取面积最小的轮廓作为靶心 =====
        if not inner_circles:
            print("No inner circles found")
            # 处理无符合条件的情况（可选）
            return result_frame, None

        # 按面积排序，选择最小的
        inner_circles.sort(key=lambda x: x['area'])
        target_circle = inner_circles[0]

        # 计算圆心坐标
        M = cv2.moments(target_circle['contour'])
        if M["m00"] == 0:  # 防止除零错误
            print("Failed to compute center")
            return result_frame, None

        target_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        # 可视化靶心
        cv2.circle(result_frame, target_center, 5, (0, 0, 255), -1)  # 红色实心圆标记圆心
        cv2.putText(result_frame, f"Target: ({target_center[0]}, {target_center[1]})",
                (target_center[0] - 100, target_center[1] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow("Result", result_frame)
        cv2.waitKey(1)
        # return result_frame, (cX, cY)  # 第二步暂不返回圆心，仅展示层级分析结果
        return result_frame, target_center  # 第二步暂不返回圆心，仅展示层级分析结果
    
    def detect_red_laser(
        self,
        frame: np.ndarray, 
        min_area: int = 100,  # 对应截图里 Min Area (0100/5000)，这里用 100 ，可按需改
        max_area: int = 400,  # 对应截图里 Max Area (05000/50000)，这里用 5000 ，可按需改
        erode_iter: int = 0,   # 对应截图里 Erode (00/10)，先设 0
        dilate_iter: int = 5   # 对应截图里 Dilate (05/10)，先设 5
    ) -> tuple[Optional[tuple[int, int]], np.ndarray]:
        """
        从输入图像中检测红色激光点，使用截图所示阈值。
        
        :param frame: 输入的BGR格式图像
        :param min_area: 最小轮廓面积阈值，对应界面 Min Area
        :param max_area: 最大轮廓面积阈值，对应界面 Max Area
        :param erode_iter: 腐蚀操作迭代次数，对应界面 Erode
        :param dilate_iter: 膨胀操作迭代次数，对应界面 Dilate
        :return: 激光点的(x, y)坐标，如果未检测到则为None, 处理后的图像
        """
        result_frame = frame.copy()
        # 用截图里的阈值设置 HSV 上下限
        # H_min (000/179), H_max (179/179)
        # S_min (000/255), S_max (255/255)
        # V_min (255/255), V_max (255/255)
        lower_hsv = np.array([0, 0, 255])  
        upper_hsv = np.array([179, 255, 255])
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # 形态学操作，根据界面滑块设置迭代次数
        kernel = np.ones((3, 3), np.uint8)  # 核大小可根据需求调整，这里先保持 1x1
        mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel, iterations=erode_iter)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=dilate_iter)
        
        cv2.imshow("laser_mask", mask)
        cv2.waitKey(1)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        laser_point = None
        laser_area = 0

        # 筛选符合条件的轮廓
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue
            print("激光面积：",area)
            # 计算轮廓中心
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                
                # 如果激光点已被检测到，选择面积较大的
                if laser_point is None or area > laser_area:
                    laser_point = (x, y)
                    laser_area = area

        # 标记激光点
        if laser_point:
            cv2.circle(result_frame, laser_point, 5, (0, 255, 0), -1)
            cv2.putText(result_frame, f"Laser: ({laser_point[0]}, {laser_point[1]})", 
                    (laser_point[0] + 10, laser_point[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("laser_result", result_frame)
        cv2.waitKey(1)
        
        return laser_point, result_frame
    
    def image_callback(self, msg: Image) -> None:
        """ 处理接收到的图像消息 """
        try:
            # 转换ROS图像消息为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 检测靶心和激光
            _, target_point = self.detect_deepest_inner_circle(frame)
            laser_point, _ = self.detect_red_laser(frame)
            
            # 计算误差并发布
            if target_point and laser_point:
                target_x, target_y = target_point
                laser_x, laser_y = laser_point
                
                # 归一化误差
                width, height = frame.shape[1], frame.shape[0]
                err_pitch = (laser_y - target_y) / height * 10  # pitch控制垂直方向
                err_roll = (laser_x - target_x) / width * 10    # roll控制水平方向
                
                # 发布误差消息（格式："pitch_error,roll_error"）
                msg = String()
                msg.data = f"{err_pitch:.6f},{err_roll:.6f}"  # 保留6位小数
                self.error_pub.publish(msg)
                
                # 可视化
                cv2.circle(frame, laser_point, 2, (0, 255, 0), -1)
                cv2.circle(frame, target_point, 2, (0, 255, 255), -1)
                cv2.putText(frame, f"Error: ({err_pitch:.2f}, {err_roll:.2f})", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示结果
            cv2.imshow("tracking", frame)
            cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def close(self) -> None:
        """ 释放资源 """
        cv2.destroyAllWindows()
        self.get_logger().info("Resources released.")


def main(args=None):
    rclpy.init(args=args)
    node = VisionTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()