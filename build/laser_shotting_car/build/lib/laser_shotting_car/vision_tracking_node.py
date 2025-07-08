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
            # print("没找到圆度符合要求的圆")

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
        min_area: int = 50,
        max_area: int = 1500,
        erode_iter: int = 1,
        dilate_iter: int = 2
    ) -> tuple[Optional[tuple[int, int]], np.ndarray]:
        """优先选择图像中间区域的激光点，兼顾面积和中心距离"""
        result_frame = frame.copy()
        h, w = frame.shape[:2]  # 获取图像高度和宽度
        center = (w // 2, h // 2)  # 图像中心点坐标（中间区域基准点）
        
        # HSV阈值和形态学操作（保持不变）
        lower_hsv = np.array([0, 0, 255])
        upper_hsv = np.array([179, 255, 255])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel, iterations=erode_iter)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=dilate_iter)
        cv2.imshow("laser_mask", mask)
        cv2.waitKey(1)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        laser_point = None
        best_score = -1  # 用于筛选最优激光点的分数（越高越好）

        # 筛选符合条件的轮廓（增加中心距离权重）
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 1. 面积过滤（基础条件）
            if area < min_area or area > max_area:
                continue
            
            # 2. 计算轮廓中心
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            
            # 3. 计算距离图像中心的距离（归一化到0~1，值越小越近）
            distance = np.sqrt((x - center[0])**2 + (y - center[1])** 2)
            normalized_distance = distance / np.sqrt((w//2)**2 + (h//2)** 2)  # 归一化到[0,1]
            
            # 4. 计算评分（面积越大、距离中心越近，评分越高）
            # 权重可调整：例如面积权重0.6，距离权重0.4
            area_score = area / max_area  # 面积归一化到[0,1]
            distance_score = 1 - normalized_distance  # 距离越近，分数越高（0~1）
            score = 0.6 * area_score + 0.4 * distance_score  # 综合评分
            
            # 5. 更新最优激光点（评分更高的优先）
            if score > best_score:
                best_score = score
                laser_point = (x, y)
                # 可选：打印调试信息
                # print(f"激光候选 - 面积: {area:.1f}, 距离中心: {distance:.1f}, 评分: {score:.3f}")

        # 标记最优激光点
        if laser_point:
            cv2.circle(result_frame, laser_point, 5, (0, 255, 0), -1)
            cv2.putText(result_frame, f"Laser: ({laser_point[0]}, {laser_point[1]})", 
                    (laser_point[0] + 10, laser_point[1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # 标记图像中心（方便调试）
            cv2.circle(result_frame, center, 3, (255, 0, 0), -1)
            cv2.putText(result_frame, "Center", (center[0]+10, center[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

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
            
            # 检测靶心
            circle_frame, target_point = self.detect_deepest_inner_circle(frame)
            
            # 检测激光点
            laser_point, laser_frame =  self.detect_red_laser(frame)
            
            # 计算误差并控制云台
            if target_point and laser_point:
                target_x, target_y = target_point
                laser_x, laser_y = laser_point
                
                # 计算水平和垂直误差（调整归一化系数）
                '''
                在视觉跟踪任务中，图像的宽度和高度可能不同。
                若直接使用像素坐标计算误差，不同方向（水平和垂直）的误差范围会因图像尺寸而异。
                归一化能把不同维度的误差统一到相同尺度，让 PID 控制器在不同方向上有一致的响应。
                '''
                # err_x = (laser_x - target_x) / width * 2
                # err_y = (laser_y - target_y) / height * 2
                err_x = (laser_x - target_x) 
                err_y = (laser_y - target_y) 
                
                # 画图
                cv2.circle(frame, laser_point, 2, (0, 255, 0), -1) # 激光点
                cv2.circle(frame, target_point, 2, (0, 255, 255), -1)
                
                # 显示误差信息
                cv2.putText(frame, f"Error: ({err_x:.2f}, {err_y:.2f})", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                print(f"靶心:({target_x},{target_y}), 激光:({laser_y},{laser_y})")
                print(f"误差:x_err={err_x},y_err={err_y}")
                # 更新云台控制
                if abs(err_x) <= 5 and abs(err_y) <=5:
                    print("打中靶心")
                # 发布误差消息（格式："pitch_error,yaw_error"）
                msg = String()
                msg.data = f"{err_x:.6f},{err_y:.6f}"  # 保留6位小数
                self.error_pub.publish(msg)

            elif not target_point and laser_point:
                # err_x = 0
                # err_y = 0
                print("未检测到靶心")
            elif target_point and not laser_point:
                # err_x = 0
                # err_y = 0
                print("未检测到激光")

            # 显示FPS
            fps = 1 / (time.time() - start_time) if 'start_time' in locals() else 0
            cv2.putText(frame, f"FPS: {int(fps)}", 
                      (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            start_time = time.time()

            # 显示结果
            cv2.imshow("tracking", frame)
        
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