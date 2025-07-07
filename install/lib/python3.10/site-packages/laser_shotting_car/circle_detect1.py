import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
import time


class CircleLaserDetector(Node):
    def __init__(self):
        super().__init__('circle_laser_detector')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            2
        )
        self.publisher_ = self.create_publisher(String, 'circle_laser_result', 2)
        self.bridge = CvBridge()

        # 激光阈值
        self.lower_red1 = np.array([32, 226, 64])
        self.upper_red1 = np.array([141, 55, 137])
        self.lower_red2 = np.array([155, 25, 115])
        self.upper_red2 = np.array([180, 255, 255])

        # 创建定时器，定期发布处理数据
        self.timer = self.create_timer(0.1, self.process_camera_data)   # 100Hz
        self.get_logger().info("任务四激光打靶节点启动")

        self.last_laser_x = None
        self.last_laser_y = None


    def image_callback(self, msg):
        self.current_image = msg

    def process_camera_data(self):
        try:
            if not hasattr(self, 'current_image'):
                return
            img_raw = self.bridge.imgmsg_to_cv2(self.current_image, 'bgr8')

            inner_center, _ = self.detect_circles_from_image(
                img_raw,
                # min_radius=5,
                # max_radius=140,
                # canny_thresh1=15,
                # canny_thresh2=50,
                # area_thresh=30
            )
            # 检查 inner_center 是否为 None
            if inner_center is not None:
                cX, cY = inner_center
            else:
                cX, cY = None, None
            laser_x, laser_y = self.detect_red_laser(
                img_raw,
                self.lower_red1,
                self.upper_red1,
                self.lower_red2,
                self.upper_red2
            )

            if cX is not None and cY is not None and laser_x is not None and laser_y is not None:                
                # self.get_logger().info(f"圆: ({cX}, {cY}),激光: ({laser_x}, {laser_y})")
                # 处理误差数据
                error_msg = String() # 创建 String 数据
                # 计算误差
                error_x = laser_x - cX
                error_y = laser_y - cY
                error_msg.data = f"@4,{int(error_x)},{int(error_y)}\r"
                # 发布误差数据
                self.publisher_.publish(error_msg)   # 发布误差数据

                # 画图
                cv2.circle(img_raw, (cX,cY), 2, (255, 0, 0), -1)   # 画出靶心 --- 红色
                cv2.circle(img_raw, (laser_x, laser_y), 2, (0, 255, 255), -1)  # 画出激光 --- 青色
                cv2.putText(img_raw, f"Error:({error_x},{error_y})", (cX - 150, cY + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2) # 显示误差
                cv2.imshow("Detected Circles", img_raw)
                cv2.waitKey(1)

            else:
                # 处理误差数据
                error_msg = String() # 创建 String 数据
                # 计算误差
                error_x = 0
                error_y = 0
                error_msg.data = f"@4,{int(error_x)},{int(error_y)}\r"
                # 发布误差数据
                self.publisher_.publish(error_msg)   # 发布误差数据
                # self.get_logger().error("未检测到圆或激光")

        except KeyboardInterrupt:
                 # 处理误差数据
                error_msg = String() # 创建 String 数据
                # 计算误差
                error_x = 0
                error_y = 0
                error_msg.data = f"@4,{int(error_x)},{int(error_y)}\r"
                # 发布误差数据
                self.publisher_.publish(error_msg)   # 发布误差数据
        
        # except Exception as e:
        #     self.get_logger().error(f"任务三图像异常: {str(e)}")

    def detect_circles_from_image(self, frame, min_radius=20, max_radius=400, canny_thresh1=20, canny_thresh2=50, area_thresh=100):
        """检测图像中的圆形目标并返回内圆圆心"""
        result_frame = frame
        inner_center = None

        # 图像预处理
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, 16, 90)  # 根据需求调整的灰度范围
        # cv2.imshow("mask", mask)

        # 形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask_dilate = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
        # cv2.imshow("mask_dilate", mask_dilate)

        # kernel = np.ones((3, 3), np.uint8)
        mask_morph = cv2.morphologyEx(mask_dilate, cv2.MORPH_CLOSE, kernel)  # 闭运算填充小孔洞
        mask_morph = cv2.morphologyEx(mask_morph, cv2.MORPH_OPEN, kernel)  # 开运算去除小噪点
        # cv2.imshow("mask_morph", mask_morph)

        contours, hierarchy = cv2.findContours(mask_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # ========= 分析层级结构，找出最深层级
        max_level = 0
        contour_levels = []  # 存储每个轮廓的层级

        for i in range(len(contours)):
            level = 0
            next_idx = i
            while hierarchy is not None and hierarchy[0][next_idx][3] != -1:
                next_idx = hierarchy[0][next_idx][3]
                level += 1
            contour_levels.append(level)
            max_level = max(max_level, level)

        # ========= 筛选最深层级中面积最小的轮廓
        deepest_contours = [i for i, level in enumerate(contour_levels) if level == max_level-2 or level == max_level]
        # min_area = float('inf')
        min_area = float('inf')
        smallest_contour_idx = -1

        for i in deepest_contours:
            area = cv2.contourArea(contours[i])
            if area < min_area and area > 100:  # 忽略过小的噪声轮廓
                min_area = area
                smallest_contour_idx = i

        # ========= 绘制结果
        # result_frame = frame.copy()

        # 绘制所有轮廓（可选）
        # color_palette = [(0, 255, 0), (0, 255, 255), (255, 255, 0)]
        # for i in range(len(contours)):
        #     color = color_palette[contour_levels[i] % len(color_palette)]
        #     cv2.drawContours(result_frame, contours, i, color, 1)

        # 标记选中的最小轮廓及其圆心
        if smallest_contour_idx != -1:
            # 绘制轮廓
            # cv2.drawContours(result_frame, contours, smallest_contour_idx, (0, 0, 255), 2)
            
            # 计算圆心
            M = cv2.moments(contours[smallest_contour_idx])
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                inner_center = cX, cY
                # 绘制圆心
                # cv2.circle(result_frame, (cX, cY), 1, (255, 0, 0), -1)
                
                # 添加标签
                # cv2.putText(result_frame, f"Inner Center: ({cX}, {cY})", 
                        # (cX - 40, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                # cv2.putText(result_frame, f"Area: {min_area:.1f}", 
                        # (cX - 30, cY - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
                # 打印信息
                # print(f"最内层最小轮廓圆心坐标: ({cX}, {cY})")
                # print(f"轮廓面积: {min_area:.1f}")
                # print(f"轮廓层级: {max_level}")

        # cv2.imshow("Smallest Inner Contour", result_frame)
        # cv2.waitKey(1)

        return inner_center, result_frame

# ////// 可以检测到最小轮廓
    # def detect_circles_from_image(self, image, min_radius=5, max_radius=400, canny_thresh1=15, canny_thresh2=50, area_thresh=30):
    #     if image is None:
    #         self.get_logger().error("无法获取图像")
    #         return [], image
        
    #     # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #     # mask = cv2.inRange(gray, 16, 90)
    #       # 增强的图像预处理流程
    #     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #     mask = cv2.inRange(gray, 16, 90)

    #     # 形态学操作去除噪声
    #     kernel = np.ones((5, 5), np.uint8)
    #     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    #     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #     # 找轮廓
    #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
    #     # Canny边缘检测，增强轮廓
    #     edges = cv2.Canny(mask, canny_thresh1, canny_thresh2)
        
    #     # 再次进行形态学膨胀，连接断裂的边缘
    #     dilated = cv2.dilate(edges, kernel, iterations=1)
        
    #     # 显示预处理结果
    #     cv2.imshow("Original", image)
    #     # cv2.imshow("Threshold", thresh)
    #     # cv2.imshow("Morphology", opened)
    #     cv2.imshow("Canny Edges", edges)
    #     cv2.imshow("Dilated Edges", dilated)
    #     cv2.waitKey(1)


    #     # 找轮廓，使用 RETR_TREE 模式获取层级结构
    #     contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #     # 复制原图像，避免修改原始图像
    #     contour_image = image.copy()
    #     circle_image = image.copy()  # 修正拼写错误

    #     # 定义不同层次轮廓的颜色
    #     colors = [
    #         (0, 0, 255),    # 红色
    #         (0, 255, 255),  # 黄色
    #         (255, 0, 0),    # 蓝色
    #         (0, 255, 0),    # 绿色
    #         (255, 255, 0),  # 青色
    #         (255, 0, 255),  # 品红色
    #         (255, 255, 255) # 白色
    #     ]

    #     circles = []
    #     if contours:
    #         # 找出最大层级
    #         max_level = 0
    #         max_level_contours = []
    #         for i in range(len(contours)):
    #             # 获取当前轮廓的层级信息
    #             current_hierarchy = hierarchy[0][i]
    #             level = 0
    #             next_index = i
    #             while current_hierarchy[3] != -1:  # 向上遍历层级直到最外层
    #                 level += 1
    #                 next_index = current_hierarchy[3]
    #                 current_hierarchy = hierarchy[0][next_index]
    #             if level > max_level:
    #                 max_level = level
    #                 max_level_contours = [i]
    #             elif level == max_level:
    #                 max_level_contours.append(i)

    #         # 画多级轮廓
    #         for i in range(len(contours)):
    #             # 获取当前轮廓的层级信息
    #             current_hierarchy = hierarchy[0][i]
    #             level = 0
    #             next_index = i
    #             while current_hierarchy[3] != -1:  # 向上遍历层级直到最外层
    #                 level += 1
    #                 next_index = current_hierarchy[3]
    #                 current_hierarchy = hierarchy[0][next_index]

    #             if level == max_level:
    #                 # 最大层级的轮廓，每一级用不同颜色
    #                 color = colors[level % len(colors)]
    #             else:
    #                 # 其他等级较少的轮廓统一用绿色
    #                 color = (0, 255, 0)
    #             # 绘制轮廓
    #             cv2.drawContours(contour_image, contours, i, color, 2)
    #         # 创建新窗口显示多轮廓结果
    #         cv2.imshow("multi contours", contour_image)
    #         cv2.waitKey(1)  # 等待 1 毫秒，刷新窗口

    #         # ======== 圆拟合逻辑 ========
    #         for cnt in contours:
    #             area = cv2.contourArea(cnt)
    #             if area < area_thresh:
    #                 continue
    #             (x, y), radius = cv2.minEnclosingCircle(cnt)
    #             if min_radius < radius < max_radius:
    #                 # 圆度判据（实际面积/理论圆面积）
    #                 circle_area = np.pi * radius * radius
    #                 if area / circle_area > 0.4:  # 圆度阈值
    #                     circles.append((int(x), int(y), int(radius)))

    #         # 按半径从内到外排序
    #         circles.sort(key=lambda circle: circle[2])
    #         # 绘制排序后的圆
    #         for index, circle in enumerate(circles):
    #             x, y, radius = circle
    #             color = colors[index % len(colors)]
    #             # 绘制检测到的圆
    #             cv2.circle(circle_image, (x, y), radius, color, 2)
    #             # 绘制圆心
    #             cv2.circle(circle_image, (x, y), 1, (0, 0, 255), -1)

    #         cv2.imshow("circle result", circle_image)
    #         cv2.waitKey(1)  # 等待 1 毫秒，刷新窗口

    #         #  ========= 合并半径相近的圆 ======
    #         merged_circles = []
    #         i = 0
    #         while i < len(circles):
    #             current_circle = circles[i]
    #             j = i + 1
    #             merged_radius_sum = current_circle[2]
    #             merged_x_sum = current_circle[0]
    #             merged_y_sum = current_circle[1]
    #             count = 1
    #             while j < len(circles) and abs(circles[j][2] - current_circle[2]) <= 20:
    #                 merged_radius_sum += circles[j][2]
    #                 merged_x_sum += circles[j][0]
    #                 merged_y_sum += circles[j][1]
    #                 count += 1
    #                 j += 1
    #             # 计算合并后的圆心和半径
    #             merged_x = int(merged_x_sum / count)
    #             merged_y = int(merged_y_sum / count)
    #             merged_radius = int(merged_radius_sum / count)
    #             merged_circles.append((merged_x, merged_y, merged_radius))
    #             i = j
    #         circles = merged_circles



    #         # 绘制排序后的圆
    #         for index, circle in enumerate(circles):
    #             x, y, radius = circle
    #             color = colors[index % len(colors)]
    #             # 绘制检测到的圆
    #             cv2.circle(circle_image, (x, y), radius, color, 2)
    #             # 绘制圆心
    #             cv2.circle(circle_image, (x, y), 1, (0, 0, 255), -1)

    #         cv2.imshow("circle result", circle_image)
    #         cv2.waitKey(1)  # 等待 1 毫秒，刷新窗口

    #     # 若检测到圆，返回第一个圆的圆心坐标，否则返回 None, None
    #     if circles:
    #         return circles[0][0], circles[0][1]
    #     return None, None

    
    def detect_red_laser(self, frame, lower_red1, upper_red1, lower_red2, upper_red2):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=3)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        laser_x, laser_y = None, None

        if contours:
            merged_contours = []
            for cnt in contours:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    x = int(M["m10"] / M["m00"])
                    y = int(M["m01"] / M["m00"])
                    merged = False
                    for i, merged_cnt in enumerate(merged_contours):
                        M_merged = cv2.moments(merged_cnt)
                        if M_merged["m00"] != 0:
                            x_merged = int(M_merged["m10"] / M_merged["m00"])
                            y_merged = int(M_merged["m01"] / M_merged["m00"])
                            dist = np.sqrt((x - x_merged) ** 2 + (y - y_merged) ** 2)
                            if dist < 20:
                                merged_contours[i] = np.concatenate((merged_cnt, cnt))
                                merged = True
                                break
                    if not merged:
                        merged_contours.append(cnt)
            contours = merged_contours

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 5:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        laser_x = int(M["m10"] / M["m00"])
                        laser_y = int(M["m01"] / M["m00"])
                        cv2.circle(frame, (laser_x, laser_y), 5, (0, 255, 255), -1)
        else:
            # self.get_logger().info("未检测到红色激光轮廓")
            pass
        return laser_x, laser_y




def main(args=None):
    rclpy.init(args=args)
    circle_laser_detector = CircleLaserDetector()

    try:
        rclpy.spin(circle_laser_detector)
    except KeyboardInterrupt:

        pass
    finally:
        cv2.destroyAllWindows()
        circle_laser_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()