import cv2
import numpy as np
import serial  # 导入串口库
import time

def detect_circles_from_image(image, min_radius=20, max_radius=400, canny_thresh1=20, canny_thresh2=50, area_thresh=100):
    """
    从输入图像中检测圆形。

    :param image: 输入的 BGR 格式图像
    :param min_radius: 最小圆半径
    :param max_radius: 最大圆半径
    :param canny_thresh1: Canny 边缘检测的第一个阈值
    :param canny_thresh2: Canny 边缘检测的第二个阈值
    :param area_thresh: 轮廓面积阈值
    :return: 检测到的圆列表和绘制后的图像
    """
    if image is None:
        print("无法获取图像")
        return [], image
    # 存储上一次检测到的圆
    last_circle = None
    # 距离阈值，可根据实际情况调整
    distance_threshold = 50  
    # 将图像转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 对灰度图进行高斯模糊
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    # 使用 Canny 算法进行边缘检测
    edges = cv2.Canny(blurred, canny_thresh1, canny_thresh2)

    # 轮廓检测
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    circles = []
    out_img = image.copy()  # 复制图像用于绘制
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < area_thresh:
            continue
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if min_radius < radius < max_radius:
            # 圆度判据（可选）：实际面积/理论圆面积
            circle_area = np.pi * radius * radius
            if area / circle_area > 0.6:  # 圆度阈值，可调整
                new_circle = (int(x), int(y), int(radius))
                if last_circle is not None:
                    last_x, last_y, _ = last_circle
                    new_x, new_y, _ = new_circle
                    # 计算新旧圆心之间的距离
                    distance = np.sqrt((new_x - last_x) ** 2 + (new_y - last_y) ** 2)
                    if distance > distance_threshold:
                        # 若距离超过阈值，使用上一次的圆心
                        new_circle = (last_x, last_y, new_circle[2])
                circles.append(new_circle)
                last_circle = new_circle
                # debug
                print(f"圆心：({x:.2f}, {y:.2f}) 半径：{radius:.2f}")
                # 绘制检测到的圆
                cv2.circle(out_img, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                # 绘制圆心
                cv2.circle(out_img, (int(x), int(y)), 2, (0, 0, 255), -1)
    return circles, out_img


def detect_red_laser(frame, circles, lower_red1, upper_red1, lower_red2, upper_red2):
    """
    从输入图像中检测圆形内的红色激光点。

    :param frame: 输入的 BGR 格式图像
    :param circles: 检测到的圆列表
    :param lower_red1: 红色 HSV 范围的第一个下限
    :param upper_red1: 红色 HSV 范围的第一个上限
    :param lower_red2: 红色 HSV 范围的第二个下限
    :param upper_red2: 红色 HSV 范围的第二个上限
    :return: 激光点的 x, y 坐标，如果未检测到则为 None
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    kernel = np.ones((1, 1), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)
    cv2.imshow("laser", mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    laser_x, laser_y = None, None
    circles_contours = []  # 存储圆内的轮廓

    if contours and circles:
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                for circle in circles:
                    circle_x, circle_y, circle_radius = circle
                    distance = np.sqrt((x - circle_x) ** 2 + (y - circle_y) ** 2)
                    if distance <= circle_radius:
                        circles_contours.append(cnt)
                        break

        if circles_contours:
            # 找到面积最大的轮廓
            max_contour = max(circles_contours, key=cv2.contourArea)
            M = cv2.moments(max_contour)
            if M["m00"] != 0:
                laser_x = int(M["m10"] / M["m00"])
                laser_y = int(M["m01"] / M["m00"])
                cv2.circle(frame, (laser_x, laser_y), 2, (255, 0, 0), -1)

    if not circles:
        print("未检测到圆，无法限定激光检测范围")
    elif not contours:
        print("未检测到红色激光轮廓")
    return laser_x, laser_y

if __name__ == "__main__":
    lower_red1 = np.array([32, 226, 64])
    upper_red1 = np.array([141, 55, 137])
    lower_red2 = np.array([155, 25, 115])
    upper_red2 = np.array([180, 255, 255])
    try:
        ser = serial.Serial('/dev/ttyS1', 115200, timeout=1)
        print("已启动串口")
    except serial.SerialException as e:
        print(f"串口初始化失败: {e}")
        exit()

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("无法打开摄像头")
        ser.close()
        exit()

    last_send_time = time.time()
    send_interval = 1 / 50

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取摄像头画面")
                break
            circles, out_img = detect_circles_from_image(
                frame,
                min_radius=15,
                max_radius=400,
                canny_thresh1=15,
                canny_thresh2=50,
                area_thresh=100
            )

            laser_x, laser_y = detect_red_laser(frame, circles, lower_red1, upper_red1, lower_red2, upper_red2)

            if circles:
                for i, (x, y, r) in enumerate(circles):
                    error2center_x = 320 - x
                    error2center_y = 240 - y
                    if laser_x is not None and laser_y is not None:
                        error_x = laser_x - x
                        error_y = laser_y - y
                        start_string = f"@4,{int(error_x)},{int(error_y)}\r"
                    else:
                        start_string = f"@4,0,0\r"
                    ser.write(start_string.encode('utf-8'))

                    if laser_x is not None and laser_y is not None:
                        error_x_text = int(error_x)
                        error_y_text = int(error_y)
                    else:
                        error_x_text = 0
                        error_y_text = 0
                    cv2.circle(out_img, (laser_x, laser_y), 2, (255, 0, 0), -1)  # 激光
                    cv2.putText(out_img, f"Error X: {error_x_text}, Y: {error_y_text}", (x - 50, y + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                start_string = f"@4,0,0,0,0\r"
                ser.write(start_string.encode('utf-8'))

            cv2.imshow("Detected Circles", out_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("用户手动中断程序")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        ser.close()