import cv2
import numpy as np

def detect_concentric_circles_from_image(image, min_radius=100, max_radius=200, min_dist=50, canny_thresh1=50, canny_thresh2=150, area_thresh=100):
    if image is None:
        print("无法获取图像")
        return [], [], image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # @TODO
    cv2.imshow("gray",gray)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    edges = cv2.Canny(blurred, canny_thresh1, canny_thresh2)

    # 轮廓检测
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    circles = []
    for cnt in contours:
        # print(f"找到{len(cnt)}个轮廓")
        area = cv2.contourArea(cnt)
        if area < area_thresh:
            continue
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if min_radius < radius < max_radius:
            # 圆度判据（可选）：实际面积/理论圆面积
            circle_area = np.pi * radius * radius
            if area / circle_area > 0.6:  # 圆度阈值，可调整
                circles.append((int(x), int(y), int(radius)))
                # debug
                print(f"圆心：({x:.2f}, {y:.2f}) 半径：{radius:.2f}")

    concentric_groups = find_concentric_groups(circles)

    # 可视化
    out_img = image.copy()
    for (x, y, r) in circles:
        cv2.circle(out_img, (x, y), r, (0, 255, 0), 2)
        cv2.circle(out_img, (x, y), 2, (0, 0, 255), 3)
    for i, group in enumerate(concentric_groups):
        if len(group) > 1:
            center_x, center_y = get_group_center(group)
            cv2.putText(out_img, f"Group {i+1}", (center_x - 20, center_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    return circles, concentric_groups, out_img

def find_concentric_groups(circles, threshold=5):
    groups = []
    for circle in circles:
        x, y, r = circle
        added = False
        for group in groups:
            group_center = get_group_center(group)
            dist = np.sqrt((x - group_center[0])**2 + (y - group_center[1])**2)
            if dist < threshold:
                group.append(circle)
                added = True
                break
        if not added:
            groups.append([circle])
    return [group for group in groups if len(group) > 1]

def get_group_center(group):
    x_sum = sum(circle[0] for circle in group)
    y_sum = sum(circle[1] for circle in group)
    return (x_sum // len(group), y_sum // len(group))

if __name__ == "__main__":
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("无法打开摄像头")
        exit()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头画面")
            break
        circles, concentric_groups, out_img = detect_concentric_circles_from_image(
            frame,
            min_radius=20,
            max_radius=400,
            min_dist=20,
            canny_thresh1=20,
            canny_thresh2=50,
            area_thresh=100
        )

        # 识别红色激光
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 红色在HSV空间中不连续，需要两个范围
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            print("检测到红色激光轮廓")
            # 找到最大的轮廓作为激光点
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                laser_x = int(M["m10"] / M["m00"])
                laser_y = int(M["m01"] / M["m00"])
                cv2.circle(out_img, (laser_x, laser_y), 5, (0, 255, 255), -1)
                # print("同心圆环：",concentric_groups)
                # for i, group in enumerate(concentric_groups):
                #     print(len(concentric_groups))
                #     if len(group) >= 1:
                #         center_x, center_y = get_group_center(group)
                #         error_x = laser_x - int(center_x)
                #         error_y = laser_y - int(center_y)
                #         print(f"Group {i+1} 横向误差: {error_x}, 纵向误差: {error_y}")
                #         cv2.putText(out_img, f"Error X: {error_x}, Y: {error_y}", (int(center_x) - 50, int(center_y) + 20),
                #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                center_x, center_y = get_group_center(group)
                #         error_x = laser_x - int(center_x)
                #         error_y = laser_y - int(center_y)
                #         print(f"Group {i+1} 横向误差: {error_x}, 纵向误差: {error_y}")
                #         cv2.putText(out_img, f"Error X: {error_x}, Y: {error_y}", (int(center_x) - 50, int(center_y) + 20),
                #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                print("激光点轮廓的矩 m00 为 0，无法计算质心")
        else:
            print("未检测到红色激光轮廓")

        cv2.imshow("Detected Circles", out_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()