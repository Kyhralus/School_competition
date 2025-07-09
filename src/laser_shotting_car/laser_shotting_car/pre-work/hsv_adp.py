import cv2
import numpy as np

def create_trackbars():
    """创建HSV阈值、灰度阈值和曝光调节滑动条"""
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 640, 480)  # 增大窗口以容纳更多滑动条
    
    # 创建HSV下限滑动条
    cv2.createTrackbar("H_min", "Trackbars", 0, 179, lambda x: x)
    cv2.createTrackbar("S_min", "Trackbars", 0, 255, lambda x: x)
    cv2.createTrackbar("V_min", "Trackbars", 0, 255, lambda x: x)
    
    # 创建HSV上限滑动条
    cv2.createTrackbar("H_max", "Trackbars", 179, 179, lambda x: x)
    cv2.createTrackbar("S_max", "Trackbars", 255, 255, lambda x: x)
    cv2.createTrackbar("V_max", "Trackbars", 255, 255, lambda x: x)
    
    # 创建灰度阈值滑动条（新增）
    cv2.createTrackbar("Gray_min", "Trackbars", 0, 255, lambda x: x)
    cv2.createTrackbar("Gray_max", "Trackbars", 255, 255, lambda x: x)
    
    # 创建形态学操作参数滑动条
    cv2.createTrackbar("Erode", "Trackbars", 0, 10, lambda x: x)
    cv2.createTrackbar("Dilate", "Trackbars", 0, 10, lambda x: x)
    
    # 创建面积阈值滑动条
    cv2.createTrackbar("Min Area", "Trackbars", 100, 5000, lambda x: x)
    cv2.createTrackbar("Max Area", "Trackbars", 5000, 50000, lambda x: x)
    
    # 创建曝光调节滑动条
    cv2.createTrackbar("Brightness", "Trackbars", 0, 100, lambda x: x)  # 亮度调节
    cv2.createTrackbar("Contrast", "Trackbars", 100, 200, lambda x: x)  # 对比度调节
    
    # 创建模式选择滑动条（新增）
    cv2.createTrackbar("Mode", "Trackbars", 0, 1, lambda x: x)  # 0: HSV模式, 1: 灰度模式

def get_trackbar_values():
    """获取所有滑动条当前值"""
    h_min = cv2.getTrackbarPos("H_min", "Trackbars")
    s_min = cv2.getTrackbarPos("S_min", "Trackbars")
    v_min = cv2.getTrackbarPos("V_min", "Trackbars")
    h_max = cv2.getTrackbarPos("H_max", "Trackbars")
    s_max = cv2.getTrackbarPos("S_max", "Trackbars")
    v_max = cv2.getTrackbarPos("V_max", "Trackbars")
    
    # 获取灰度阈值（新增）
    gray_min = cv2.getTrackbarPos("Gray_min", "Trackbars")
    gray_max = cv2.getTrackbarPos("Gray_max", "Trackbars")
    
    erode = cv2.getTrackbarPos("Erode", "Trackbars")
    dilate = cv2.getTrackbarPos("Dilate", "Trackbars")
    min_area = cv2.getTrackbarPos("Min Area", "Trackbars")
    max_area = cv2.getTrackbarPos("Max Area", "Trackbars")
    
    # 获取曝光调节值
    brightness = cv2.getTrackbarPos("Brightness", "Trackbars") - 50  # 范围：-50到50
    contrast = cv2.getTrackbarPos("Contrast", "Trackbars") / 100.0  # 范围：1.0到2.0
    
    # 获取模式选择（新增）
    mode = cv2.getTrackbarPos("Mode", "Trackbars")  # 0: HSV, 1: 灰度
    
    return (h_min, s_min, v_min), (h_max, s_max, v_max), gray_min, gray_max, erode, dilate, min_area, max_area, brightness, contrast, mode

def adjust_exposure(frame, brightness=0, contrast=1.0):
    """调整图像亮度和对比度"""
    # 应用线性变换：result = contrast * frame + brightness
    adjusted = cv2.convertScaleAbs(frame, alpha=contrast, beta=brightness)
    return adjusted

def track_color(frame, lower_hsv, upper_hsv, erode, dilate, min_area, max_area):
    """追踪指定颜色的色块（HSV模式）"""
    result_frame = frame.copy()
    
    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 创建颜色掩码
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    # 形态学操作
    kernel = np.ones((3, 3), np.uint8)
    if erode > 0:
        mask = cv2.erode(mask, kernel, iterations=erode)
    if dilate > 0:
        mask = cv2.dilate(mask, kernel, iterations=dilate)
    
    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 存储找到的色块信息
    color_blobs = []
    
    for contour in contours:
        # 计算轮廓面积
        area = cv2.contourArea(contour)
        
        # 过滤面积不符合要求的轮廓
        if area < min_area or area > max_area:
            continue
        
        # 计算轮廓中心
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            color_blobs.append({
                "contour": contour,
                "center": (cx, cy),
                "area": area
            })
    
    # 在图像上标记色块
    for blob in color_blobs:
        cx, cy = blob["center"]
        cv2.circle(result_frame, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(result_frame, f"({cx}, {cy})", (cx + 10, cy - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.drawContours(result_frame, [blob["contour"]], -1, (0, 255, 0), 2)
    
    return result_frame, mask, hsv, color_blobs

def track_gray(frame, gray_min, gray_max, erode, dilate, min_area, max_area):
    """追踪灰度范围内的区域（新增）"""
    result_frame = frame.copy()
    
    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 创建灰度掩码
    mask = cv2.inRange(gray, gray_min, gray_max)
    
    # 形态学操作
    kernel = np.ones((3, 3), np.uint8)
    if erode > 0:
        mask = cv2.erode(mask, kernel, iterations=erode)
    if dilate > 0:
        mask = cv2.dilate(mask, kernel, iterations=dilate)
    
    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 存储找到的区域信息
    gray_blobs = []
    
    for contour in contours:
        # 计算轮廓面积
        area = cv2.contourArea(contour)
        
        # 过滤面积不符合要求的轮廓
        if area < min_area or area > max_area:
            continue
        
        # 计算轮廓中心
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            gray_blobs.append({
                "contour": contour,
                "center": (cx, cy),
                "area": area
            })
    
    # 在图像上标记区域
    for blob in gray_blobs:
        cx, cy = blob["center"]
        cv2.circle(result_frame, (cx, cy), 5, (0, 0, 255), -1)  # 红色标记灰度检测
        cv2.putText(result_frame, f"({cx}, {cy})", (cx + 10, cy - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.drawContours(result_frame, [blob["contour"]], -1, (0, 0, 255), 2)
    
    return result_frame, mask, gray, gray_blobs

def main():
    # 初始化摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    # 设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 创建滑动条
    create_trackbars()
    
    print("程序启动，使用滑动条调整参数")
    print("模式: 0=HSV颜色追踪, 1=灰度追踪")
    print("按'q'键退出程序")
    
    try:
        while True:
            # 读取帧
            ret, frame = cap.read()
            if not ret:
                print("无法获取图像")
                break
            
            # 获取滑动条值（包含灰度阈值和模式）
            lower_hsv, upper_hsv, gray_min, gray_max, erode, dilate, min_area, max_area, brightness, contrast, mode = get_trackbar_values()
            
            # 调整曝光
            adjusted_frame = adjust_exposure(frame, brightness, contrast)
            
            # 根据模式选择追踪方法
            if mode == 0:  # HSV模式
                result_frame, mask, processed, blobs = track_color(
                    adjusted_frame, lower_hsv, upper_hsv, erode, dilate, min_area, max_area
                )
                mode_text = "HSV Mode"
            else:  # 灰度模式
                result_frame, mask, processed, blobs = track_gray(
                    adjusted_frame, gray_min, gray_max, erode, dilate, min_area, max_area
                )
                mode_text = "Gray Mode"
            
            # 显示原始帧和曝光调整后的帧
            cv2.imshow("Original", frame)
            cv2.imshow("Adjusted", adjusted_frame)
            
            # 显示处理后的图像和掩码
            cv2.imshow("Processed", processed)
            cv2.imshow("Mask", mask)
            
            # 显示找到的色块数量、模式和曝光参数
            cv2.putText(result_frame, f"Mode: {mode_text}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(result_frame, f"Blobs: {len(blobs)}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(result_frame, f"Brightness: {brightness}, Contrast: {contrast:.2f}", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 显示结果
            cv2.imshow("Result", result_frame)
            
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except Exception as e:
        print(f"发生错误: {e}")
    
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()
        print("程序退出")

if __name__ == "__main__":
    main()
