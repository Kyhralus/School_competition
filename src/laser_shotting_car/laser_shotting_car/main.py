import cv2
import time
from servo_horizontal import Servo as ServoH
from servo_vertical import Servo as ServoV
from circle_detect import detect_concentric_circles_from_image

# 摄像头参数（请根据实际情况调整）
CAM_FOV_X = 60  # 水平方向视场角，单位度
CAM_FOV_Y = 45  # 垂直方向视场角，单位度
CAM_RES_X = 640  # 图像宽度
CAM_RES_Y = 480  # 图像高度
LASER_OFFSET = 5.0  # 激光笔在摄像头正上方5cm

# 舵机初始化
servo_h = ServoH(pwmchip=1, channel=0, freq=50)
servo_v = ServoV(pwmchip=3, channel=0, freq=50)

def pixel_to_angle(x, y):
    # 将像素坐标(x, y)转换为舵机角度偏移
    dx = x - CAM_RES_X // 2
    dy = y - CAM_RES_Y // 2
    angle_x = dx / CAM_RES_X * CAM_FOV_X  # 水平角度
    angle_y = dy / CAM_RES_Y * CAM_FOV_Y  # 垂直角度
    return angle_x, angle_y

def clamp(val, minv, maxv):
    return max(minv, min(maxv, val))

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_RES_X)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_RES_Y)
    # 舵机初始角度
    angle_h = 90
    angle_v = 90
    servo_h.set_angle(angle_h)
    servo_v.set_angle(angle_v)
    print("系统启动，等待识别...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法获取摄像头图像")
                break
            circles, _, out_img = detect_concentric_circles_from_image(frame)
            if circles:
                # 取第一个圆心
                x, y, r = circles[0]
                # 计算需要调整的角度
                delta_h, delta_v = pixel_to_angle(x, y)
                # 计算新角度
                angle_h_new = clamp(90 + delta_h, 0, 180)
                angle_v_new = clamp(90 - delta_v, 0, 180)
                # 控制舵机
                servo_h.set_angle(angle_h_new)
                servo_v.set_angle(angle_v_new)
                print(f"目标圆心: ({x},{y}), 水平角: {angle_h_new:.1f}, 垂直角: {angle_v_new:.1f}")
                # 可视化
                cv2.circle(out_img, (int(x), int(y)), int(r), (0,255,0), 2)
                cv2.circle(out_img, (int(x), int(y)), 2, (0,0,255), 3)
            cv2.imshow("Laser Targeting", out_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        servo_h.close()
        servo_v.close()
        cap.release()
        cv2.destroyAllWindows()