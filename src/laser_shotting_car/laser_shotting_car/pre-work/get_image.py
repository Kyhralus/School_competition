import cv2
import time
import os

def capture_images():
    try:
        # 检查摄像头设备
        if not os.path.exists('/dev/video0'):
            print("错误：未检测到摄像头设备 /dev/video0")
            print("请检查摄像头是否已正确连接")
            return
        
        # 尝试打开摄像头
        cap = cv2.VideoCapture(2)
        if not cap.isOpened():
            print("错误：无法打开摄像头")
            print("可能原因：")
            print("1. 摄像头被其他程序占用")
            print("2. 权限不足，请尝试使用sudo运行")
            print("3. 摄像头驱动问题")
            return
        
        # 设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # 检查实际分辨率
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"摄像头已打开，分辨率: {width}x{height}")
        
        img_count = 1200
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法获取帧")
                break
            frame = cv2.circle(frame, (160, 120), 3, (0, 255, 255), -1)
            cv2.imshow('Camera View', frame)
            
            key = cv2.waitKey(1)
            if key == 27:  # ESC键退出
                break
            elif key == ord('s'):  # 按s键开始保存
                print("开始保存10张图片...")
                
                # 确保data目录存在
                if not os.path.exists('data'):
                    os.makedirs('data')
                
                for i in range(2):
                    ret, frame = cap.read()
                    if not ret:
                        print("无法获取帧")
                        break
                        
                    filename = f"/home/orangepi/ros2_workspace/school_competition/src/laser_shotting_car/datasets/img{img_count}.jpg"
                    if cv2.imwrite(filename, frame):
                        print(f"已保存: {filename}")
                        img_count += 1
                    else:
                        print(f"保存失败: {filename}")
                    frame = cv2.circle(frame, (320, 240), 5, (0, 255, 255), -1)
                    cv2.imshow('Camera View', frame)
                    if cv2.waitKey(1) == 27:
                        break
                    time.sleep(0.5)
                
                print("10张图片保存完成")
        
    except Exception as e:
        print(f"程序运行出错: {str(e)}")
    finally:
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()
        print("程序结束")

if __name__ == "__main__":
    capture_images()