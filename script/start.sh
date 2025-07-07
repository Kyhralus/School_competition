#!/bin/bash

# chmod +x /home/orangepi/ros2_workspace/school_competition/start.sh
# 构建工作空间
# # 终止所有 ros2 进程
# pkill -9 -f "ros2"

# colcon build --merge-install
# 加载工作空间环境
. install/setup.bash 

# 后台启动节点
. install/setup.bash 
ros2 launch bluesea2 uart_lidar.launch &

. install/setup.bash 
ros2 run car_t265 t265_publisher &
# 延时 2 秒启动 yolov8_detector 节点
sleep 3
. install/setup.bash 
ros2 run laser_shotting_car main_controller &
. install/setup.bash 
ros2 run laser_shotting_car lidar_processor &
. install/setup.bash 
ros2 run laser_shotting_car camera_publisher &
. install/setup.bash 
ros2 run laser_shotting_car yolov8_detector &
. install/setup.bash 
ros2 run laser_shotting_car circle_laser_detector &

