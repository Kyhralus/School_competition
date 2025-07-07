# 校赛

# 隔离编译 --- 为了使每个包的编译文件处于同一文件夹
cd school_competition
colcon build --merge-install

# 启动雷达
. install/setup.bash 
ros2 launch bluesea2 uart_lidar.launch 
ros2 run scan_subscriber scan_subscriber_py

# 启动串口
. install/setup.bash 
<!-- ros2 launch laser_shotting_car uart.launch.py -->
ros2 launch laser_shotting_car uart.launch.py

sudo minicom -D /dev/ttyS1 -b 11520 # 直接启动minicom
ros2 topic pub --once /uart_manager_send_data std_msgs/msg/String "{data: '要发送的实际数据'}"
ros2 topic pub -r 1 /uart_manager_send_data std_msgs/msg/String "{data: '要发送的实际数据'}"  # 循环发送

ros2 topic pub --once /serial_receive std_msgs/msg/String "{data: 'r13'}"  # 循环发送

# 启动激光雷达处理节点
. install/setup.bash 
ros2 run laser_shotting_car lidar_processor

# 一键启动
ros2 launch laser_shotting_car start_nodes.launch.py
# 分开启动


# 相机
. install/setup.bash 
ros2 run laser_shotting_car camera_publisher
. install/setup.bash 
ros2 run laser_shotting_car main_controller
. install/setup.bash 
ros2 run laser_shotting_car lidar_processor
. install/setup.bash 
ros2 run laser_shotting_car yolov8_detector
. install/setup.bash 
ros2 run laser_shotting_car circle_laser_detector

# t265
. install/setup.bash 
ros2 run car_t265 t265_publisher
. install/setup.bash
ros2 run car_t265 target_publisher# School_competition
