colcon build --merge-install
. install/setup.bash


# 一键启动
. install/setup.bash
ros2 launch car_t265 car_t265.launch.py


# ======= debug 用 ======
# 启动t265
. install/setup.bash
ros2 launch car_t265 t265_publisher.launch.py

# 启动串口
. install/setup.bash
ros2 launch car_t265 uart_driver.launch.py

# 启动导航
. install/setup.bash
ros2 launch car_t265 navigation.launch.py

# 发布目标点
. install/setup.bash
ros2 launch car_t265 target_publisher.launch.py


# 启动imu
. install/setup.bash
ros2 launch car_t265 t265_Imu_pub.launch.py

# 测试线速度
. install/setup.bash
ros2 run car_t265 linear_test