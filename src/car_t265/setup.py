from setuptools import find_packages, setup
import glob

package_name = 'car_t265'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装所有 launch 文件
        ('share/' + package_name + '/launch', glob.glob('launch/*launch.py')),
    ],

    install_requires=['setuptools', 'pyrealsense2'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='alineyiee@shu.edu.cn',
    description='Robot navigation package using T265 camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 自主小车
            # ======= 测试用节点 -run ======
            'uart_driver = car_t265.uart_driver:main',
            'motor_controller = car_t265.motor_controller:main',
            'navigation_controller = car_t265.navigation_controller:main',
            'target_publisher = car_t265.target_publisher:main',
            't265_publisher = car_t265.t265_publisher:main',  # t265
            't265_Imu_pub = car_t265.t265_Imu_pub:main',   # 265 Imu数据
            'linear_test = car_t265.linear_test:main',    # 测试线速度
            # 'car_t265 = car_t265.launch:generate_launch_description',   # 完整一键启动
            
        ],
    },
)
