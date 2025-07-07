from setuptools import find_packages, setup
import glob

package_name = 'laser_shotting_car'

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

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='alineyiee@shu.edu.cn',
    description='laser shotting car',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = laser_shotting_car.camera_publisher:main',
            'main_controller = laser_shotting_car.main_controller   :main',
            'lidar_processor = laser_shotting_car.lidar_processor:main',
            'yolov8_detector = laser_shotting_car.yolov8_detector:main',
            'circle_laser_detector = laser_shotting_car.circle_laser_detector:main',
            'circle_detect1 = laser_shotting_car.circle_detect1:main',
            'vision_tracking_node = laser_shotting_car.vision_tracking_node:main',
            'gimbal_control_node = laser_shotting_car.gimbal_control_node:main',
        ],
    },
)
