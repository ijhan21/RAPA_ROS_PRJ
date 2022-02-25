from setuptools import setup
from glob import glob
import os

package_name = 'jetbot_ros'
submodules = "jetbot_ros"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hans',
    maintainer_email='withyou@empal.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lidar_control = jetbot_ros.lidar_control:main",
            "lidar_imu_move_control = jetbot_ros.lidar_imu_move_control:main",
            "lidar_control_img = jetbot_ros.lidar_control_img:main",
            "cmd_vel_control = jetbot_ros.cmd_vel_control:main",
            "cam_pub = jetbot_ros.cam_pub:main",
            "cam_sub = jetbot_ros.cam_sub:main",
            "imu_control = jetbot_ros.imu_control:main",
            "hover_serial = jetbot_ros.hover_serial:main",
        ],
    },
)
