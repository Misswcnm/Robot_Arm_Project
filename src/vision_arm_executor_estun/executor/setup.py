from glob import glob
import os

from setuptools import find_packages, setup


PACKAGE_NAME = 'vision_arm_executor_estun'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME],
        ),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (
            os.path.join('share', PACKAGE_NAME, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', PACKAGE_NAME, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vision_arm_executor_estun = '
            'vision_arm_executor_estun.node:main',
            'estun_handeye_calibration = '
            'vision_arm_executor_estun.handeye_calibration:main',
            'estun_handeye_calibration_all = '
            'vision_arm_executor_estun.calibration_stack:main',
            'estun_tcp_calibration = '
            'vision_arm_executor_estun.tcp_calibration:main',
            'estun_gripper_tool = '
            'vision_arm_executor_estun.gripper_tool:main',
            'estun_connection_probe = '
            'vision_arm_executor_estun.connection_probe:main',
            'estun_apriltag_motion_probe = '
            'vision_arm_executor_estun.apriltag_motion_probe:main',
        ],
    },
)
