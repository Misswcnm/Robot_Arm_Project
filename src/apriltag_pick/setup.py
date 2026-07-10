from setuptools import find_packages, setup

package_name = 'apriltag_pick'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pick.yaml', 'config/tags.yaml']),
        ('share/' + package_name + '/launch', ['launch/apriltag_pick.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ylx',
    maintainer_email='ylx@example.com',
    description='AprilTag localization and guarded pick control for Dobot CR5 and RealSense D455',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_node = apriltag_pick.pick_node:main',
            'interactive = apriltag_pick.interactive:main',
        ],
    },
)
