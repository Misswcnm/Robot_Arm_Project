from setuptools import find_packages, setup

package_name = 'yolo_seg_pick'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/yolo_seg_pick.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ylx',
    maintainer_email='ylx@example.com',
    description='Guarded YOLO11 segmentation RGB-D picking demo',
    license='MIT',
    entry_points={'console_scripts': [
        'yolo_seg_pick = yolo_seg_pick.node:main',
    ]},
)
