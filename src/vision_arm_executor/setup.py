from setuptools import find_packages, setup


PACKAGE_NAME = 'vision_arm_executor'


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
            'share/' + PACKAGE_NAME + '/config',
            ['config/executor.yaml', 'config/nx_routes.json'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    test_suite='test',
    entry_points={
        'console_scripts': [
            'vision_arm_executor = vision_arm_executor.node:main',
            'nx_compat_gateway = vision_arm_executor.nx_gateway:main',
            'icp_teach = vision_arm_executor.teach_cli:main',
        ],
    },
)
