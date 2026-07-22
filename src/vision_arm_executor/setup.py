from setuptools import find_packages, setup
setup(name='vision_arm_executor', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/vision_arm_executor']),
 ('share/vision_arm_executor',['package.xml']),('share/vision_arm_executor/config',[
     'config/executor.yaml', 'config/nx_routes.json'])],
 install_requires=['setuptools'], zip_safe=True,
 entry_points={'console_scripts':[
     'vision_arm_executor = vision_arm_executor.node:main',
     'nx_compat_gateway = vision_arm_executor.nx_gateway:main',
 ]})
