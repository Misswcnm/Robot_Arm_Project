from setuptools import find_packages, setup
setup(name='icp_servoing', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/icp_servoing']),
                  ('share/icp_servoing', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts':['icp_servoing = icp_servoing.main:main']})
