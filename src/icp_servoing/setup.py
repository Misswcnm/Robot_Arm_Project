from setuptools import setup

# setup.py lives inside the import package directory in this legacy layout.
# find_packages() therefore returns an empty list; map the current directory
# explicitly so `import icp_servoing` works from an installed ROS2 workspace.
setup(name='icp_servoing', version='0.1.0',
      packages=['icp_servoing'],
      package_dir={'icp_servoing': '.'},
      data_files=[('share/ament_index/resource_index/packages', ['resource/icp_servoing']),
                  ('share/icp_servoing', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts':['icp_servoing = icp_servoing.main:main']})
