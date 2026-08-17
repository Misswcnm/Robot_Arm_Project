## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD 
from distutils.core import setup 
from catkin_pkg.python_setup import generate_distutils_setup 
# fetch values from package.xml 
setup_args = generate_distutils_setup(
   packages=['master_node','functools32','jsonschema'],
   package_dir={'': 'src','jsonschema':'src/jsonschema'},
   package_data={'jsonschema': ['schemas/*.json']},
) 

setup(**setup_args)

