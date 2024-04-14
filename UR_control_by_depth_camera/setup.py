# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ["UR_control_by_depth_camera"],
    package_dir = {'': 'src'},
)

setup(**setup_args)