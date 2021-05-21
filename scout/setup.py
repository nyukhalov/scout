from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # the package list is not recursive and all sub-packages must be listed
    # here in order to be installed
    packages=[
        'scout',
        'scout.lib',
        'scout.lib.driver',
        'scout.ros',
        'scout.ros.controller',
        'scout.ros.joystick',
    ],
    package_dir={'': 'src'}
)

setup(**d)
