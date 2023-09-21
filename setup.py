#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dyros_gui'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'roscpp'],
    scripts=['scripts/dyros_gui']
)

setup(**d)