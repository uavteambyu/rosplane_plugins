#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['waypoint_planning'],
    package_dir={'': 'src'},
    scripts={'scripts/waypoint_planning'}
)

setup(**d)