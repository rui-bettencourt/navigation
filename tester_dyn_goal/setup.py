#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['tester_dyn_goal', 'tester_dyn_goal_ros'],
 package_dir={'tester_dyn_goal': 'common/src/tester_dyn_goal', 'tester_dyn_goal_ros': 'ros/src/tester_dyn_goal_ros'}
)

setup(**d)
