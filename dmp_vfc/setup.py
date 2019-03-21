#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['robots', 'april_tag_detection', 'general_utils', 'ginger_utils', 'apriltag', 'cameras', 'test', 'utils', 'action_handler', 'scripts'],
     package_dir={'': 'src'}
)

setup(**setup_args)
