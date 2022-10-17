#!/usr/bin/env python3

## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
setup_args = generate_distutils_setup(
  packages=['symbolic_fact_generation'],
  package_dir={'': 'src'}
)

setup(**setup_args)
