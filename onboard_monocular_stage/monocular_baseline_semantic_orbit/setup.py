#!/usr/bin/env python3
from setuptools import find_packages, setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=find_packages("vendor"),
    package_dir={"": "vendor"},
    package_data={"camera_coordinate": ["weights/*.pth"]},
)
setup(**setup_args)
