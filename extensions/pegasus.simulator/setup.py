"""
| File: setup.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: File that defines the installation requirements for this python package.
"""
import os

from setuptools import setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # generic
    "numpy",
    "pymavlink",
    "scipy",
    "pyyaml",
]

# Installation operation
setup(
    name="pegasus-simulator",
    author="Marcelo Jacinto",
    maintainer="Marcelo Jacinto",
    maintainer_email="marcelo.jacinto@tecnico.ulisboa.pt",
    url="https://github.com/PegasusSimulator/PegasusSimulator/tree/main/extensions/pegasus.simulator",
    version="4.5.0",
    description="Extension providing the main framework interfaces for simulating aerial vehicles using PX4, Python or ROS 2 as a backend",
    keywords=["drone", "quadrotor", "multirotor", "UAV", "px4", "sitl", "robotics"],
    license="BSD-3-Clause",
    include_package_data=True,
    python_requires=">=3.7",
    install_requires=INSTALL_REQUIRES,
    packages=["pegasus.simulator"],
    classifiers=["Natural Language :: English", "Programming Language :: Python :: 3.7"],
    zip_safe=False,
)