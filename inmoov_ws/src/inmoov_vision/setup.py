#!/usr/bin/env python3
"""
setup.py

Setup script for the inmoov_vision ROS 2 package.
This script configures installation, specifies which resources and launch files to include,
and registers Python ROS 2 executables for launching via ros2 run or ros2 launch.

Author: Alejandro Alonso Puig
GitHub: https://github.com/aalonsopuig
Date: June 4, 2025
License: Apache 2.0
"""

from setuptools import setup         # Core setuptools functionality for packaging
from glob import glob                # To match file patterns (e.g., all launch/*.py)
import os                            # For directory and path operations

package_name = 'inmoov_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],   # Main Python package directory (should match the subfolder name)
	package_data={
    	'inmoov_vision': [
        	'data/*.pickle',
        	'models/*.dat',     # Añadido: para shape_predictor
    	],
	},
    data_files=[
        # Required by ament for package indexing in ROS 2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),

        # Install the ROS 2 package manifest
        ('share/' + package_name, ['package.xml']),

        # Install Haar cascade XMLs (if used) for face detection
        (os.path.join('share', package_name, 'haarcascades'),
         glob(os.path.join(package_name, 'haarcascades', '*.xml'))),

        # Install all launch files from the launch/ directory
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),

        # Install data files (such as encodings.pickle) to the correct site-packages path
        (os.path.join('lib', package_name, 'data'),
         glob(os.path.join(package_name, 'data', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro Alonso Puig',
    maintainer_email='aalonsopuig@gmail.com',
    description=(
        "ROS 2 package for face tracking and face recognition using OpenCV and Dlib. "
        "Includes launch files, classifiers, and resources for integration in modular robotics."
    ),
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        # Register Python nodes so they can be launched with ros2 run or ros2 launch
        'console_scripts': [
            'face_tracker_node = inmoov_vision.face_tracker_node:main',           # Face tracking node
            'face_recognition_node = inmoov_vision.face_recognition_node:main',   # Face recognition node
            # (Add more nodes here as needed)
        ],
    },
)
