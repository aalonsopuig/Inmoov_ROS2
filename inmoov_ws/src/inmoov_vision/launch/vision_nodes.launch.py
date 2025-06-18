#!/usr/bin/env python3
"""
vision_nodes.launch.py

ROS 2 launch file to start both face tracking and face recognition nodes with specific parameters.

Authors: Alejandro Alonso Puig + ChatGPT 4.1
GitHub: https://github.com/aalonsopuig
Date: June 5, 2025
License: Apache 2.0
"""

from launch import LaunchDescription                  # Base class for defining a launch file
from launch_ros.actions import Node                    # Action to launch a ROS 2 node

def generate_launch_description():
    # Create and return a launch description containing both nodes with custom parameters and log level
    return LaunchDescription([

        # --- Face Tracker Node ---
        Node(
            package='inmoov_vision',                   # ROS 2 package containing the node
            executable='face_tracker_node',            # Python executable to run
            name='face_tracker_node',                  # ROS node name
            output='screen',                           # Output logs to the terminal

            # Set ROS 2 parameters for this node:
            # - processing_frequency: frame processing rate (Hz)
            # - show_debug_window: disable debug GUI window
            # - video_device: index of the video device (e.g. 2 for /dev/video2)
            parameters=[
                {'processing_frequency': 1.0},
                {'show_debug_window': False},
                {'video_device': 2}
            ],

            # Set log level to WARN to reduce console output noise
            arguments=['--ros-args', '--log-level', 'WARN']
        ),

        # --- Face Recognition Node ---
        Node(
            package='inmoov_vision',                   # Same package
            executable='face_recognition_node',        # Second executable to run
            name='face_recognition_node',              # Node name
            output='screen',                           # Log to screen

            # Set log level to WARN to suppress info/debug logs
            arguments=['--ros-args', '--log-level', 'WARN']
        )
    ])
