#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# File: xicro_nodes.launch.py
# Description: ROS 2 launch file to start both XICRO communication nodes
#              (subsystem 1 and subsystem 2) for the InMoov robot.
# Author: Alejandro Alonso Puig
# Date: 2025-05-28
# License: Apache License, Version 2.0
#
# Copyright 2025 Alejandro Alonso Puig
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -----------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts both XICRO nodes for subsystem 1 (right arm)
    and subsystem 2 (left arm and head) for the InMoov robot.
    """
    return LaunchDescription([
        Node(
            package='xicro_nodes',
            executable='xicro_node_subsys1_ID_1_arduino',
            #name='xicro_subsys1',
            output='screen'
        ),
        Node(
            package='xicro_nodes',
            executable='xicro_node_subsys2_ID_2_arduino',
            #name='xicro_subsys2',
            output='screen'
        ),
    ])
