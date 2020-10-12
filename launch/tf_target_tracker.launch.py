#  Copyright 2020 ADLINK Technology, Inc.
#  Developer: Skyler Pan (skylerpan@gmail.com)
# 
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
# 
#      http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([

        # Use TensorFlow
        launch_ros.actions.Node(
            package='robot_tracking_controller', node_executable='target_extractor',
            output='screen',
            parameters=[
                {'camera_link':'camera_link'},
            ],
        ),

        launch_ros.actions.Node(
            package='robot_tracking_controller', node_executable='target_filter',
            output='screen',
        ),

        launch_ros.actions.Node(
            package='robot_tracking_controller', node_executable='target_tracker',
            output='screen',
        ),

        #launch_ros.actions.Node(
        #    package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        #    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0', '/camera_link', '/world']),

        #launch_ros.actions.Node(
        #    package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        #    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0', '/camera_link', '/base_link']),

        #launch_ros.actions.Node(
        #    package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        #    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0', '/base_link', '/base_footprint'])
    ])