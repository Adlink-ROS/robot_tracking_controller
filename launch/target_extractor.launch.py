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
        launch_ros.actions.Node(
            package='tf_ai_tracker', node_executable='target_extractor',
            output='screen',
            parameters=[
                {'camera_link':'camera_color_optical_frame'},
            #{'enable_depth':True},
            #{'enable_pointcloud':True},
            #{'enable_aligned_depth':True},
            #{'enable_aligned_pointcloud':True},
            ],
            remappings=[
                ('/camera/pointcloud', '/camera/aligned_depth_to_color/color/points')
                ],
             #arguments=['__params:=' + aligned_yaml],            
            ),

        # Rviz
        #launch_ros.actions.Node(
            #package='rviz2', node_executable='rviz2', output='screen',
            #arguments=['--display-config', default_rviz]),

        # TF
        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher', output='screen',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0', '/base_footprint', '/camera_color_optical_frame']),

        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher', output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0', '/camera_base', '/base_link']),

        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher', output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0', '/base_link', '/base_footprint'])
    ])