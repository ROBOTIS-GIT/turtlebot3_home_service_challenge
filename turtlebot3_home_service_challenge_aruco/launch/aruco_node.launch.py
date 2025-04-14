#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Author: ChanHyeong Lee

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    laucnh_mode_arg = DeclareLaunchArgument(
        'launch_mode',
        default_value='simulation',
        description='launch mode type [simulation, actual]')
    launch_mode = LaunchConfiguration('launch_mode')

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.088',
        description='marker size double value')
    marker_size = LaunchConfiguration('marker_size')

    tracker_node = Node(
        package='turtlebot3_home_service_challenge_aruco',
        executable='aruco_tracker',
        name='aruco_tracker',
        output='screen',
        parameters=[{
            'launch_mode': launch_mode,
            'marker_size': marker_size,
        }],
    )

    parking_node = Node(
        package='turtlebot3_home_service_challenge_aruco',
        executable='aruco_parking',
        name='aruco_parking',
        output='screen',
    )

    return LaunchDescription([
        laucnh_mode_arg,
        marker_size_arg,
        tracker_node,
        parking_node,
    ])
