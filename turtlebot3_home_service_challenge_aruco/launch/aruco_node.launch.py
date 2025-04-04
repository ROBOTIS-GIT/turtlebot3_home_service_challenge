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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    detect_node = Node(
        package='turtlebot3_home_service_challenge_aruco',
        executable='aruco_detect',
        name='aruco_detect',
        output='screen',
    )

    parking_node = Node(
        package='turtlebot3_home_service_challenge_aruco',
        executable='aruco_parking',
        name='aruco_parking',
        output='screen',
    )

    navigation_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_home_service_challenge_tools'),
                    'launch',
                    'navigation2.launch.py'
                ])
            ]),
        )

    return LaunchDescription([
        navigation_node,
        detect_node,
        parking_node,
    ])
