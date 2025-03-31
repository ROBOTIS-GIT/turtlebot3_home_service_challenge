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
    scenario_yaml = PathJoinSubstitution([
        FindPackageShare('turtlebot3_home_service_challenge_core'),
        'config',
        'scenario.yaml'
    ])

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_home_service_challenge_aruco'),
                    'launch',
                    'aruco_node.launch.py'
                ])
            ]),
        ),

        Node(
            package='turtlebot3_home_service_challenge_manipulator',
            executable='manipulator_controller',
            name='manipulator_controller',
            output='screen',
        ),

        Node(
            package='turtlebot3_home_service_challenge_core',
            executable='home_service_challenge_core',
            name='home_service_challenge_core',
            parameters=[scenario_yaml]
        ),
    ])
