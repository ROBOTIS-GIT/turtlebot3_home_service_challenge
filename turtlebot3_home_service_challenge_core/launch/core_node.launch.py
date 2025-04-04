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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    laucnh_mode_arg = DeclareLaunchArgument(
        'launch_mode',
        default_value='simulation',
        description='launch mode type [simulation, actual]'
    )
    launch_mode = LaunchConfiguration('launch_mode')

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.088',
        description='marker size double value'
    )
    marker_size = LaunchConfiguration('marker_size')

    scenario_yaml = PathJoinSubstitution([
        FindPackageShare('turtlebot3_home_service_challenge_core'),
        'config',
        'scenario.yaml'
    ])

    laucnh_aruco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_home_service_challenge_aruco'),
                'launch',
                'aruco_node.launch.py'
            ])
        ]),
        launch_arguments={
            'launch_mode': launch_mode,
            'marker_size': marker_size,
        }.items(),
    )

    launch_manipulator_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_home_service_challenge_manipulator'),
                'launch',
                'manipulator_node.launch.py'
            ])
        ]),
    )

    core_node = Node(
            package='turtlebot3_home_service_challenge_core',
            executable='home_service_challenge_core',
            name='home_service_challenge_core',
            parameters=[
                scenario_yaml,
            ]
        )

    return LaunchDescription([
        laucnh_mode_arg,
        marker_size_arg,
        laucnh_aruco_node,
        launch_manipulator_node,
        core_node,
    ])
