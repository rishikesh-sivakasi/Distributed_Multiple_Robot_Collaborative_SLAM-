#!/usr/bin/env python3
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
# Authors: Arshad Mehmood

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    pkg_share = get_package_share_directory('task_gazebo')

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true.'
    )

    # Create main launch description
    ld = LaunchDescription()
    ld.add_action(declare_sim_time_arg)

    # Load robot configurations
    # robots = [{'name':'burger'},
    #           {'name':'waffle_pi'}
    #         ]

    # use_sim_time = LaunchConfiguration('use_sim_time')

    # # Per-robot bringup
    # for robot in robots:
    #     robot_name = robot['name']
    #     namespace = f'{robot_name}'

    #     param_file_name = f"{namespace}nav.yaml"
    #     param_file_path = os.path.join(pkg_share, 'config', param_file_name)

    #     nav2_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(nav2_navigation_launch_path),
    #         launch_arguments={
    #                 'use_sim_time': LaunchConfiguration('use_sim_time'),
    #                 'namespace': '/'+namespace,
    #                 'params_file': param_file_path,
    #                 'slam': 'True',
    #                 'use_localization': 'False',
    #                 'use_composition': 'False',
    #                 'autostart': 'True',
    #                 'use_namespace': 'True',
    #         }.items()
    #     )

    #     relay_tf = Node(
    #         package='topic_tools',
    #         executable='relay',
    #         name='relay_tf',
    #         output='screen',
    #         arguments=['tf', namespace+'/tf'],
    #         parameters=[
    #             {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #         ]
    #     )

    #     relay_tf_static = Node(
    #         package='topic_tools',
    #         executable='relay',
    #         name='relay_tf_static',
    #         output='screen',
    #         arguments=['tf_static', namespace+'/tf_static'],
    #         parameters=[
    #             {'use_sim_time': LaunchConfiguration('use_sim_time')},
    #         ]
    #     )
    
    #     ld.add_action(nav2_launch)
    #     ld.add_action(relay_tf)
    #     ld.add_action(relay_tf_static)

    param_file_path = os.path.join(pkg_share, 'config', 'burgernav.yaml')
    param_file_path2 = os.path.join(pkg_share, 'config', 'waffle_pinav.yaml')

    navigation_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'namespace': '/burger',
                'params_file': param_file_path,
                'slam': 'True',
                'use_localization': 'False',
                'use_composition': 'False',
                'autostart': 'True',
        }.items()
    )

    navigation_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'namespace': '/waffle_pi',
                'params_file': param_file_path2,
                'slam': 'True',
                'use_localization': 'False',
                'use_composition': 'False',
                'autostart': 'True',
        }.items()
    )

    relay_tf_1 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tf_1',
        output='screen',
        arguments=['tf', 'burger/tf'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_tf_static_1 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tf_static_1',
        output='screen',
        arguments=['tf_static', 'burger/tf_static'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_tf_2 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tf_2',
        output='screen',
        arguments=['tf', 'waffle_pi/tf'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_tf_static_2 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tf_static_2',
        output='screen',
        arguments=['tf_static', 'waffle_pi/tf_static'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    ld.add_action(navigation_launch_1)
    # ld.add_action(relay_tf_1)
    # ld.add_action(relay_tf_static_1)

    delayed_waffle = TimerAction(
        period=10.0,
        actions=[
            navigation_launch_2,
            # relay_tf_2,
            # relay_tf_static_2
        ]
    )

    ld.add_action(delayed_waffle)


    return ld