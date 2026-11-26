import os
import yaml
from launch.substitutions import LaunchConfiguration
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
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Create main launch description
    ld = LaunchDescription()
    ld.add_action(declare_sim_time_arg)

    # Load robot configurations
    robots = [{'name':'burger'},
              {'name':'waffle_pi'}
            ]

    # Per-robot bringup
    for robot in robots:
        robot_name = robot['name']
        namespace = f'{robot_name}'

        param_file_name = f"{namespace}nav.yaml"
        param_file_path = os.path.join(pkg_share, 'config', param_file_name)

        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation_launch_path),
            launch_arguments={
                    'use_sim_time': use_sim_time,
                    'namespace': '/'+namespace,
                    'params_file': param_file_path,
                    'slam': 'False',
                    'use_localization': 'False',
                    'use_composition': 'False',
                    'autostart': 'True',
                    'use_namespace': 'True',
            }.items()
        )

        relay_tf = Node(
            package='topic_tools',
            executable='relay',
            name='relay_tf',
            output='screen',
            arguments=['tf', namespace+'/tf'],
            parameters=[
                {'use_sim_time': use_sim_time},
            ]
        )

        relay_tf_static = Node(
            package='topic_tools',
            executable='relay',
            name='relay_tf_static',
            output='screen',
            arguments=['tf_static', namespace+'/tf_static'],
            parameters=[
                {'use_sim_time':use_sim_time},
            ]
        )
    
        ld.add_action(nav2_launch)
        ld.add_action(relay_tf)
        ld.add_action(relay_tf_static)

    return ld