import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node , LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch_ros.descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ld = LaunchDescription()
    pkg_share = get_package_share_directory('task_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    robots = [
        {'name':'burger'},
        {'name':'waffle_pi'},
        # {'name':'waffle'}
    ]

    for robot in robots :

        namespace = robot['name']

        slam_param = os.path.join(pkg_share, 'config', namespace +'map.yaml')


        slam_params_file_w_subst = ParameterFile(
            slam_param,
            allow_substs=True,
        )

        slam_node = LifecycleNode(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            output='screen',
            remappings=[
                ('/map','map'),
                ('/map_metadata','map_metadata'),
                ('/slam_toolbox/scan_visualization','slam_toolbox/scan_visualization'),
                ('/slam_toolbox/graph_visualization','slam_toolbox/graph_visualization'),
            ],
            parameters=[
                    slam_params_file_w_subst,
                    {'use_sim_time': use_sim_time},
            ],
        )

        slam_configure_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )

        slam_activate_event = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_node,
                goal_state='inactive',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        )
        
        ld.add_action(slam_node)
        ld.add_action(slam_configure_event)
        ld.add_action(slam_activate_event)

    return ld