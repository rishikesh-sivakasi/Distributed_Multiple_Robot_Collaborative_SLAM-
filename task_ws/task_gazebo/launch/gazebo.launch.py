import os
import xacro
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.actions import TimerAction
from launch_ros.actions import Node , LifecycleNode
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription , GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ld = LaunchDescription()
    namespace = LaunchConfiguration('namespace', default='')
    pkg_share = get_package_share_directory('task_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_file = os.path.join(pkg_share, 'worlds', 'hospital.world') 

    gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_model_path = "/home/arz-1029/Desktop/Rishikesh_Evaluvation/task_ws/install/task_gazebo/share/task_gazebo/models"
    urdf_meshes_path = "/home/arz-1029/Desktop/Rishikesh_Evaluvation/task_ws/install/task_description/share"

    resource_path = f"{world_model_path}:{urdf_meshes_path}"

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': f'-r -s -v2 {world_file}', 'on_exit_shutdown': 'true'}.items()
    # )
    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items()
    # )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py',)),
        launch_arguments={'gz_args': [world_file , ' -r']}.items()
    )

    ld.add_action(gazebo_resource_path)
    ld.add_action(gazebo_launch)
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)

    robots = [
        {'name':'burger' , 'x':'1.5' , 'y':'1.0', 'z':'1'},
        {'name':'waffle_pi' , 'x':'3.0' , 'y':'9.0', 'z':'1' },
        # {'name':'waffle' , 'x':'-2.0' , 'y':'-2.0', 'z':'1'}
    ]

    for robot in robots :

        namespace = robot['name']

        frame_prefix = LaunchConfiguration('frame_prefix', default='')

        urdf_file_name = 'turtlebot3_' + namespace + '.urdf'

        bridge = os.path.join(pkg_share, 'config', namespace +'bridge.yaml')

        print('urdf_file_name : {}'.format(urdf_file_name))


        urdf = os.path.join(
            get_package_share_directory('task_description'),
            'urdf',
            urdf_file_name)

        robot_desc = Command([
            'xacro ',
            urdf,
            ' namespace:=',
            PythonExpression(['"', namespace, '" + "/" if "', namespace, '" != "" else ""']),
        ])

        # rviz_node = Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     namespace=namespace,
        #     name='rviz2',
        #     output='screen'
        # )

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_desc,
                        'frame_prefix': PythonExpression(["'", frame_prefix, "/'"]),
                        'use_sim_time':use_sim_time,
                        }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        )

        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-name' , 'turtlebot3_'+namespace,
                '-robot_namespace',namespace,
                '-topic', '/'+namespace+'/robot_description',
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', robot['z'],
                '-Y', '0.0',
            ]
        )

        bridge_cmd_vel = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': bridge,
                'use_sim_time': use_sim_time}],
            output='screen'
        )
        
        # ld.add_action(rviz_node)

        delayed = TimerAction(
            period=20.0,
            actions=[
                bridge_cmd_vel,
                robot_state_publisher,
                spawn_entity,
            ]
        )   

        ld.add_action(delayed)

        # ld.add_action(robot_state_publisher)
        # ld.add_action(spawn_entity)
        # ld.add_action(bridge_cmd_vel)

    return ld