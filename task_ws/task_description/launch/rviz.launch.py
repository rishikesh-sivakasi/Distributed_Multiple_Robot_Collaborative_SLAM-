import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

# export TURTLEBOT3_MODEL=burger

def generate_launch_description():

    pkg_share = get_package_share_directory('task_gazebo')

    world_file = os.path.join(pkg_share, 'worlds', 'hospital.world') 

    gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_model_path = "/home/arz-1029/Desktop/Rishikesh_Evaluvation/task_ws/install/task_gazebo/share/task_gazebo/models"
    urdf_meshes_path = "/home/arz-1029/Desktop/Rishikesh_Evaluvation/task_ws/install/task_description/share"

    resource_path = f"{world_model_path}:{urdf_meshes_path}"

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py',)),
        launch_arguments={'gz_args': [world_file , ' -r']}.items()
    )
    
    return LaunchDescription([
        gazebo_resource_path,
        gazebo_launch
    ])