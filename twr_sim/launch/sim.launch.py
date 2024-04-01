import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name='twr_sim'

    # Create launch description
    ld = LaunchDescription()

    # === robot_state_publisher ===
    twr_description_pkg_path = get_package_share_directory('twr_description')
    
    xacro_config_file = os.path.join(
        twr_description_pkg_path, 
        'urdf',
        'twr.urdf.xacro')

    urdf_config_file = Command(['xacro ', xacro_config_file])
    robot_state_publisher_param = {
        'robot_description': urdf_config_file,
        'use_sim_time': True
    }
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_state_publisher_param]
    )
    

    # === gazebo ====
    gazebo_pkg_path = FindPackageShare('gazebo_ros')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_pkg_path, 'launch', 'gazebo.launch.py'])
        ])
    )

    # === spawn_entity ===
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'twr_model']
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_entity_node)

    return ld