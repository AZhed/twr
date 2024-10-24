import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True'
    )

    sim_time_launch_cfg = LaunchConfiguration('use_sim_time')

    twr_description_pkg_path = FindPackageShare('twr_description')
    xacro_config_file = PathJoinSubstitution([twr_description_pkg_path, 'urdf', 'twr.urdf.xacro'])

    urdf_config_file = Command(['xacro ', xacro_config_file])
    rsp_node_param = {
        'robot_description': urdf_config_file,
        'use_sim_time': sim_time_launch_cfg
    }
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[rsp_param]
    )

    ld.add_action(sim_time_launch_arg)
    ld.add_action(rsp_node)

    return ld