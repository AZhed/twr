import os

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Create launch description
    ld = LaunchDescription()

    twr_description_pkg_path = get_package_share_directory('twr_description')

    rviz_config_file = os.path.join(
        twr_description_pkg_path, 
        'rviz',
        'config.rviz')

    xacro_config_file = os.path.join(
        twr_description_pkg_path, 
        'urdf',
        'twr.urdf.xacro')

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file])


    urdf_config_file = Command(['xacro ', xacro_config_file])

    robot_state_publisher_param = [{'robot_description': urdf_config_file}]
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=robot_state_publisher_param)
        


    ld.add_action(rviz2_node)
    ld.add_action(robot_state_publisher_node)

    return ld