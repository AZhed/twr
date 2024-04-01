import os

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Create launch description
    ld = LaunchDescription()

    twr_description_pkg_path = get_package_share_directory('twr_description')

    #  === rviz2 ===
    rviz_config_file = os.path.join(
        twr_description_pkg_path, 
        'rviz',
        'config.rviz'
        )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    # === robot_state_publisher ===
    xacro_config_file = os.path.join(
        twr_description_pkg_path, 
        'urdf',
        'twr.urdf.xacro'
    )

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

    #  === joint_state_publisher ===
    # provide JointState messages for robot_state_publisher,
    # also necessary for correct representation of axes in rviz2
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
        


    ld.add_action(rviz2_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)

    return ld