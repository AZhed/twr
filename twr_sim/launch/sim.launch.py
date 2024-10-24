import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # =============================
    # === Robot State Publisher ===
    # =============================
    twr_control_pkg_path = FindPackageShare('twr_control')
    rsp_ld_source = PythonLaunchDescriptionSource([
        PathJoinSubstitution([twr_control_pkg_path, 'launch', 'rsp.launch.py'])
    ])

    rsp_ld = IncludeLaunchDescription(
        rsp_ld_source,
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    
    # ===================
    # === Gazebo Sim ====
    # ===================
    ros_gz_pkg_path = FindPackageShare('ros_gz_sim')
    ros_gz_ld_source = PythonLaunchDescriptionSource([
        PathJoinSubstitution([ros_gz_pkg_path, 'launch', 'gz_sim.launch.py'])
    ])

    ros_gz_ld = IncludeLaunchDescription(ros_gz_ld_source)

    # === spawn_entity ===
    # robot_description from robot_state_publisher node
    spawn_entity_node_param = {'name' : 'twr',
                               'topic': 'robot_description'}

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[spawn_entity_param]
    )

    ld.add_action(rsp_ld)
    ld.add_action(ros_gz_ld)
    ld.add_action(spawn_entity_node)
    
    return ld