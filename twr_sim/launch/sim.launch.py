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
    # === robot_state_publisher ===
    # =============================
    rsp_pkg_path = FindPackageShare('twr_sim')
    rsp_ld_source = PythonLaunchDescriptionSource(
        [PathJoinSubstitution([rsp_pkg_path, 'launch', 'rsp.launch.py'])]
    )

    rsp_ld = IncludeLaunchDescription(
        rsp_ld_source,
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    
    # ===============
    # === GAZEBO ====
    # ===============
    gazebo_pkg_path = FindPackageShare('gazebo_ros')
    gazebo_ld_source = PythonLaunchDescriptionSource(
        [PathJoinSubstitution([gazebo_pkg_path, 'launch', 'gazebo.launch.py'])]
    )

    gazebo_ld = IncludeLaunchDescription(gazebo_ld_source)

    # === spawn_entity ===
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'twr_model']
    )

    ld.add_action(rsp_ld)
    ld.add_action(gazebo_ld)
    ld.add_action(spawn_entity_node)
    
    return ld