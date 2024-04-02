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
    rsp_ld_source = PythonLaunchDescriptionSource([
        PathJoinSubstitution([rsp_pkg_path, 'launch', 'rsp.launch.py'])
    ])

    rsp_ld = IncludeLaunchDescription(
        rsp_ld_source,
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    
    # =============
    # === RViz2 ===
    # =============
    twr_description_pkg_path = get_package_share_directory('twr_description')

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

    #  === joint_state_publisher ===
    # provide JointState messages for robot_state_publisher,
    # also necessary for correct representation of axes in rviz2
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    ld.add_action(rsp_ld)
    ld.add_action(rviz2_node)
    ld.add_action(jsp_node)
    
    return ld