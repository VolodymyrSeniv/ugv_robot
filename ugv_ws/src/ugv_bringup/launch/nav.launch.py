"""Nav2 (planner + controller + costmaps on RTAB-Map /map) and the mission manager."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ugv_navigation'), 'config', 'nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'params_file': params,
                          'autostart': 'true', 'use_composition': 'False'}.items())
    mission = Node(package='ugv_mission', executable='mission_manager', output='screen',
                   parameters=[{'use_sim_time': True,
                                'jump_guard': LaunchConfiguration('jump_guard')}])
    cmd_odom = Node(package='ugv_mission', executable='cmd_vel_odom', output='log',
                    parameters=[{'use_sim_time': True}])
    return LaunchDescription([
        DeclareLaunchArgument('jump_guard', default_value='true'),
        nav2, mission, cmd_odom])
