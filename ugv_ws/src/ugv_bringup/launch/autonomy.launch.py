"""Everything for a simulated autonomous run: sim + SLAM/EKF + Nav2 + mission manager."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ugv_bringup')

    def inc(name, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            launch_arguments=(args or {}).items())

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('world', default_value='test_world'),
        DeclareLaunchArgument('jump_guard', default_value='true'),
        DeclareLaunchArgument('publish_map_tf', default_value='true'),
        DeclareLaunchArgument('ui', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='/ws/data/rtabmap.db'),
        inc('sim.launch.py', {'headless': LaunchConfiguration('headless'),
                              'world': LaunchConfiguration('world')}),
        inc('slam.launch.py', {'mode': LaunchConfiguration('mode'),
                               'publish_map_tf': LaunchConfiguration('publish_map_tf'),
                               'database_path': LaunchConfiguration('database_path')}),
        inc('nav.launch.py', {'jump_guard': LaunchConfiguration('jump_guard')}),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ugv_ui'), 'launch', 'ui.launch.py')),
            condition=IfCondition(LaunchConfiguration('ui'))),
    ])
