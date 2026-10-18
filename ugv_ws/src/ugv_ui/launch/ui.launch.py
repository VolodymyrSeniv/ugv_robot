"""Operator UI: rosbridge (ws :9090), static web server (http :8080) and the pose republisher.

Open http://<robot-or-sim-host>:8080/ in any browser.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    web = os.path.join(get_package_share_directory('ugv_ui'), 'web')
    sim = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket',
             parameters=[{'port': 9090, 'address': '', 'use_sim_time': sim}], output='log'),
        ExecuteProcess(cmd=['python3', '-m', 'http.server', '8080', '--directory', web], output='log'),
        Node(package='ugv_ui', executable='robot_pose_pub', output='log',
             parameters=[{'use_sim_time': sim}]),
    ])
