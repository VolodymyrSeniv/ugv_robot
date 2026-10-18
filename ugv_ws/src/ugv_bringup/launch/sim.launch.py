"""Gazebo Fortress simulation: world + robot + ros_gz bridge + robot_state_publisher.

world:=test_world          unique texture per wall/obstacle (default)
world:=test_world_aliased  one texture on every wall (perceptual aliasing, hard case for loop closure)
"""
import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    desc = get_package_share_directory('ugv_description')
    world_name = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

    # The world references textures by absolute path; fill it in for this install.
    with open(os.path.join(desc, 'worlds', world_name + '.sdf')) as f:
        sdf = f.read().replace('__TEXDIR__', os.path.join(desc, 'textures'))
    world = os.path.join(tempfile.gettempdir(), 'ugv_' + world_name + '.sdf')
    with open(world, 'w') as f:
        f.write(sdf)
    robot_xml = xacro.process_file(os.path.join(desc, 'urdf', 'ugv.urdf.xacro')).toxml()

    gz_cmd = ['ign', 'gazebo', '-s', '-r', '--headless-rendering', world] if headless \
        else ['ign', 'gazebo', '-r', world]
    gz = ExecuteProcess(cmd=gz_cmd, output='screen')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_xml, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'test_world', '-name', 'ugv',
                            '-string', robot_xml, '-z', '0.2'],
                 output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/stereo/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/stereo/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/stereo/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/stereo/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ])
    return [gz, rsp, spawn, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value='test_world'),
        OpaqueFunction(function=_setup),
    ])
