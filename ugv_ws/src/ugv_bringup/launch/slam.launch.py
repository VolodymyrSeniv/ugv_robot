"""Stereo visual odometry -> EKF (VO + IMU) -> RTAB-Map SLAM (map->odom + /map grid).

mode:=mapping       build a new map (deletes the old database)
mode:=localization  reuse the saved database, no new nodes
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ekf_cfg = os.path.join(get_package_share_directory('ugv_localization'), 'config', 'ekf.yaml')
    mode = LaunchConfiguration('mode')
    db = LaunchConfiguration('database_path')

    common = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.02,
        'queue_size': 20,
    }
    remap = [
        ('left/image_rect', '/stereo/left/image_raw'),
        ('right/image_rect', '/stereo/right/image_raw'),
        ('left/camera_info', '/stereo/left/camera_info'),
        ('right/camera_info', '/stereo/right/camera_info'),
    ]

    # Planar robot on (mostly) flat ground: constrain to 3 DoF.
    planar = {'Reg/Force3DoF': 'true', 'Optimizer/Slam2D': 'true'}

    vo = Node(
        package='rtabmap_odom', executable='stereo_odometry', output='screen',
        parameters=[common, {'publish_tf': False, 'subscribe_odom_info': False,
                            # a lost frame must not reach the EKF as a (0,0) pose with zero covariance
                            'publish_null_when_lost': False,
                            # EKF pose as motion prior, and restart VO after a lost frame
                            'guess_frame_id': 'odom'},
                    planar, {'Odom/Strategy': '0', 'Vis/MinInliers': '15', 'Odom/ResetCountdown': '1'}],
        remappings=remap + [('odom', '/vo/odom')])

    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
               output='screen', parameters=[ekf_cfg],
               remappings=[('odometry/filtered', '/odometry/filtered')])

    rtabmap_params = {
        'database_path': db,
        'publish_tf': LaunchConfiguration('publish_map_tf'),
        'map_frame_id': 'map',
        'subscribe_odom_info': False,
        'Rtabmap/DetectionRate': '2',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '6.0',
        'Grid/MaxObstacleHeight': '1.2',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/NormalsSegmentation': 'false',
        'Grid/RayTracing': 'true',
        'Grid/3D': 'false',
        # Loop closures: the arena is nearly featureless, so a weak match is usually a false one
        # (a closure with 81 deg of rotation was accepted at 20 inliers while VO was healthy).
        'Vis/MinInliers': '40',
        'Vis/MinInliersDistribution': '0.05',
        'Vis/PnPReprojError': '1.0',
        'Rtabmap/LoopThr': '0.2',
        'RGBD/OptimizeMaxError': '2.0',
    }
    slam_map = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
        parameters=[common, rtabmap_params, planar, {'Mem/IncrementalMemory': 'true'}],
        remappings=remap, arguments=['-d'])
    slam_loc = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
        parameters=[common, rtabmap_params, planar,
                    {'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true',
                     # the robot is placed at the map origin (start pose), not where the last session ended
                     'RGBD/StartAtOrigin': 'true'}],
        remappings=remap)

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('database_path', default_value='/ws/data/rtabmap.db'),
        DeclareLaunchArgument('publish_map_tf', default_value='true'),
        vo, ekf, slam_map, slam_loc,
    ])
