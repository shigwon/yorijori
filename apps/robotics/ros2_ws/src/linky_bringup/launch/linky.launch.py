from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True}]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/infra1/image_rect_raw'),
          ('rgb/camera_info', '/camera/infra1/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw')]
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_linky'),
                    'launch',
                    'realsense2_linky.py'
                ])
            )
        ),

        Node(
            package='face_recognition_pkg',
            executable='face_recognition_server_node',
            name='face_recognition',
            output='screen'
        ),
        
        Node(
            package='mqtt_bridge_pkg',
            executable='mqtt_bridge_node',
            name='mqtt_bridge',
            output='screen'
        ),
        Node(
            package='vehicle_contoller_pkg',
            executable='drive_controller_node',
            name='drive_controller',
            output='screen'
        ),

        Node(
            package='slam_pkg',
            executable='slam_node',
            name='slam',
            output='screen'
        )
    ])
