from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    return LaunchDescription([
        included_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            )
        )

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
            package='webrtc_streamer_pkg',
            executable='webrtc_streamer_node',
            name='webrtc_streamer',
            output='screen'
        ),

        Node(
            package='slam_pkg',
            executable='slam_node',
            name='slam',
            output='screen'
        )
    ])
