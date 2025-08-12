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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_linky'),
                    'launch',
                    'realsense2_linky.launch.py'
                ])
            )
        ),

        Node(
            package='face_recognition_pkg',
            executable='face_db_matcher_node',
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
    ])
