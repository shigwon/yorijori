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
        Node(
            package='vehicle_contoller_pkg',
            executable='drive_controller_node',
            name='drive_controller',
            output='screen'
        ),

        Node(
            package='vehicle_contoller_pkg',
            executable='keyboard_drive_teleop',
            name='keyboard_teleop',
            output='screen'
        )
    ])
