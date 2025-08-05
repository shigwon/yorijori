from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    # 다른 패키지의 launch 파일 포함
    # included_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('other_pkg_name'),
    #             'launch',
    #             'other_launch_file.launch.py'
    #         ])
    #     )
    # )

    controller_node = Node(
        package='contoller_pkg',
        executable='controller_node',
        name='controller',
        output='screen'
    )

    face_recognition_server_node = Node(
        package='face_recognition_pkg',
        executable='face_recognition_server_node',
        name='face_recognition',
        output='screen'
    )

    mqtt_bridge_node = Node(
        package='mqtt_bridge_pkg',
        executable='mqtt_bridge_node',
        name='mqtt_bridge',
        output='screen'
    )

    return LaunchDescription([
        # included_launch,
        controller_node,
        face_recognition_server_node,
        mqtt_bridge_node
    ])
