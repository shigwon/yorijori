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

    # 노드 직접 실행 예시 1 - Steering + Throttle controller
    controller_node = Node(
        package='driving_pkg',
        executable='controller_node',
        name='controller',
        output='screen'
    )

    face_recognition_node = Node(
        package='face_recognitnion_pkg',
        executable='face_recognitnion_node',
        name='face_recognition',
        output='screen's
    )

    return LaunchDescription([
        # included_launch,
        controller_node,
        face_recognition_node
    ])
