# example.launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=[
                '0.0',    # x translation
                '0.0',    # y translation
                '0.0',    # z translation
                '0.0',    # x rotation
                '0.0',    # y rotation
                '1.0',    # z rotation
                'world_origin',  # Parent frame ID
                'marker_0'     # Child frame ID
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=[
                '0.0',    # x translation
                '0.0',    # y translation
                '0.0',    # z translation
                '0.0',    # x rotation
                '0.0',    # y rotation
                '1.0',    # z rotation
                'world_origin',  # Parent frame ID
                'marker_1'     # Child frame ID
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=[
                '0.0',    # x translation
                '0.0',    # y translation
                '0.0',    # z translation
                '0.0',    # x rotation
                '0.0',    # y rotation
                '1.0',    # z rotation
                'world_origin',  # Parent frame ID
                'marker_2'     # Child frame ID
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=[
                '0.0',    # x translation
                '0.0',    # y translation
                '0.0',    # z translation
                '0.0',    # x rotation
                '0.0',    # y rotation
                '1.0',    # z rotation
                'world_origin',  # Parent frame ID
                'marker_3'     # Child frame ID
            ]
        ),
    ])
