#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_folder = os.path.join(
        get_package_share_directory("turtlebot3_mazesolver"), "rviz"
    )

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(rviz_folder, "merged_map.rviz")],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(rviz_folder, "tb0_map.rviz")],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(rviz_folder, "tb1_map.rviz")],
                output="screen",
            ),
        ]
    )
