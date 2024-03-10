#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    turtlebot3_mazesolver = get_package_share_directory("turtlebot3_mazesolver")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        turtlebot3_mazesolver,
                        "launch",
                        "common",
                        "map_merge.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "False",
                    "known_init_poses": "false",
                }.items(),
            ),
        ]
    )
