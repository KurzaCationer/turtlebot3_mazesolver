#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mazesolver_launch_dir = os.path.join(
        get_package_share_directory("turtlebot3_mazesolver"), "launch"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mazesolver_launch_dir, "physical_world.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mazesolver_launch_dir, "physical_tb3.launch.py")
                ),
                launch_arguments={"name": "tb0"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mazesolver_launch_dir, "physical_tb3.launch.py")
                ),
                launch_arguments={"name": "tb1"}.items(),
            ),
        ]
    )
