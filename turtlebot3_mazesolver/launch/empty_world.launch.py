#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    world_file_name = LaunchConfiguration("world_file_name")

    gazebo_ros = get_package_share_directory("gazebo_ros")

    world = PathJoinSubstitution(
        [get_package_share_directory("turtlebot3_gazebo"), "worlds", world_file_name]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_file_name",
                default_value="turtlebot3_world.world",
                description="Name of the world file",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros, "launch", "gzserver.launch.py")
                ),
                launch_arguments={"world": world}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros, "launch", "gzclient.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("multirobot_map_merge"),
                        "launch",
                        "map_merge.launch.py",
                    )
                ),
                launch_arguments={"known_init_poses": "false"}.items(),
            ),
        ]
    )
