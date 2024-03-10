#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    world_file_name = LaunchConfiguration(
        "world_file_name", default="turtlebot3_world.world"
    )

    gazebo_ros = get_package_share_directory("gazebo_ros")
    turtlebot3_mazesolver = get_package_share_directory("turtlebot3_mazesolver")

    world = PathJoinSubstitution([turtlebot3_mazesolver, "worlds", world_file_name])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_file_name",
                default_value=world_file_name,
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
                        turtlebot3_mazesolver,
                        "launch",
                        "common",
                        "map_merge.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "True",
                    "known_init_poses": "false",
                }.items(),
            ),
        ]
    )
