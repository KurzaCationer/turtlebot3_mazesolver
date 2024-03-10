#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    x_pose = LaunchConfiguration("x_pose", default="0")
    y_pose = LaunchConfiguration("y_pose", default="0")
    name = LaunchConfiguration("name", default="tb0")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "x_pose",
                default_value=x_pose,
                description="Specify namespace of the robot",
            ),
            DeclareLaunchArgument(
                "y_pose",
                default_value=y_pose,
                description="Specify namespace of the robot",
            ),
            DeclareLaunchArgument(
                "name",
                default_value=name,
                description="The name that will be used as part of the frames and namespaces",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("turtlebot3_mazesolver"),
                        "launch",
                        "common",
                        "tb3.launch.py",
                    )
                ),
                launch_arguments={"use_sim_time": "True", "name": name}.items(),
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic",
                    ["/", name, "/", "robot_description"],
                    "-entity",
                    name,
                    "-robot_namespace",
                    ["/", name],
                    "-x",
                    x_pose,
                    "-y",
                    y_pose,
                    "-z",
                    "0.01",
                ],
                output="screen",
            ),
        ]
    )
