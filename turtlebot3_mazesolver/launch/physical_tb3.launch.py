#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    LDS_MODEL = os.environ["LDS_MODEL"]

    usb_port = LaunchConfiguration("usb_port", default="/dev/ttyACM0")
    name = LaunchConfiguration("name", default="tb0")

    turtlebot3_mazesolver = get_package_share_directory("turtlebot3_mazesolver")

    params = os.path.join(
        turtlebot3_mazesolver,
        "config",
        "params.yaml",
    )

    if LDS_MODEL == "LDS-02":
        lidar_node = Node(
            package="ld08_driver",
            executable="ld08_driver",
            namespace=["/", name],
            name="ld08_driver",
            output="screen",
        )
    else:
        lidar_node = Node(
            package="hls_lfcd_lds_driver",
            executable="hlds_laser_publisher",
            name="hlds_laser_publisher",
            namespace=["/", name],
            parameters=[{"port": "/dev/ttyUSB0", "frame_id": [name, "/base_scan"]}],
            output="screen",
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "name",
                default_value=name,
                description="Name of the robot",
            ),
            DeclareLaunchArgument(
                "usb_port",
                default_value=usb_port,
                description="Connected USB port with OpenCR",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        turtlebot3_mazesolver,
                        "launch",
                        "common",
                        "tb3.launch.py",
                    )
                ),
                launch_arguments={"use_sim_time": "False", "name": name}.items(),
            ),
            lidar_node,
            Node(
                package="turtlebot3_node",
                executable="turtlebot3_ros",
                namespace=["/", name],
                parameters=[params],
                arguments=["-i", usb_port],
                output="screen",
            ),
        ]
    )
