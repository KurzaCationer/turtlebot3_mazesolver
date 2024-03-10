#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    name = LaunchConfiguration("name", default="tb0")
    resolution = LaunchConfiguration("resolution", default="0.05")
    publish_period_sec = LaunchConfiguration("publish_period_sec", default="1.0")

    turtlebot3_mazesolver = get_package_share_directory("turtlebot3_mazesolver")

    params = os.path.join(turtlebot3_mazesolver, "config", "params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument("name", default_value=name, description="name"),
            DeclareLaunchArgument(
                "resolution",
                default_value=resolution,
                description="Resolution of a grid cell in the published occupancy grid",
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value=publish_period_sec,
                description="OccupancyGrid publishing period",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", [name, "/map"]],
            ),
            # Node(
            #     package="slam_gmapping",
            #     name="slam_gmapping",
            #     namespace=["/", name],
            #     executable="slam_gmapping",
            #     output="screen",
            #     parameters=[params, {"use_sim_time": use_sim_time}],
            # ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                remappings=[
                    ("/scan", ["/", name, "/scan"]),
                    ("/map", ["/", name, "/map"]),
                    ("/odom", ["/", name, "/odom"]),
                    ("/map_metadata", ["/", name, "/map_metadata"]),
                    (
                        "/slam_toolbox/scan_visualization",
                        ["/", name, "/slam_toolbox/scan_visualization"],
                    ),
                    (
                        "/slam_toolbox/graph_visualization",
                        ["/", name, "/slam_toolbox/graph_visualization"],
                    ),
                ],
                parameters=[
                    params,
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": [name, "/odom"],
                        "map_frame": [name, "/map"],
                        "base_frame": [name, "/base_footprint"],
                    },
                ],
            ),
        ]
    )
