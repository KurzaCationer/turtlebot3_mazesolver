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
    
    config_file_dir = os.path.join(turtlebot3_mazesolver, "config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
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
            DeclareLaunchArgument("name", default_value="tb0", description="name"),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                namespace=["/", name],
                remappings=[
                    ("/scan", "scan"),
                    ("/map", "map"),
                    ("/odom", "odom"),
                    ("/map_metadata", "map_metadata"),
                    (
                        "/slam_toolbox/scan_visualization",
                        "slam_toolbox/scan_visualization",
                    ),
                    (
                        "/slam_toolbox/graph_visualization",
                        "slam_toolbox/graph_visualization",
                    ),
                ],
                parameters=[
                    os.path.join(config_file_dir, "slam_params_online_async.yaml"),
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
