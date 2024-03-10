#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
    tb_model = "turtlebot3_" + TURTLEBOT3_MODEL

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    name = LaunchConfiguration("name", default="tb0")

    turtlebot3_mazesolver = get_package_share_directory("turtlebot3_mazesolver")

    urdf_path = os.path.join(turtlebot3_mazesolver, "urdf", tb_model + ".urdf")
    robot_desc = Command(
        ["xacro ", str(urdf_path), " frame_prefix:=", name, " topic_prefix:=", name]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation time (/clock)",
            ),
            DeclareLaunchArgument(
                "name",
                default_value=name,
                description="The name that will be used as part of the frames and namespaces",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        turtlebot3_mazesolver, "launch", "common", "slam.launch.py"
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time, "name": name}.items(),
            ),
            Node(
                package="robot_state_publisher",
                namespace=["/", name],
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_desc,
                        "publish_frequency": 10.0,
                        "frame_prefix": ["/", name, "/"],
                    }
                ],
            ),
        ]
    )
