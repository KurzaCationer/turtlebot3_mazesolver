#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    x_pose = LaunchConfiguration("x_pose", default="0")
    y_pose = LaunchConfiguration("y_pose", default="0")
    name = LaunchConfiguration("name", default="tb0")

    turtlebot3_mazesolver = get_package_share_directory("turtlebot3_mazesolver")

    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
    tb_model = "turtlebot3_" + TURTLEBOT3_MODEL

    urdf_path = os.path.join(turtlebot3_mazesolver, "urdf", tb_model + ".urdf")
    robot_desc = Command(
        ["xacro ", str(urdf_path), " frame_prefix:=", name, " topic_prefix:=", name]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "x_pose",
                default_value="0",
                description="Specify namespace of the robot",
            ),
            DeclareLaunchArgument(
                "y_pose",
                default_value="0",
                description="Specify namespace of the robot",
            ),
            DeclareLaunchArgument(
                "name",
                default_value="tb0",
                description="The name that will be used as part of the frames and namespaces",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_mazesolver, "launch", "slam.launch.py")
                ),
                launch_arguments={"use_sim_time": "True", "name": name}.items(),
            ),
            Node(
                package="robot_state_publisher",
                namespace=["/", name],
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": robot_desc,
                        "publish_frequency": 10.0,
                        "frame_prefix": ["/", name, "/"],
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", [name, "/map"]],
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
