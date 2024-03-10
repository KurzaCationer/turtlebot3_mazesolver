import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("turtlebot3_mazesolver"), "config", "params.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    known_init_poses = LaunchConfiguration("known_init_poses")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation/Gazebo clock",
            ),
            DeclareLaunchArgument(
                "known_init_poses",
                default_value="False",
                description="Known initial poses of the robots. If so don't forget to declare them in the params.yaml file",
            ),
            Node(
                package="multirobot_map_merge",
                name="map_merge",
                executable="map_merge",
                parameters=[
                    params,
                    {
                        "known_init_poses": known_init_poses,
                        "use_sim_time": use_sim_time,
                    },
                ],
                output="screen",
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
        ]
    )
