from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

import math


def generate_launch_description():

    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
    )
    
    odometry_node = LifecycleNode(
        package="amr_turtlebot3",
        executable="odometry_node",
        name="odometry_node",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    # coppeliasim_node = LifecycleNode(
    #     package="amr_simulation",
    #     executable="coppeliasim",
    #     name="coppeliasim",
    #     namespace="",
    #     output="screen",
    #     arguments=["--ros-args", "--log-level", "WARN"],
    #     parameters=[{"start": start}],
    # )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": (
                    "wall_follower",
                    "odometry_node",
                    # "coppeliasim",  # Must be started last
                )
            }
        ],
    )

    return LaunchDescription(
        [
            wall_follower_node,
            odometry_node,
            # coppeliasim_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )
