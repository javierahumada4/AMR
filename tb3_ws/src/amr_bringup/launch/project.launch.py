from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    world = "project"
    goal = (1.0, -1.0)

    particle_filter_node = LifecycleNode(
        package="amr_localization",
        executable="particle_filter",
        name="particle_filter",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "enable_plot": False,
                "global_localization": True,
                "particles": 5000,
                "sigma_v": 0.05,
                "sigma_w": 0.1,
                "sigma_z": 0.2,
                "world": world,
            }
        ],
    )

    probabilistic_roadmap_node = LifecycleNode(
        package="amr_planning",
        executable="probabilistic_roadmap",
        name="probabilistic_roadmap",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "connection_distance": 0.15,
                "enable_plot": True,
                "goal": goal,
                "grid_size": 0.1,
                "node_count": 250,
                "obstacle_safety_distance": 0.16,
                "smoothing_additional_points": 0,
                "smoothing_data_weight": 1.0,
                "smoothing_smooth_weight": 0.0,
                "use_grid": True,
                "world": world,
            }
        ],
    )

    odometry_node = LifecycleNode(
        package="amr_turtlebot3",
        executable="odometry_node",
        name="odometry_node",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    pure_pursuit_node = LifecycleNode(
        package="amr_control",
        executable="pure_pursuit",
        name="pure_pursuit",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[{"lookahead_distance": 0.2}],
    )

    monitoring_node = LifecycleNode(
        package="amr_turtlebot3",
        executable="monitoring",
        name="monitoring",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "goal": goal,
                "goal_tolerance": 0.1,
            }
        ],
    )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": (
                    "particle_filter",
                    "probabilistic_roadmap",
                    "odometry_node",
                    "wall_follower",
                    "pure_pursuit",
                    "monitoring",
                )
            }
        ],
    )

    return LaunchDescription(
        [
            particle_filter_node,
            probabilistic_roadmap_node,
            odometry_node,
            wall_follower_node,
            pure_pursuit_node,
            monitoring_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )
