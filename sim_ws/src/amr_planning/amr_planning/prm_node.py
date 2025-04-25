import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from amr_msgs.msg import PoseStamped as AmrPoseStamped  # Custom message for robot pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import os
import time
import traceback
from typing import List, Tuple

from amr_planning.prm import PRM  # Custom PRM class for path planning


class PRMNode(LifecycleNode):
    """
    A ROS2 lifecycle node implementing a Probabilistic Roadmap (PRM) for robot path planning.

    This node handles roadmap creation, pathfinding, and path smoothing using PRM. It transitions
    through lifecycle states and interacts with publishers and subscribers to provide planned paths.
    """

    def __init__(self) -> None:
        """
        Initializes the PRMNode with configurable parameters.

        The node declares parameters for roadmap creation, path smoothing, and visualization,
        and sets up the lifecycle node.
        """
        super().__init__("probabilistic_roadmap")

        # Declare parameters for roadmap creation and path smoothing
        self.declare_parameter(
            "connection_distance", 0.3
        )  # Max distance to connect nodes in the roadmap
        self.declare_parameter(
            "enable_plot", False
        )  # Enable visualization of paths and roadmap
        self.declare_parameter(
            "goal", (0.0, 0.0)
        )  # Goal position for pathfinding (x, y)
        self.declare_parameter(
            "grid_size", 0.05
        )  # Grid resolution for roadmap generation
        self.declare_parameter("node_count", 250)  # Number of nodes in the roadmap
        self.declare_parameter(
            "obstacle_safety_distance", 0.12
        )  # Safety distance from obstacles [m]
        self.declare_parameter(
            "smoothing_additional_points", 3
        )  # Extra points added during smoothing
        self.declare_parameter(
            "smoothing_data_weight", 0.1
        )  # Weight for data fidelity during smoothing
        self.declare_parameter(
            "smoothing_smooth_weight", 0.3
        )  # Weight for smoothness during smoothing
        self.declare_parameter(
            "use_grid", False
        )  # Use grid-based roadmap generation if True
        self.declare_parameter("world", "project")  # Name of the map file to load

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Handles the transition to the 'inactive' state.

        This method initializes the PRM object, loads map data, sets up publishers and subscribers,
        and logs relevant information about roadmap creation.

        Args:
            state (LifecycleState): Current lifecycle state.

        Returns:
            TransitionCallbackReturn: Result of the transition (SUCCESS or ERROR).
        """
        self.get_logger().info(
            f"Transitioning from '{state.label}' to 'inactive' state."
        )

        try:
            # Parameters
            connection_distance = (
                self.get_parameter("connection_distance")
                .get_parameter_value()
                .double_value
            )
            self._enable_plot = (
                self.get_parameter("enable_plot").get_parameter_value().bool_value
            )
            self._goal = tuple(
                self.get_parameter("goal")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            grid_size = (
                self.get_parameter("grid_size").get_parameter_value().double_value
            )
            node_count = (
                self.get_parameter("node_count").get_parameter_value().integer_value
            )
            obstacle_safety_distance = (
                self.get_parameter("obstacle_safety_distance")
                .get_parameter_value()
                .double_value
            )
            self._smoothing_additional_points = (
                self.get_parameter("smoothing_additional_points")
                .get_parameter_value()
                .integer_value
            )
            self._smoothing_data_weight = (
                self.get_parameter("smoothing_data_weight")
                .get_parameter_value()
                .double_value
            )
            self._smoothing_smooth_weight = (
                self.get_parameter("smoothing_smooth_weight")
                .get_parameter_value()
                .double_value
            )
            use_grid = self.get_parameter("use_grid").get_parameter_value().bool_value
            world = self.get_parameter("world").get_parameter_value().string_value

            # Initialize PRM object with map data and parameters
            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )
            self._localized = False

            start_time = time.perf_counter()
            self._planning = PRM(
                map_path,
                obstacle_safety_distance,
                use_grid,
                node_count,
                grid_size,
                connection_distance,
            )
            roadmap_creation_time = time.perf_counter() - start_time

            # Log roadmap creation time for debugging purposes
            self.get_logger().info(
                f"Roadmap creation time: {roadmap_creation_time:1.3f} s"
            )

            # Create a publisher for the path
            self._path_publisher = self.create_publisher(Path, "/path", 10)

            # Create a subscription to the robot's pose
            self._subscriber_pose = self.create_subscription(
                AmrPoseStamped, "/pose", self._path_callback, 10
            )

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Handles the transition to the 'active' state.

        Args:
            state (LifecycleState): Current lifecycle state.

        Returns:
            TransitionCallbackReturn: Result of the transition (SUCCESS).
        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")
        return super().on_activate(state)

    def _path_callback(self, pose_msg: AmrPoseStamped) -> None:
        """
        Subscriber callback to compute and publish a smoothed path to the goal.

        This method uses A* search to find a path from the robot's current position to the goal.
        The path is then smoothed using a weighted algorithm before being published.

        Args:
            pose_msg (AmrPoseStamped): Message containing the robot's current pose estimate.
        """
        if pose_msg.localized and not self._localized:
            start = (pose_msg.pose.position.x, pose_msg.pose.position.y)

            # Perform pathfinding using A* search in the PRM roadmap
            path = self._planning.find_path(start, self._goal)

            # Smooth the computed path using a weighted algorithm
            smoothed_path = PRM.smooth_path(
                path=path,
                data_weight=self._smoothing_data_weight,
                smooth_weight=self._smoothing_smooth_weight,
                additional_smoothing_points=self._smoothing_additional_points,
            )

            if self._enable_plot:
                # Visualize paths if plotting is enabled in parameters
                self._planning.show(
                    path=path, smoothed_path=smoothed_path, save_figure=True
                )

            # Publish the smoothed path as a Path message
            self._publish_path(smoothed_path)

        # Update localization status based on pose message data
        self._localized = pose_msg.localized

    def _publish_path(self, path: List[Tuple[float, float]]) -> None:
        """
        Publishes a smoothed path as a nav_msgs.msg.Path message.

        Args:
            path (List[Tuple[float, float]]): Smoothed path represented as a list of (x, y) coordinates.
                                              The first point corresponds to the robot's initial location.
        """
        # Create Path message with header timestamp and poses list populated with waypoints from the smoothed path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = []
        for point in path:
            pose_stamped: PoseStamped = PoseStamped()
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]

            path_msg.poses.append(pose_stamped)  # Add the pose to the Path message

        # Publish the Path message
        self._path_publisher.publish(path_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    prm_node: PRMNode = PRMNode()

    try:
        rclpy.spin(prm_node)
    except KeyboardInterrupt:
        pass

    prm_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
