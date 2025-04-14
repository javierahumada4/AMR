import datetime
import math
import os
import pytz
import random
import time

# This try-except enables local debugging of the PRM class
try:
    from amr_planning.maps import Map
except ImportError:
    from maps import Map

from matplotlib import pyplot as plt


class PathNotFound(Exception):
    """Custom exception raised when no path can be found."""

    pass


class PRM:
    """
    Implements probabilistic roadmaps (PRM) for robot path planning.

    The PRM class generates a graph of nodes representing the environment and uses the A* algorithm
    to find a path from a starting location to a goal location. It supports both random sampling
    and grid-based node generation.
    """

    def __init__(
        self,
        map_path: str,
        obstacle_safety_distance=0.08,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
        sensor_range_max: float = 8.0,
    ):
        """
        Initializes the PRM object and generates the roadmap graph.

        Args:
            map_path (str): Path to the map of the environment.
            obstacle_safety_distance (float): Distance to grow obstacles by [m].
            use_grid (bool): If True, generates nodes in a grid layout; otherwise, uses random sampling.
            node_count (int): Number of random nodes to generate (used only if `use_grid` is False).
            grid_size (float): Distance between consecutive nodes in x and y directions (used only if `use_grid` is True).
            connection_distance (float): Maximum distance to connect two nodes with an edge [m].
            sensor_range_max (float): Maximum range of sensor measurements [m].
        """
        # Load the map and apply obstacle safety distance
        self._map: Map = Map(
            map_path,
            sensor_range=sensor_range_max,
            safety_distance=obstacle_safety_distance,
            compiled_intersect=False,
            use_regions=False,
        )

        # Create the roadmap graph based on the selected sampling method
        self._graph: dict[tuple[float, float], list[tuple[float, float]]] = self._create_graph(
            use_grid, node_count, grid_size, connection_distance
        )

        # Initialize visualization components for debugging or monitoring purposes
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))

        # Timestamp for saving figures or logs with unique identifiers
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def find_path(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> list[tuple[float, float]]:
        """
        Finds the shortest path from a start location to a goal location using the A* algorithm.

        This method checks if both the start and goal locations are valid within the map boundaries.
        If necessary, it adjusts the start location to a nearby valid point. The A* algorithm is then
        used to find the shortest path between nodes in the roadmap graph.

        Args:
            start (tuple[float, float]): Initial location in (x, y) format [m].
            goal (tuple[float, float]): Destination location in (x, y) format [m].

        Returns:
            list[tuple[float, float]]: Path from start to goal as a list of (x, y) coordinates.
                                       The first point corresponds to the initial location.

        Raises:
            ValueError: If either the start or goal location is outside the environment.
            PathNotFound: If no path can be found between the start and goal locations.
        """

        def calculate_dis(node, pos):
            """Helper function to calculate Euclidean distance between two points."""
            return math.sqrt((node[0] - pos[0]) ** 2 + (node[1] - pos[1]) ** 2)

        # Validate and adjust start location if necessary
        if not self._map.contains(start):
            closest_point = None
            closest_distance = float("inf")

            # Try finding a nearby valid point within 100 attempts
            for _ in range(100):
                x = random.gauss(start[0], 0.1)
                y = random.gauss(start[1], 0.1)
                point = (x, y)
                if self._map.contains(point):
                    distance = calculate_dis(start, point)
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_point = point

            if closest_point is not None:
                start = closest_point  # Adjust start location to nearby valid point
            else:
                raise ValueError("Start location is outside the environment.")

        # Validate goal location
        if not self._map.contains(goal):
            raise ValueError("Goal location is outside the environment.")

        # Initialize data structures for A* algorithm
        ancestors: dict[tuple[float, float], tuple[float, float]] = {}  # Tracks path reconstruction
        path: list[tuple[float, float]] = []  # Stores final path

        initial_node = min(
            self._graph, key=lambda node: calculate_dis(node, start)
        )  # Closest graph node to start
        final_node = min(
            self._graph, key=lambda node: calculate_dis(node, goal)
        )  # Closest graph node to goal

        open_list: dict[
            tuple[float, float], tuple[float, float]
        ] = {}  # Nodes to explore with their f and g values
        close_list: set[tuple[float, float]] = set()  # Explored nodes

        open_list[initial_node] = (
            calculate_dis(initial_node, final_node),
            0,
        )  # Initialize open list with start node

        node = initial_node

        while open_list and node != final_node:
            # Select node with lowest f value from open list
            node = min(open_list.keys(), key=lambda k: open_list[k][0])

            f, g = open_list[node]  # Retrieve f and g values for current node

            for neighbour in self._graph[node]:
                h_new = calculate_dis(neighbour, final_node)  # Heuristic cost to goal
                g_new = g + calculate_dis(
                    neighbour, node
                )  # Cost from start to neighbor via current node
                f_new = h_new + g_new  # Total cost

                if neighbour not in close_list:
                    if neighbour not in open_list or open_list[neighbour][1] > g_new:
                        open_list[neighbour] = (f_new, g_new)  # Update cost values for neighbor
                        ancestors[neighbour] = node  # Track ancestor for path reconstruction

            close_list.add(node)  # Mark current node as explored
            open_list.pop(node)  # Remove current node from open list

        if final_node in close_list:
            if initial_node != start:
                ancestors[initial_node] = start  # Add direct link from adjusted start position

            if final_node != goal:
                ancestors[goal] = final_node  # Add direct link to adjusted goal position

            path = self._reconstruct_path(
                start, goal, ancestors
            )  # Reconstruct path using ancestors

            if len(path) > 1 and calculate_dis(start, path[1]) < 0.05:
                path.pop(1)  # Remove first intermediate node for smoother transition

            return path  # Return reconstructed path

        else:
            raise PathNotFound("Path not found")  # Raise exception if no valid path exists

    @staticmethod
    def smooth_path(
        path,
        data_weight: float = 0.1,
        smooth_weight: float = 0.3,
        additional_smoothing_points: int = 0,
        tolerance: float = 1e-6,
        logger=None,
    ) -> list[tuple[float, float]]:
        """
        Smooths a piecewise linear path using an iterative optimization algorithm.

        This method adds intermediate points to the original path (if specified), then iteratively adjusts
        the path to balance data fidelity and smoothness. The smoothing process stops when changes between
        iterations fall below a given tolerance.

        Args:
            path (list[tuple[float, float]]): Original non-smoothed path (first point is the start location).
            data_weight (float): Weight for maintaining similarity to the original path. Larger values preserve the original shape.
            smooth_weight (float): Weight for achieving smoothness in the output path. Larger values result in smoother paths.
            additional_smoothing_points (int): Number of intermediate points to add between consecutive nodes of the original path.
            tolerance (float): Threshold for stopping the smoothing process. Iterations stop when changes fall below this value.
            logger: Logger object for debugging and logging information.

        Returns:
            list[tuple[float, float]]: Smoothed path with the same start and end locations as the original path.
        """
        logger.info(f"Path: {path}")

        # Initialize the extended path by adding intermediate points if specified
        extended_path: list[tuple[float, float]] = path.copy()

        if additional_smoothing_points > 0:
            for i, point in enumerate(path[:-1]):
                x_start, y_start = point
                x_end, y_end = path[i + 1]
                x_diff = (x_end - x_start) / (additional_smoothing_points + 1)
                y_diff = (y_end - y_start) / (additional_smoothing_points + 1)
                for j in range(1, additional_smoothing_points + 1):
                    extended_path.insert(
                        i * (additional_smoothing_points + 1) + j,
                        (x_start + j * x_diff, y_start + j * y_diff),
                    )

        logger.info(f"Extended Path: {extended_path}")

        # Initialize smoothed path and change tracker
        smoothed_path: list[tuple[float, float]] = extended_path.copy()
        change = float("inf")

        # Iteratively adjust the smoothed path until changes fall below tolerance
        while change >= tolerance:
            change = 0.0
            new_smoothed_path = smoothed_path.copy()

            # Update each point in the smoothed path except the start and end points
            for i in range(1, len(smoothed_path) - 1):
                x = smoothed_path[i][0]
                x += data_weight * (extended_path[i][0] - smoothed_path[i][0])
                x += smooth_weight * (
                    smoothed_path[i - 1][0] + smoothed_path[i + 1][0] - 2 * smoothed_path[i][0]
                )

                y = smoothed_path[i][1]
                y += data_weight * (extended_path[i][1] - smoothed_path[i][1])
                y += smooth_weight * (
                    smoothed_path[i - 1][1] + smoothed_path[i + 1][1] - 2 * smoothed_path[i][1]
                )

                new_smoothed_path[i] = (x, y)

                # Track total change between iterations
                change += abs(new_smoothed_path[i][0] - smoothed_path[i][0]) + abs(
                    new_smoothed_path[i][1] - smoothed_path[i][1]
                )

            # Update the smoothed path for the next iteration
            smoothed_path = new_smoothed_path

        logger.info(f"Smoothed Path: {smoothed_path}")

        return smoothed_path

    def plot(
        self,
        axes,
        path: list[tuple[float, float]] = (),
        smoothed_path: list[tuple[float, float]] = (),
    ):
        """
        Visualizes the roadmap graph, original path, and smoothed path on a given set of axes.

        This method plots nodes and edges of the roadmap graph along with paths provided as input.
        It distinguishes between original and smoothed paths using different colors and markers.

        Args:
            axes: Matplotlib axes object where plots will be drawn.
            path (list[tuple[float, float]]): Original piecewise linear path (first point is the start location).
            smoothed_path (list[tuple[float, float]]): Smoothed version of the original path.

        Returns:
            axes: Modified axes object containing the plotted roadmap graph and paths.
        """
        # Plot nodes of the roadmap graph
        x, y = zip(*self._graph.keys())
        axes.plot(list(x), list(y), "co", markersize=1)  # Nodes as cyan dots

        # Plot edges of the roadmap graph
        for node, neighbors in self._graph.items():
            x_start, y_start = node

            if neighbors:
                for x_end, y_end in neighbors:
                    axes.plot(
                        [x_start, x_end], [y_start, y_end], "c-", linewidth=0.25
                    )  # Edges as cyan lines

        # Plot original piecewise linear path
        if path:
            x_val = [x[0] for x in path]
            y_val = [x[1] for x in path]

            axes.plot(x_val, y_val)  # Path as a line
            axes.plot(x_val[1:-1], y_val[1:-1], "bo", markersize=4)  # Nodes as blue circles

        # Plot smoothed path
        if smoothed_path:
            x_val = [x[0] for x in smoothed_path]
            y_val = [x[1] for x in smoothed_path]

            axes.plot(x_val, y_val, "y")  # Smoothed path as a yellow line
            axes.plot(x_val[1:-1], y_val[1:-1], "yo", markersize=2)  # Nodes as yellow circles

        if path or smoothed_path:
            # Mark start location with a red square and goal location with a green star
            axes.plot(x_val[0], y_val[0], "rs", markersize=7)
            axes.plot(x_val[-1], y_val[-1], "g*", markersize=12)

        return axes

    def show(
        self,
        title: str = "",
        path=(),
        smoothed_path=(),
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """
        Displays or saves a visualization of the roadmap, path, and smoothed path.

        This method plots the roadmap graph along with the original and smoothed paths. It can display
        the visualization in a window, save it as an image file, or both.

        Args:
            title (str): Title of the plot.
            path (list[tuple[float, float]]): Original path (start location first).
            smoothed_path (list[tuple[float, float]]): Smoothed path (start location first).
            display (bool): If True, opens a window to visualize the plot in real-time.
                            Note: This can be time-consuming and may not work inside containers without screen forwarding.
            block (bool): If True, blocks program execution until the figure window is closed.
            save_figure (bool): If True, saves the plot as a .png file.
            save_dir (str): Directory where the image will be saved if `save_figure` is True.
        """
        figure = self._figure  # Get the figure object
        axes = self._axes  # Get the axes object
        axes.clear()  # Clear previous plots on the axes

        # Plot the map and paths on the axes
        axes = self._map.plot(axes)
        axes = self.plot(axes, path, smoothed_path)

        # Set plot title and adjust layout
        axes.set_title(title)
        figure.tight_layout()  # Reduce white margins around the plot

        if display:
            # Display the plot in a window
            plt.show(block=block)
            plt.pause(0.001)  # Pause briefly to ensure the plot is displayed

        if save_figure:
            # Save the figure to a file
            save_path = os.path.join(os.path.dirname(__file__), "..", save_dir)

            if not os.path.isdir(save_path):
                os.makedirs(save_path)  # Create directory if it doesn't exist

            file_name = f"{self._timestamp} {title.lower()}.png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)  # Save the figure as a PNG file

    def _connect_nodes(
        self,
        graph: dict[tuple[float, float], list[tuple[float, float]]],
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """
        Connects nodes in a graph based on proximity and obstacle-free paths.

        This method iterates through all nodes in the graph and connects each node to its neighbors
        that are within a specified distance and do not have obstacles between them.

        Args:
            graph (dict[tuple[float, float], list[tuple[float, float]]]): Graph with nodes as keys
                                                                        and empty lists as values.
            connection_distance (float): Maximum distance to consider adding an edge between two nodes [m].

        Returns:
            dict[tuple[float, float], list[tuple[float, float]]]: Modified graph with lists of connected nodes as values.
                                                                Each node is connected only to valid neighbors.
        """

        def calculate_dis(pos_1, pos_2):
            """Calculates Euclidean distance between two points."""
            return math.sqrt((pos_1[0] - pos_2[0]) ** 2 + (pos_1[1] - pos_2[1]) ** 2)

        def get_nodes(node_pos):
            """
            Finds valid neighbors for a given node.

            Args:
                node_pos (tuple[float, float]): Position of the current node.

            Returns:
                list[tuple[float, float]]: List of neighboring nodes that are within `connection_distance`
                                        and do not have obstacles blocking their connection.
            """
            return [
                target_pos
                for target_pos in graph.keys()
                if (node_pos != target_pos)
                and (calculate_dis(node_pos, target_pos) <= connection_distance)
                and (
                    not self._map.crosses([target_pos, node_pos])
                )  # Check for obstacle-free connection
            ]

        # Update graph with connected neighbors for each node
        graph = {node_pos: get_nodes(node_pos) for node_pos in graph.keys()}

        return graph

    def _create_graph(
        self,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """
        Creates a roadmap as a graph by generating nodes and connecting them based on proximity.

        This method generates nodes either randomly or in a grid layout depending on `use_grid`.
        It then connects nodes that are within a specified distance and have obstacle-free paths.

        Args:
            use_grid (bool): If False, generates nodes randomly; if True, generates nodes in a grid layout.
            node_count (int): Number of random nodes to generate. Used only if `use_grid` is False.
            grid_size (float): Distance between consecutive nodes in x and y directions. Used only if `use_grid` is True.
            connection_distance (float): Maximum distance to consider adding an edge between two nodes [m].

        Returns:
            dict[tuple[float, float], list[tuple[float, float]]]: A dictionary representing the roadmap graph.
                                                                Keys are node positions (x, y), and values are lists of connected nodes.
                                                                Node positions are rounded to fixed decimal places for consistency.
        """

        # Generate nodes based on sampling method
        graph = self._generate_nodes(use_grid, node_count, grid_size)

        # Connect generated nodes based on proximity and obstacle-free paths
        graph = self._connect_nodes(graph, connection_distance)

        return graph

    def _generate_nodes(
        self, use_grid: bool = False, node_count: int = 50, grid_size=0.1
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """
        Generates a set of valid nodes for building a roadmap.

        This method creates nodes either randomly or in a grid layout based on the `use_grid` parameter.
        Nodes are only added if they fall within the valid bounds of the map.

        Args:
            use_grid (bool): If False, generates nodes randomly; if True, generates nodes in a grid layout.
            node_count (int): Number of random nodes to generate (used only if `use_grid` is False).
            grid_size (float): Distance between consecutive nodes in x and y directions (used only if `use_grid` is True).

        Returns:
            dict[tuple[float, float], list[tuple[float, float]]]: A dictionary where keys are valid node positions
                                                                (x, y) [m] and values are empty lists to store edges.
                                                                Node positions are rounded to three decimal places.
        """
        graph: dict[tuple[float, float], list[tuple[float, float]]] = {}

        # Get map boundaries
        x_min, y_min, x_max, y_max = self._map.bounds()

        if use_grid:
            # Generate nodes in a grid layout
            x = x_min
            while x <= x_max:
                y = y_min
                while y <= y_max:
                    rounded_x = round(x, 3)
                    rounded_y = round(y, 3)
                    # Add node only if it is within the map bounds
                    if self._map.contains((rounded_x, rounded_y)):
                        graph[(rounded_x, rounded_y)] = []
                    y += grid_size
                x += grid_size
        else:
            # Generate random nodes
            for i in range(node_count):
                valid = False
                while not valid:
                    # Generate random (x, y) within bounds and round to three decimal places
                    x = round(random.uniform(x_min, x_max), 3)
                    y = round(random.uniform(y_min, y_max), 3)
                    valid = self._map.contains((x, y))  # Check if the node is within map bounds
                graph[(x, y)] = []

        return graph

    def _reconstruct_path(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        ancestors: dict[tuple[int, int], tuple[int, int]],
    ) -> list[tuple[float, float]]:
        """
        Reconstructs the path from the start to the goal using ancestor information.

        This method traces back from the goal to the start using the `ancestors` dictionary created
        during pathfinding. The reconstructed path includes all intermediate nodes.

        Args:
            start (tuple[float, float]): Initial location in (x, y) format.
            goal (tuple[float, float]): Goal location in (x, y) format.
            ancestors (dict[tuple[int, int], tuple[int, int]]): Dictionary mapping each node to its ancestor
                                                                from which it was reached during pathfinding.

        Returns:
            list[tuple[float, float]]: Reconstructed path from start to goal as a list of (x, y) coordinates.
                                    The first element is the start location.
        """
        path: list[tuple[float, float]] = [
            goal
        ]  # Start with the goal as the first point in the path
        node = goal

        # Trace back from goal to start using ancestor information
        while node != start:
            next_node = ancestors[node]  # Get the ancestor of the current node
            path.insert(0, next_node)  # Add the ancestor to the beginning of the path
            node = next_node

        return path


if __name__ == "__main__":
    # Define map name and path to map file
    map_name = "project"
    map_path = os.path.realpath(
        os.path.join(os.path.dirname(__file__), "..", "maps", map_name + ".json")
    )

    # Create the roadmap using PRM
    start_time = time.perf_counter()
    prm = PRM(map_path, use_grid=True, node_count=250, grid_size=0.1, connection_distance=0.15)
    roadmap_creation_time = time.perf_counter() - start_time

    print(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

    # Find a path from start to goal using A* search on the roadmap
    start_time = time.perf_counter()
    path = prm.find_path(start=(-1.0, -1.0), goal=(1.0, 1.0))
    pathfinding_time = time.perf_counter() - start_time

    print(f"Pathfinding time: {pathfinding_time:1.3f} s")

    # Smooth the found path for better navigation
    start_time = time.perf_counter()
    smoothed_path = prm.smooth_path(
        path,
        data_weight=0.1,
        smooth_weight=0.3,
        additional_smoothing_points=3,
    )
    smoothing_time = time.perf_counter() - start_time

    print(f"Smoothing time: {smoothing_time:1.3f} s")

    # Visualize and save the roadmap and paths
    prm.show(path=path, smoothed_path=smoothed_path, save_figure=True)
