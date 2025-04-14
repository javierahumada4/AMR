import datetime
import math
import numpy as np
import os
import pytz
import random

from amr_localization.maps import Map
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN


class ParticleFilter:
    """
    A particle filter implementation for robot localization.

    The particle filter uses a set of particles to represent the robot's belief about its pose
    and updates these particles based on motion commands and sensor measurements.
    """

    def __init__(
        self,
        dt: float,
        map_path: str,
        particle_count: int,
        sigma_v: float = 0.05,
        sigma_w: float = 0.1,
        sigma_z: float = 0.2,
        sensor_range_max: float = 8.0,
        sensor_range_min: float = 0.16,
        global_localization: bool = True,
        initial_pose: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
        initial_pose_sigma: tuple[float, float, float] = (float("nan"), float("nan"), float("nan")),
    ):
        """
        Initializes the particle filter with the given parameters.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].
            sensor_range_max: Maximum sensor measurement range [m].
            sensor_range_min: Minimum sensor measurement range [m].
            global_localization: If True, initializes particles globally; otherwise, tracks a given pose.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].
        """
        # Initialize parameters and variables related to particle filter operation
        self._dt = dt  # Sampling period
        self._initial_particle_count = particle_count  # Initial number of particles
        self._particle_count = particle_count  # Current number of particles
        self._sensor_range_max = sensor_range_max  # Maximum range for sensors
        self._sensor_range_min = sensor_range_min  # Minimum range for sensors
        self._sigma_v = sigma_v  # Linear velocity noise standard deviation
        self._sigma_w = sigma_w  # Angular velocity noise standard deviation
        self._sigma_z = sigma_z  # Measurement noise standard deviation
        self._iteration = 0  # Tracks the number of iterations

        # Load the map and initialize particles based on localization mode
        self._map = Map(
            map_path,
            sensor_range_max,
            compiled_intersect=True,
            use_regions=False,
            safety_distance=0.08,
        )

        # Initialize particles either globally or around a specific pose
        self._particles = self._init_particles(
            particle_count, global_localization, initial_pose, initial_pose_sigma
        )

        # Setup visualization components for debugging or monitoring purposes
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))

        # Timestamp for saving figures or logs with unique identifiers
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """
        Computes the pose estimate when the particles form a single DBSCAN cluster.

        This function uses DBSCAN clustering to group particles based on their proximity.
        If multiple clusters are detected, the particle count is dynamically increased to improve
        localization accuracy.
        If only one cluster is detected, the system assumes successful localization and reduces the
        particle count for efficient tracking.
        The final pose estimate is computed as the mean position and orientation of particles in the
        main cluster.

        Returns:
            localized (bool): True if a valid pose estimate is obtained (single cluster detected).
            pose (tuple): Estimated robot pose (x, y, theta) in meters and radians.
                        Defaults to (inf, inf, inf) if localization fails.
        """
        # Check if there are any particles; return invalid pose if none exist
        if self._particles.shape[0] == 0:
            return False, (float("inf"), float("inf"), float("inf"))

        localized = False  # Flag indicating whether localization succeeded
        pose = (float("inf"), float("inf"), float("inf"))  # Default invalid pose

        # Create a 4D representation of particles for clustering: (x, y, sin(theta), cos(theta))
        particles_4d = np.column_stack(
            (
                self._particles[:, 0],  # x-coordinate
                self._particles[:, 1],  # y-coordinate
                np.sin(self._particles[:, 2]),  # sin(theta)
                np.cos(self._particles[:, 2]),  # cos(theta)
            )
        )

        # Apply DBSCAN clustering to group particles based on proximity
        clustering = DBSCAN(eps=0.15, min_samples=20).fit(particles_4d)
        clusters = clustering.labels_  # Cluster labels for each particle

        # Extract unique cluster labels and their counts (excluding noise labeled as -1)
        unique_clusters, counts = np.unique(clusters[clusters != -1], return_counts=True)

        if len(unique_clusters) > 1:
            # If multiple clusters are detected, increase particle count dynamically for better accuracy
            self._particle_count = max(200, len(unique_clusters) * 200)

        elif len(unique_clusters) == 1:
            localized = True  # Localization succeeded with a single cluster

            # Reduce particle count for efficient tracking when only one cluster is detected
            self._particle_count = 50

            # Identify the main cluster (the one with the most particles)
            main_cluster = unique_clusters[np.argmax(counts)]
            cluster_mask = clusters == main_cluster

            # Compute the mean position (x, y) of particles in the main cluster
            x_mean, y_mean = np.mean(particles_4d[cluster_mask, :2], axis=0)

            # Compute the mean orientation (theta) using sin and cos components
            sin_mean = np.mean(particles_4d[cluster_mask, 2])  # Mean sin(theta)
            cos_mean = np.mean(particles_4d[cluster_mask, 3])  # Mean cos(theta)
            theta_mean = math.atan2(sin_mean, cos_mean)  # atan2 handles periodicity

            pose = (x_mean, y_mean, theta_mean)  # Final estimated pose

        return localized, pose

    def move(self, v: float, w: float) -> None:
        """
        Updates the particles based on the robot's motion model.

        This function applies a motion update to all particles by simulating the robot's movement
        with added noise for linear and angular velocities. It also checks for collisions with the map
        and adjusts the particle positions accordingly.

        Args:
            v (float): Linear velocity of the robot [m/s].
            w (float): Angular velocity of the robot [rad/s].
        """
        self._iteration += 1  # Increment the iteration counter

        # Loop through each particle to apply the motion update
        for i, particle in enumerate(self._particles):
            x, y, theta = particle  # Extract current particle position and orientation
            x_old, y_old, _ = x, y, theta  # Store previous position for collision checks

            # Add Gaussian noise to linear and angular velocities
            noise_v = random.gauss(v, self._sigma_v)
            noise_w = random.gauss(w, self._sigma_w)

            # Update particle position based on noisy velocities and time step
            x += noise_v * math.cos(theta) * self._dt
            y += noise_v * math.sin(theta) * self._dt

            # Update particle orientation and normalize it to [-pi, pi]
            theta += noise_w * self._dt
            theta = (theta + math.pi) % (2 * math.pi) - math.pi

            # Save updated particle state
            self._particles[i] = (x, y, theta)

            # Check for collisions with the map along the particle's trajectory
            intersection_result = self._map.check_collision([(x_old, y_old), (x, y)])

            if intersection_result[0]:  # If a collision occurred
                x_collision, y_collision = intersection_result[0]  # Extract collision coordinates
                self._particles[i] = (
                    x_collision,
                    y_collision,
                    theta,
                )  # Adjust particle position to collision point

    def resample(self, measurements: list[float]) -> None:
        """
        Resamples particles based on their likelihood given sensor measurements.

        This function uses importance sampling to generate a new set of particles.
        Each particle's weight is determined by its measurement likelihood compared to actual sensor data.
        Resampling ensures that particles with higher weights are more likely to be retained.

        Args:
            measurements (list[float]): Sensor measurements [m].
        """
        # Compute raw probabilities for each particle based on sensor measurements
        raw_probabilities = np.array(
            [self._measurement_probability(measurements, particle) for particle in self._particles]
        )

        # Convert probabilities to log scale to avoid numerical underflow issues
        log_probabilities = np.log(raw_probabilities + 1e-300)

        # Normalize log probabilities by subtracting the maximum log probability
        max_log_prob = np.max(log_probabilities)
        log_probabilities -= max_log_prob

        # Convert log probabilities back to normal scale
        probabilities = np.exp(log_probabilities)

        # Normalize probabilities so they sum to 1
        total_prob = np.sum(probabilities)

        if total_prob == 0:  # Handle edge case where all probabilities are zero
            probabilities = np.ones_like(probabilities) / len(probabilities)
        else:
            probabilities /= total_prob

        # Compute cumulative sum of normalized probabilities for sampling
        cumulative_sum = np.cumsum(probabilities)

        # Ensure cumulative sum ends at exactly 1.0 to avoid floating-point errors during sampling
        cumulative_sum[-1] = 1.0

        # Generate N uniformly spaced random values between 0 and 1/N for systematic resampling
        N = self._particle_count
        start = np.random.uniform(0, 1 / N)
        positions = start + np.arange(N) / N

        # Find indices of particles corresponding to generated random values using cumulative sum
        indexes = np.searchsorted(cumulative_sum, positions)

        # Clip indices to ensure they remain within valid range of particles array indices
        indexes = np.clip(indexes, 0, len(self._particles) - 1)

        # Resample particles based on computed indices and add small random perturbations for diversity
        self._particles = self._particles[indexes].copy()

        self._particles += np.random.normal(
            0, 0.02, self._particles.shape
        )  # Add Gaussian noise for robustness

    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(
                self._particles[:, 0],
                self._particles[:, 1],
                dx,
                dy,
                color="b",
                scale=15,
                scale_units="inches",
            )
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], "bo", markersize=1)

        return axes

    def show(
        self,
        title: str = "",
        orientation: bool = True,
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)

        axes.set_title(title + " (Iteration #" + str(self._iteration) + ")")
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", save_dir, self._timestamp)
            )

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = str(self._iteration).zfill(4) + " " + title.lower() + ".png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _init_particles(
        self,
        particle_count: int,
        global_localization: bool,
        initial_pose: tuple[float, float, float],
        initial_pose_sigma: tuple[float, float, float],
    ) -> np.ndarray:
        """
        Initializes a set of particles either globally or around a specific pose.

        For global localization, particles are distributed randomly across the map within valid bounds.
        For pose tracking, particles are distributed around the given initial pose with Gaussian noise.

        Args:
            particle_count (int): Number of particles to initialize.
            global_localization (bool): If True, particles are distributed globally; otherwise,
            around a specific pose.
            initial_pose (tuple[float, float, float]): Approximate initial robot pose
            (x, y, theta) [m, m, rad].
            initial_pose_sigma (tuple[float, float, float]): Standard deviation for the initial pose
            guess [m, m, rad].

        Returns:
            np.ndarray: A NumPy array of shape (particle_count, 3), where each row represents a
            particle (x, y, theta).
        """
        # Create an empty array to store the particles
        particles = np.empty((particle_count, 3), dtype=np.float64)

        if global_localization:
            # Global localization: distribute particles randomly across the map
            x_min, y_min, x_max, y_max = self._map.bounds()  # Get map boundaries
            for i in range(particle_count):
                valid = False
                while not valid:
                    # Generate random x and y within map bounds
                    x = random.uniform(x_min, x_max)
                    y = random.uniform(y_min, y_max)
                    valid = self._map.contains(
                        (x, y)
                    )  # Check if the position is valid within the map
                # Randomly assign one of four discrete orientations
                theta = random.choice([0, math.pi / 2, math.pi, 3 * math.pi / 2])
                particles[i] = (x, y, theta)  # Store the particle
        else:
            # Pose tracking: distribute particles around the given initial pose with Gaussian noise
            x_mean, y_mean, theta_mean = initial_pose  # Mean values for x, y, and theta
            x_sigma, y_sigma, theta_sigma = (
                initial_pose_sigma  # Standard deviations for x, y, and theta
            )
            for i in range(particle_count):
                valid = False
                while not valid:
                    # Generate random x and y using Gaussian distribution around the mean values
                    x = random.gauss(x_mean, x_sigma)
                    y = random.gauss(y_mean, y_sigma)
                    valid = self._map.contains(
                        (x, y)
                    )  # Check if the position is valid within the map
                # Generate random theta using Gaussian distribution around the mean value
                theta = random.gauss(theta_mean, theta_sigma)
                particles[i] = (x, y, theta)  # Store the particle

        return particles  # Return the initialized particles

    def _sense(self, particle: tuple[float, float, float]) -> list[float]:
        """
        Simulates LiDAR sensor measurements for a given particle's pose.

        This function predicts measurements by simulating LiDAR rays from the particle's position.
        If a ray does not intersect any obstacle within the sensor range limit,
        its measurement is set to NaN.

        Args:
            particle (tuple[float, float, float]): Particle's pose
            (x-coordinate [m], y-coordinate [m], orientation [rad]).

        Returns:
            list[float]: Predicted measurements for each LiDAR ray. NaN indicates no intersection
            within range.
        """
        z_hat: list[float] = []  # List to store predicted measurements

        # Generate LiDAR rays based on the particle's pose and predefined angles
        rays = self._lidar_rays(
            particle, tuple(range(0, 240, 15))
        )  # Rays at intervals of 15 degrees

        for ray in rays:
            _, distance = self._map.check_collision(
                ray, True
            )  # Check for intersections with obstacles
            if distance <= self._sensor_range_max:
                z_hat.append(distance)  # Valid measurement within sensor range
            else:
                z_hat.append(float("nan"))  # Out-of-range measurement

        return z_hat

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """
        Computes the probability density of a Gaussian distribution.

        This function calculates the likelihood of a value `x` given a Gaussian distribution
        with mean `mu` and standard deviation `sigma`.

        Args:
            mu (float): Mean of the Gaussian distribution.
            sigma (float): Standard deviation of the Gaussian distribution.
            x (float): Value for which to compute the probability density.

        Returns:
            float: Probability density at `x`.
        """
        return math.exp(-((mu - x) ** 2) / (2 * sigma**2)) / ((2 * math.pi * sigma**2) ** 0.5)

    def _lidar_rays(
        self, pose: tuple[float, float, float], indices: tuple[float], degree_increment: float = 1.5
    ) -> list[list[tuple[float, float]]]:
        """
        Generates simulated LiDAR ray segments for a given robot pose.

        This function calculates the start and end points of LiDAR rays based on the robot's position
        and orientation. The rays are defined in a counterclockwise order relative to the robot's
        forward-facing direction.

        Args:
            pose (tuple[float, float, float]): Robot pose (x, y, theta) in meters and radians.
            indices (tuple[float]): Indices of rays of interest in counterclockwise order
            (0 corresponds to the forward-facing ray).
            degree_increment (float): Angular increment between contiguous rays in degrees.

        Returns:
            list[list[tuple[float, float]]]: A list of ray segments. Each ray segment is represented
            as:
                [[(x_start, y_start), (x_end, y_end)], ...]
                where (x_start, y_start) is the starting point and (x_end, y_end) is the endpoint
                of the ray.
        """
        x, y, theta = pose  # Extract robot's position and orientation

        # Calculate the starting point of the rays in world coordinates
        x_start = x - 0.035 * math.cos(theta)  # Offset from the robot's position
        y_start = y - 0.035 * math.sin(theta)

        rays = []  # List to store ray segments

        for index in indices:
            # Compute the angle of each ray relative to the robot's orientation
            ray_angle = math.radians(degree_increment * index)

            # Calculate the endpoint of each ray based on maximum sensor range
            x_end = x_start + self._sensor_range_max * math.cos(theta + ray_angle)
            y_end = y_start + self._sensor_range_max * math.sin(theta + ray_angle)

            # Append the ray segment as a pair of start and end points
            rays.append([(x_start, y_start), (x_end, y_end)])

        return rays

    def _measurement_probability(
        self, measurements: list[float], particle: tuple[float, float, float]
    ) -> float:
        """
        Computes the likelihood of a set of sensor measurements given a particle's pose.

        This function compares actual sensor measurements with simulated measurements from a
        particle's pose.
        If a measurement is unavailable (e.g., out of range), it is replaced with a value slightly
        below the minimum sensor range for computation purposes.

        Args:
            measurements (list[float]): Actual sensor measurements [m].
            particle (tuple[float, float, float]): Particle pose
            (x-coordinate [m], y-coordinate [m], orientation [rad]).

        Returns:
            float: The computed probability that the given particle corresponds to the actual
            measurements.
        """
        probability = 1.0  # Initialize probability as 1

        # Replace NaN measurements with a value slightly below the minimum sensor range
        measurements = [
            measure if not math.isnan(measure) else self._sensor_range_min - 0.01
            for measure in measurements
        ]

        # Obtain simulated measurements from the particle's pose
        sim_measurements = self._sense(particle)

        # Replace NaN simulated measurements with a value slightly below the minimum sensor range
        sim_measurements = [
            measure if not math.isnan(measure) else self._sensor_range_min - 0.01
            for measure in sim_measurements
        ]

        # Compute probability by comparing actual and simulated measurements using Gaussian distribution
        for z, z_hat in zip(
            measurements[::15], sim_measurements
        ):  # Use every 15th measurement for comparison
            probability *= self._gaussian(
                z, self._sigma_z, z_hat
            )  # Multiply probabilities for all rays

        return probability
