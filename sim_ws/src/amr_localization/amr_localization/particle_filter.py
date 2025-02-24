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
    """Particle filter implementation."""

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
        """Particle filter class initializer.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].
            sensor_range_max: Maximum sensor measurement range [m].
            sensor_range_min: Minimum sensor measurement range [m].
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

        """
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._particle_count: int = particle_count
        self._sensor_range_max: float = sensor_range_max
        self._sensor_range_min: float = sensor_range_min
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._iteration: int = 0

        self._map = Map(
            map_path,
            sensor_range_max,
            compiled_intersect=True,
            use_regions=False,
            safety_distance=0.08,
        )
        self._particles = self._init_particles(
            particle_count, global_localization, initial_pose, initial_pose_sigma
        )
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(pytz.timezone("Europe/Madrid")).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        # TODO: 3.10. Complete the missing function body with your code.
        localized: bool = False
        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))

        # Escobar
        # clusters = DBSCAN(eps=0.5, min_samples=10).fit(self._particles[:, :2]).labels_
        # unique_clusters = np.unique(clusters[clusters != -1])
        # if len(unique_clusters) == 1:
        #     self._particle_count = 50
        #     localized = True
        #     x_mean, y_mean = np.mean(self._particles[:, :2], axis=0)
        #     theta_mean = np.arctan2(
        #         np.mean(np.sin(self._particles[:, 2])),
        #         np.mean(np.cos(self._particles[:, 2])),
        #     )     
        #     pose = (x_mean, y_mean, theta_mean)
        
        return localized, pose

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].

        """
        self._iteration += 1

        # TODO: 3.5. Complete the function body with your code.
            # x, y, theta = particle
            # x_old, y_old, theta_old = x, y, theta

            # theta += w * self._dt
            # theta %= 2*math.pi
            
            # sen_old = math.sin(theta_old)
            # cos_old = math.cos(theta_old)
            # sen_new = math.sin(theta)
            # cos_new = math.cos(theta)

            # x += (-v/w)*sen_old + (v/w)*sen_new
            # y += (v/w)*cos_old - (v/w)*cos_new

            # x += random.gauss(0, self._sigma_v)
            # y += random.gauss(0, self._sigma_v)
            # theta += random.gauss(0, self._sigma_w)
            
            # self._particles[i] = (x, y, theta)
        for i, particle in enumerate(self._particles):
            x, y, theta = particle
            x_old, y_old, theta_old = x, y, theta
            noise_v = random.gauss(v, self._sigma_v)
            noise_w = random.gauss(w, self._sigma_w)

            x += noise_v * math.cos(theta) * self._dt
            y += noise_v * math.sin(theta) * self._dt
            
            theta += noise_w * self._dt
            theta %= 2*math.pi
            
            self._particles[i] = (x, y, theta)

            intersections, _ = self._map.check_collision([(x_old, y_old), (x, y)])
            if intersections:
                self._particles[i] = (intersections[0], intersections[1], theta)
        
    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles.

        Args:
            measurements: Sensor measurements [m].

        """
        # TODO: 3.9. Complete the function body with your code (i.e., replace the pass statement).
        # Calculate measurement probabilities for each particle
        raw_probabilities = np.array([self._measurement_probability(measurements, particle) for particle in self._particles])
        log_probabilities = np.log(raw_probabilities)
        max_log_prob = np.max(log_probabilities)
        log_probabilities -= max_log_prob # Avoid underflow by subtracting the maximum log probability
        probabilities = np.exp(log_probabilities)

        # Normalize the probabilities to sum to 1
        probabilities /= np.sum(probabilities)

        print(f"Raw measurement probs: {raw_probabilities[:10]}")
        print(f"Log probs: {log_probabilities[:10]}")
        print(f"Exp log probs: {np.exp(log_probabilities[:10])}")
        print(f"Final normalized probs: {probabilities[:10]}")

        # Compute the cumulative sum of the normalized probabilities
        cumulative_sum = np.cumsum(probabilities)
        cumulative_sum[-1] = 1.0  # Ensure the sum is exactly 1 avoiding floating-point errors

        # Generate N uniform random values between 0 and 1/N
        N = self._particle_count
        start = np.random.uniform(0, 1 / N)
        positions = start + np.arange(N) / N

        indexes = np.searchsorted(cumulative_sum, positions)
        self._particles = self._particles[indexes]
        
        
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
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3*pi/2].

        Args:
            particle_count: Number of particles.
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

        Returns: A NumPy array of tuples (x, y, theta) [m, m, rad].

        """
        particles = np.empty((particle_count, 3), dtype=object)

        # TODO: 3.4. Complete the missing function body with your code.
        if global_localization:
            x_min, y_min, x_max, y_max = self._map.bounds()
            for i in range(particle_count):
                x : float
                y : float
                valid = False
                while not valid:
                    x = random.uniform(x_min, x_max)
                    y = random.uniform(y_min, y_max)
                    valid = self._map.contains((x, y))
                theta = random.choice([0, math.pi / 2, math.pi, 3 * math.pi / 2])
                particles[i] = (x, y, theta)
        else:
            x_mean, y_mean, theta_mean = initial_pose
            x_sigma, y_sigma, theta_sigma = initial_pose_sigma
            for i in range(particle_count):
                x : float
                y : float
                valid = False
                while not valid:
                    x = random.gauss(x_mean, x_sigma)
                    y = random.gauss(y_mean, y_sigma)
                    valid = self._map.contains((x, y))
                theta = random.gauss(theta_mean, theta_sigma)
                particles[i] = (x, y, theta)

        return particles

    def _sense(self, particle: tuple[float, float, float]) -> list[float]:
        """Obtains the predicted measurement of every LiDAR ray given the robot's pose.

        Args:
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; nan if a sensor is out of range.

        """
        z_hat: list[float] = []

        # TODO: 3.6. Complete the missing function body with your code.
        rays = self._lidar_rays(particle, tuple(range(0,240,15)))
        for ray in rays:
            _ , distance = self._map.check_collision(ray, True)
            if distance <= self._sensor_range_max: # 1 before
                z_hat.append(distance)
            else:
                z_hat.append(float("nan"))

        return z_hat

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian value.

        """
        # TODO: 3.7. Complete the function body (i.e., replace the code below).
        return math.exp(-((mu - x)**2)/(2*sigma**2))/((2*math.pi*sigma**2)**0.5)
        
    def _lidar_rays(
        self, pose: tuple[float, float, float], indices: tuple[float], degree_increment: float = 1.5
    ) -> list[list[tuple[float, float]]]:
        """Determines the simulated LiDAR ray segments for a given robot pose.

        Args:
            pose: Robot pose (x, y, theta) in [m] and [rad].
            indices: Rays of interest in counterclockwise order (0 for to the forward-facing ray).
            degree_increment: Angle difference of the sensor between contiguous rays [degrees].

        Returns: Ray segments. Format:
                 [[(x0_start, y0_start), (x0_end, y0_end)],
                  [(x1_start, y1_start), (x1_end, y1_end)],
                  ...]

        """
        x, y, theta = pose

        # Convert the sensor origin to world coordinates
        x_start = x - 0.035 * math.cos(theta)
        y_start = y - 0.035 * math.sin(theta)

        rays = []

        for index in indices:
            ray_angle = math.radians(degree_increment * index)
            x_end = x_start + self._sensor_range_max * math.cos(theta + ray_angle)
            y_end = y_start + self._sensor_range_max * math.sin(theta + ray_angle)
            rays.append([(x_start, y_start), (x_end, y_end)])

        return rays

    def _measurement_probability(
        self, measurements: list[float], particle: tuple[float, float, float]
    ) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with
        the minimum sensor range to perform the computation because the environment is smaller
        than the maximum range.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns:
            float: Probability.

        """
        probability = 1.0

        # TODO: 3.8. Complete the missing function body with your code.
        measurements = [measure if not math.isnan(measure) else self._sensor_range_min - 0.01 for measure in measurements]

        sim_measurements = self._sense(particle)
        sim_measurements = [measure if not math.isnan(measure) else self._sensor_range_min - 0.01 for measure in sim_measurements]

        for z, z_hat in zip(measurements, sim_measurements):
            probability *= self._gaussian(z, self._sigma_z, z_hat)
    
        return probability
