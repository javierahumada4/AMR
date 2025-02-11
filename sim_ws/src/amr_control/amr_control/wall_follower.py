class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt
        self.state = "AVANZAR"

        self.Kp = 1.8
        self.Kd = 1.0
        self.prev_error = 0.0

        self.stop_distance = 0.25
        self.follow_distance = 0.4

    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).
        
        # Selección de sectores del LiDAR
        front = z_scan[:20] + z_scan[-20:]
        left = z_scan[20:100]
        right = z_scan[140:220]

        # # Ignorar valores nan o negativos
        valid_front = [d for d in front if d > 0]  
        valid_left = [d for d in left if d > 0]
        valid_right = [d for d in right if d > 0]

        # Cálculo de distancias mínimas con detección de pared muy cercana
        d_front = min(valid_front) if valid_front else 0
        d_left = min(valid_left) if valid_left else 0
        d_right = min(valid_right) if valid_right else 0

        # Estado: AVANZAR
        if self.state == "AVANZAR":
            error = self.follow_distance - d_right

            # Control PD
            P = self.Kp * error
            D = self.Kd * (error - self.prev_error) / self._dt
            w = (P + D) * 0.8 + z_w * 0.2  # Suavizamos con la velocidad angular actual
            self.prev_error = error

            v = min(0.2, z_v + 0.02)  # Aceleramos progresivamente

            # Si la pared desaparece, vuelve a AVANZAR
            if d_front < self.stop_distance:
                self.state = "GIRO"
                v = 0
                w = 0.5 if d_left > d_right else -0.5

        # Estado: GIRO
        elif self.state == "GIRO":
            if d_front > self.stop_distance:
                self.state = "AVANZAR"
                v = 0.2
                w = 0
            else:
                v = 0
                w = 0.5 if d_left > d_right else -0.5 # Giramos en la dirección con más espacio

        return v, w
