import math
class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt
        self.state = "AVANZAR"

        self.Kp = 150.0
        self.Kd = 1.0
        self.prev_error = 0.0

        self.stop_distance = 0.2
        self.follow_distance = 0.2
        
        self.v = 2.0
        self.w_control = 0.2
        self.w_giro = 3.0
        
        self.w_actual = 0.0

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
        front = list(z_scan[0:20]) + list(z_scan[-20:])
        left = list(z_scan[40:60])
        right = list(z_scan[-60:-40])

        # # Ignorar valores nan o negativos
        valid_front = [d for d in front if d > 0.0]  
        valid_left = [d for d in left if d > 0.0]
        valid_right = [d for d in right if d > 0.0]

        # Cálculo de distancias mínimas con detección de pared muy cercana
        d_front = min(valid_front) if valid_front else 0.0
        d_left = min(valid_left) if valid_left else 0.0
        d_right = min(valid_right) if valid_right else 0.0


        # Estado: AVANZAR
        if self.state == "AVANZAR":
            error = self.follow_distance - d_right

            # Control PD
            P = self.Kp * error
            D = self.Kd * (error - self.prev_error) / self._dt
            w = (P + D) * 0.8 + z_w * 0.2 # Suavizamos con la velocidad angular actual
            w = min(w, self.w_control) if w > 0 else max(w, -self.w_control)
                
            self.prev_error = error

            v = min(self.v, z_v + 0.02)  # Aceleramos progresivamente

            # Si la pared desaparece, vuelve a AVANZAR
            if d_front < self.stop_distance:
                self.angle = 0.0
                self.state = "GIRO"
                v = 0.2
                w = self.w_giro if d_left > d_right else -self.w_giro
                self.w_actual = w

        # Estado: GIRO
        elif self.state == "GIRO":
            if d_front > self.stop_distance + 0.15:
                self.state = "AVANZAR"
                v = self.v
                w = -self.w_actual*0.5
            else:
                v = 0.12
                w = self.w_actual
            

        return v, w
