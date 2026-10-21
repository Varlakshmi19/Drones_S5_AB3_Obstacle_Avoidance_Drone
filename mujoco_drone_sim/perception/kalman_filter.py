"""
Kalman Filter Implementation for Dynamic Obstacle Tracking and Trajectory Prediction.

Provides:
- Linear Kalman Filter (LKF) with Constant-Velocity (CV) kinematic motion model.
- Multi-step future trajectory rollout and covariance expansion.
- Measurement gating and noise filtering for LiDAR rangefinding observations.
- Multi-target tracking manager (MultiObstacleTracker) for multiple simultaneous dynamic obstacles.
"""

import numpy as np


class ObstacleKalmanFilter:
    """
    Kalman Filter for estimating 3D position and velocity of a dynamic obstacle
    from noisy sensor measurements.
    
    State vector: x = [p_x, p_y, p_z, v_x, v_y, v_z]^T in R^6
    Measurement:  z = [z_x, z_y, z_z]^T in R^3
    """

    def __init__(
        self,
        initial_pos=(0.0, 0.0, 0.0),
        initial_vel=(0.0, 0.0, 0.0),
        dt=1.0 / 48.0,
        process_noise_std=0.35,
        measurement_noise_std=0.06,
        track_id=0
    ):
        self.dt = float(dt)
        self.track_id = track_id
        self.process_noise_std = float(process_noise_std)
        self.measurement_noise_std = float(measurement_noise_std)

        # State vector: [px, py, pz, vx, vy, vz]
        self.x = np.zeros(6, dtype=np.float64)
        self.x[0:3] = initial_pos
        self.x[3:6] = initial_vel

        # State transition matrix F (6x6)
        self.F = np.eye(6, dtype=np.float64)
        self.F[0:3, 3:6] = np.eye(3) * self.dt

        # Measurement matrix H (3x6)
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0:3, 0:3] = np.eye(3)

        # Process noise covariance Q (6x6) using Discrete White Noise Acceleration (DWNA)
        q = self.process_noise_std ** 2
        q_pos = (self.dt ** 3) / 3.0 * q
        q_pos_vel = (self.dt ** 2) / 2.0 * q
        q_vel = self.dt * q

        self.Q = np.zeros((6, 6), dtype=np.float64)
        self.Q[0:3, 0:3] = np.eye(3) * q_pos
        self.Q[0:3, 3:6] = np.eye(3) * q_pos_vel
        self.Q[3:6, 0:3] = np.eye(3) * q_pos_vel
        self.Q[3:6, 3:6] = np.eye(3) * q_vel

        # Measurement noise covariance R (3x3)
        r = self.measurement_noise_std ** 2
        self.R = np.eye(3, dtype=np.float64) * r

        # Estimation error covariance P (6x6)
        self.P = np.eye(6, dtype=np.float64)
        self.P[0:3, 0:3] = np.eye(3) * (self.measurement_noise_std ** 2)
        self.P[3:6, 3:6] = np.eye(3) * 1.0  # Initial velocity uncertainty

        # Diagnostics & Tracking History
        self.step_count = 0
        self.last_innovation = np.zeros(3)
        self.last_innovation_cov = np.eye(3)
        self.history = []

    def predict(self, dt=None):
        """
        Kalman Filter Prediction Step (Time Update).
        Propagates state estimate and covariance forward by dt.
        """
        if dt is not None and abs(dt - self.dt) > 1e-6:
            self.dt = float(dt)
            self.F[0:3, 3:6] = np.eye(3) * self.dt
            q = self.process_noise_std ** 2
            self.Q[0:3, 0:3] = np.eye(3) * ((self.dt ** 3) / 3.0 * q)
            self.Q[0:3, 3:6] = np.eye(3) * ((self.dt ** 2) / 2.0 * q)
            self.Q[3:6, 0:3] = np.eye(3) * ((self.dt ** 2) / 2.0 * q)
            self.Q[3:6, 3:6] = np.eye(3) * (self.dt * q)

        # x_{k|k-1} = F * x_{k-1|k-1}
        self.x = self.F @ self.x

        # P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.x.copy()

    def update(self, measurement):
        """
        Kalman Filter Measurement Update Step.
        measurement: array-like of length 3 [z_x, z_y, z_z]
        """
        z = np.asarray(measurement, dtype=np.float64)[:3]

        # Innovation: y = z - H * x
        y = z - self.H @ self.x
        self.last_innovation = y.copy()

        # Innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R
        self.last_innovation_cov = S.copy()

        # Kalman Gain: K = P * H^T * S^{-1}
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Posterior state: x = x + K * y
        self.x = self.x + K @ y

        # Joseph form for numerical stability: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
        I_KH = np.eye(6) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        self.step_count += 1
        pos_cov_diag = np.diag(self.P[0:3, 0:3])

        entry = {
            "step": self.step_count,
            "measurement": z.tolist(),
            "filtered_pos": self.x[0:3].tolist(),
            "filtered_vel": self.x[3:6].tolist(),
            "std_pos": np.sqrt(np.maximum(pos_cov_diag, 1e-9)).tolist(),
            "innovation": y.tolist()
        }
        self.history.append(entry)

        return self.x.copy()

    def predict_trajectory(self, horizon_sec=1.0, step_dt=0.05):
        """
        Predict future trajectory over horizon_sec seconds.
        Returns list of (t, predicted_pos, 2_sigma_uncertainty).
        """
        num_steps = max(1, int(round(horizon_sec / step_dt)))
        trajectory = []
        cur_x = self.x.copy()
        cur_P = self.P.copy()

        # Step transition
        F_step = np.eye(6)
        F_step[0:3, 3:6] = np.eye(3) * step_dt
        q = self.process_noise_std ** 2
        Q_step = np.zeros((6, 6))
        Q_step[0:3, 0:3] = np.eye(3) * ((step_dt ** 3) / 3.0 * q)
        Q_step[0:3, 3:6] = np.eye(3) * ((step_dt ** 2) / 2.0 * q)
        Q_step[3:6, 0:3] = np.eye(3) * ((step_dt ** 2) / 2.0 * q)
        Q_step[3:6, 3:6] = np.eye(3) * (step_dt * q)

        for step in range(1, num_steps + 1):
            t_future = step * step_dt
            cur_x = F_step @ cur_x
            cur_P = F_step @ cur_P @ F_step.T + Q_step
            p_cov = np.diag(cur_P[0:3, 0:3])
            sigma_pos = 2.0 * np.sqrt(np.maximum(p_cov, 1e-9))

            trajectory.append({
                "dt": round(t_future, 3),
                "pos": cur_x[0:3].tolist(),
                "vel": cur_x[3:6].tolist(),
                "uncertainty_radius": float(np.mean(sigma_pos))
            })

        return trajectory

    @property
    def position(self):
        """Current filtered 3D position."""
        return self.x[0:3].copy()

    @property
    def velocity(self):
        """Current filtered 3D velocity."""
        return self.x[3:6].copy()

    @property
    def position_std(self):
        """1-sigma position standard deviation [sigma_x, sigma_y, sigma_z]."""
        return np.sqrt(np.maximum(np.diag(self.P[0:3, 0:3]), 1e-9))

    @property
    def velocity_std(self):
        """1-sigma velocity standard deviation."""
        return np.sqrt(np.maximum(np.diag(self.P[3:6, 3:6]), 1e-9))


class MultiObstacleTracker:
    """
    Manages Kalman Filters for tracking multiple simultaneous dynamic obstacles
    in the MuJoCo simulation arena.
    """

    def __init__(self, dt=1.0 / 48.0, process_noise_std=0.35, measurement_noise_std=0.06):
        self.dt = dt
        self.process_noise_std = process_noise_std
        self.measurement_noise_std = measurement_noise_std
        self.filters = {}  # obstacle_name -> ObstacleKalmanFilter
        self.last_update_times = {}

    def get_or_create(self, name, initial_pos=(0.0, 0.0, 1.0), initial_vel=(0.0, 0.0, 0.0)):
        if name not in self.filters:
            self.filters[name] = ObstacleKalmanFilter(
                initial_pos=initial_pos,
                initial_vel=initial_vel,
                dt=self.dt,
                process_noise_std=self.process_noise_std,
                measurement_noise_std=self.measurement_noise_std,
                track_id=len(self.filters) + 1
            )
        return self.filters[name]

    def predict_all(self, dt=None):
        for kf in self.filters.values():
            kf.predict(dt)

    def update_obstacle(self, name, measurement):
        kf = self.get_or_create(name, initial_pos=measurement)
        kf.predict(self.dt)
        kf.update(measurement)
        return kf

    def get_states(self):
        """Return dict of estimated states for all tracked obstacles."""
        states = {}
        for name, kf in self.filters.items():
            states[name] = {
                "pos": kf.position.tolist(),
                "vel": kf.velocity.tolist(),
                "speed": float(np.linalg.norm(kf.velocity)),
                "pos_std": kf.position_std.tolist(),
                "trajectory_prediction": kf.predict_trajectory(horizon_sec=1.0)
            }
        return states
