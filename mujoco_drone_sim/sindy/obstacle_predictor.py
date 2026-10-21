"""
Online SINDy Dynamic Moving Obstacle Trajectory Predictor.
Observes positions of moving obstacles (e.g., Scenario 7 and Scenario 9) over time,
identifies their governing harmonic/polynomial equations, and predicts future states.
"""

from collections import deque
import numpy as np
from .sindy_core import SINDy, PolynomialLibrary, finite_difference


class SINDyObstaclePredictor:
    """
    Identifies harmonic differential equations for moving obstacles:
        d²y/dt² = -omega² * y
    and predicts future collision horizons t + delta_t.
    """

    def __init__(self, history_len=30, dt=1.0 / 48.0, poly_degree=2):
        self.history_len = history_len
        self.dt = dt
        self.poly_degree = poly_degree

        self.history_t = deque(maxlen=history_len)
        self.history_pos = deque(maxlen=history_len)
        self.model = SINDy(
            library=PolynomialLibrary(degree=poly_degree, include_bias=True),
            threshold=0.04,
            alpha=1e-5
        )

        self.is_fitted = False
        self.identified_omega = 1.0
        self.amplitude = 0.60
        self.last_forecast_error = 0.0

    def add_observation(self, t, pos):
        """Add timestamped obstacle position observation."""
        self.history_t.append(float(t))
        self.history_pos.append(np.asarray(pos, dtype=float).copy())

        # Fit SINDy once enough observation history is gathered
        if len(self.history_pos) >= 15:
            self._fit_motion_model()

    def _fit_motion_model(self):
        """Fit harmonic motion model to moving obstacle trajectory."""
        Y = np.array([p[1] for p in self.history_pos])
        t_arr = np.array(self.history_t)
        dt = np.mean(np.diff(t_arr)) if len(t_arr) > 1 else self.dt

        # Compute velocity and acceleration via finite differences
        v_y = finite_difference(Y.reshape(-1, 1), dt, smooth_window=5).flatten()
        a_y = finite_difference(v_y.reshape(-1, 1), dt, smooth_window=5).flatten()

        X = np.column_stack([Y, v_y])
        dX = np.column_stack([v_y, a_y])

        try:
            self.model.fit(X, dX, state_names=["y", "v_y"])
            self.is_fitted = True

            # Extract estimated harmonic frequency omega^2 from coefficient of 'y' in acceleration equation
            # d(v_y)/dt = -omega^2 * y
            if hasattr(self.model, "Xi") and self.model.Xi.shape[1] >= 2:
                # find index of 'y' in library
                if "y" in self.model.library.names_:
                    idx_y = self.model.library.names_.index("y")
                    coeff_y = self.model.Xi[idx_y, 1]  # coefficient in d(v_y)/dt
                    if coeff_y < -0.05:
                        self.identified_omega = float(np.sqrt(-coeff_y))
            self.amplitude = float(np.max(np.abs(Y)))
        except Exception:
            self.is_fitted = False

    def predict_future_position(self, future_horizon_seconds=0.8):
        """
        Forecast future obstacle position at t + future_horizon_seconds.
        """
        if len(self.history_pos) == 0:
            return np.array([2.8, 0.0, 1.0])

        latest_pos = self.history_pos[-1].copy()
        if not self.is_fitted or len(self.history_pos) < 5:
            return latest_pos

        current_y = latest_pos[1]
        dt = np.mean(np.diff(np.array(self.history_t))) if len(self.history_t) > 1 else self.dt
        vy = (self.history_pos[-1][1] - self.history_pos[-2][1]) / max(1e-5, dt)

        # Analytical harmonic rollout using discovered omega
        omega = self.identified_omega
        tau = future_horizon_seconds
        pred_y = current_y * np.cos(omega * tau) + (vy / max(1e-4, omega)) * np.sin(omega * tau)
        pred_y = np.clip(pred_y, -self.amplitude * 1.1, self.amplitude * 1.1)

        predicted_pos = latest_pos.copy()
        predicted_pos[1] = pred_y
        return predicted_pos

    def get_motion_summary(self):
        """Return discovered motion equation string."""
        if not self.is_fitted:
            return "Collecting trajectory observations..."
        return f"d^2y/dt^2 = -{self.identified_omega**2:.2f} * y (Period: {2*np.pi/max(1e-4, self.identified_omega):.2f}s)"
