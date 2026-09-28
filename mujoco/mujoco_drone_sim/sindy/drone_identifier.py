"""
Drone Dynamic System Identification with SINDy (Sparse Identification of Nonlinear Dynamics).
Discovers symbolic mathematical equations of motion for the MuJoCo quadcopter from flight telemetry.
"""

import numpy as np
from .sindy_core import SINDy, PolynomialLibrary, finite_difference


class DroneDynamicsIdentifier:
    """
    Data-driven identification of quadcopter translational and rotational dynamics.
    Learns:
      1. Translational acceleration: [a_x, a_y, a_z] = f(v, roll, pitch, yaw, total_thrust)
      2. Angular acceleration: [alpha_x, alpha_y, alpha_z] = f(omega, roll, pitch, motor_moments)
    """

    def __init__(self, dt=1.0 / 48.0, poly_degree=2, threshold=0.03):
        self.dt = dt
        self.poly_degree = poly_degree
        self.threshold = threshold

        # SINDy model for translational dynamics
        self.trans_model = SINDy(
            library=PolynomialLibrary(degree=poly_degree, include_bias=True),
            threshold=threshold,
            alpha=1e-5
        )

        # SINDy model for rotational dynamics
        self.rot_model = SINDy(
            library=PolynomialLibrary(degree=poly_degree, include_bias=True),
            threshold=threshold,
            alpha=1e-5
        )

        self.is_fitted = False
        self.trans_scores = None
        self.rot_scores = None

    def fit_from_telemetry(self, flight_log):
        """
        Fit SINDy models from recorded flight history.
        flight_log: list of dicts with 'pos', 'vel', 'quat', 'ang_vel', 'ctrl' (motor thrusts)
        """
        n_samples = len(flight_log)
        if n_samples < 20:
            raise ValueError(f"Insufficient telemetry samples ({n_samples}) for SINDy identification (minimum 20 required).")

        # Extract arrays
        pos = np.array([f["pos"] for f in flight_log])
        vel = np.array([f["vel"] for f in flight_log])
        angvel = np.array([f.get("angvel", np.zeros(3)) for f in flight_log])
        
        # Extract motor controls if available
        ctrl = np.array([f.get("ctrl", np.full(4, 0.08)) for f in flight_log])
        total_thrust = np.sum(ctrl, axis=1, keepdims=True)

        # Extract Euler angles (roll, pitch, yaw) from quaternion or position/accel
        quats = [f.get("quat", [1.0, 0.0, 0.0, 0.0]) for f in flight_log]
        eulers = []
        for q in quats:
            w, x, y, z = q
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (w * y - z * x)
            pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
            eulers.append([roll, pitch, yaw])
        eulers = np.array(eulers)

        # 1. Compute state derivatives via finite difference
        accel = finite_difference(vel, self.dt, smooth_window=5)
        ang_accel = finite_difference(angvel, self.dt, smooth_window=5)

        # 2. Translational SINDy: State = [vx, vy, vz], Inputs = [roll, pitch, yaw, thrust]
        X_trans = vel
        U_trans = np.hstack([eulers, total_thrust])
        state_names_trans = ["v_x", "v_y", "v_z"]
        input_names_trans = ["roll", "pitch", "yaw", "T_tot"]

        self.trans_model.fit(
            X_trans, accel, U_trans,
            state_names=state_names_trans,
            input_names=input_names_trans
        )
        self.trans_scores = self.trans_model.score(X_trans, accel, U_trans)

        # 3. Rotational SINDy: State = [wx, wy, wz], Inputs = [u1, u2, u3, u4]
        X_rot = angvel
        U_rot = ctrl
        state_names_rot = ["w_x", "w_y", "w_z"]
        input_names_rot = ["u_1", "u_2", "u_3", "u_4"]

        self.rot_model.fit(
            X_rot, ang_accel, U_rot,
            state_names=state_names_rot,
            input_names=input_names_rot
        )
        self.rot_scores = self.rot_model.score(X_rot, ang_accel, U_rot)

        self.is_fitted = True
        return {
            "trans_scores": self.trans_scores.tolist(),
            "rot_scores": self.rot_scores.tolist(),
            "trans_equations": self.trans_model.equations(precision=3),
            "rot_equations": self.rot_model.equations(precision=3)
        }

    def predict_next_state(self, current_pos, current_vel, current_euler, total_thrust, dt=None):
        """
        Predict position and velocity after dt using discovered SINDy equations.
        """
        if dt is None:
            dt = self.dt
        if not self.is_fitted:
            # Fallback kinematic step
            next_pos = current_pos + current_vel * dt
            return next_pos, current_vel

        X = current_vel.reshape(1, 3)
        U = np.hstack([current_euler, [total_thrust]]).reshape(1, 4)
        accel_pred = self.trans_model.predict(X, U)[0]

        next_vel = current_vel + accel_pred * dt
        next_pos = current_pos + (current_vel + next_vel) * 0.5 * dt
        return next_pos, next_vel

    def forward_rollout(self, start_pos, start_vel, target_heading_deg, target_speed, horizon_steps=15):
        """
        Rollout candidate forward trajectory using SINDy dynamic equations.
        """
        rad = np.radians(target_heading_deg)
        target_dir = np.array([np.cos(rad), np.sin(rad), 0.0])
        target_vel = target_dir * target_speed

        trajectory = [start_pos.copy()]
        cur_pos = start_pos.copy()
        cur_vel = start_vel.copy()

        # Approximate roll/pitch from desired acceleration
        desired_acc = (target_vel - cur_vel) / (horizon_steps * self.dt)
        pitch = np.clip(desired_acc[0] / 9.81, -0.3, 0.3)
        roll = np.clip(-desired_acc[1] / 9.81, -0.3, 0.3)
        euler = np.array([roll, pitch, rad])
        hover_thrust = 0.033 * 9.81

        for _ in range(horizon_steps):
            cur_pos, cur_vel = self.predict_next_state(cur_pos, cur_vel, euler, hover_thrust, self.dt)
            trajectory.append(cur_pos.copy())

        return np.array(trajectory)

    def print_summary(self):
        """Print discovered symbolic equations and R^2 accuracy."""
        print("\n" + "=" * 75)
        print("  SINDy DATA-DRIVEN QUADCOPTER DYNAMICS IDENTIFICATION")
        print("=" * 75)
        if not self.is_fitted:
            print("[WARN] Model not yet fitted.")
            return

        print("\n[TRANSLATIONAL ACCELERATION EQUATIONS]:")
        for eq, r2 in zip(self.trans_model.equations(precision=3), self.trans_scores):
            print(f"  {eq}  (R^2 = {r2:.4f})")

        print("\n[ROTATIONAL ANGULAR ACCELERATION EQUATIONS]:")
        for eq, r2 in zip(self.rot_model.equations(precision=3), self.rot_scores):
            print(f"  {eq}  (R^2 = {r2:.4f})")
        print("=" * 75 + "\n")
