"""
Adaptive Neural Brain for reactive drone navigation in MuJoCo.
Implements the Neural Avoidance System (RNN steering network + Synaptic Plasticity & Frustration)
(Devos, Ebeid, Manoonpong, DSD 2018) with directional sector sensitivity.
"""

import math
import numpy as np
import mujoco
from .base_avoidance import BaseObstacleAvoidance

# Body-relative ray fan (degrees, +CCW / left). 0 = current heading.
# Positive angles (+8, +20, +40, +60, +90) are LEFT (+Y).
# Negative angles (-8, -20, -40, -60, -90) are RIGHT (-Y).
RAY_ANGLES_DEG = np.array([-90, -60, -40, -20, -8, 0, 8, 20, 40, 60, 90, 180], dtype=float)


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class NeuralBrainAvoidance(BaseObstacleAvoidance):
    """
    Adaptive Neural Brain (RNN with Synaptic Plasticity & Frustration)
    for Autonomous Obstacle Avoidance in MuJoCo.
    """

    def __init__(
        self,
        goal_position=np.array([6.0, 0.0, 1.0]),
        flight_altitude=1.0,
        control_freq=48.0
    ):
        super().__init__(goal_position, flight_altitude)
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq

        # Configuration parameters
        self.sensor_range = 1.8          # max ray length (m)
        self.react_front = 1.10          # reaction range straight ahead (m)
        self.react_side = 0.40           # reaction range at +-90 deg (m)

        self.v_max = 0.38                # cruise speed (m/s)
        self.v_side = 0.38               # lateral bypass speed (m/s)
        self.v_reverse = 0.20            # max backing speed (m/s)
        self.stop_dist = 0.28            # front distance at which forward speed hits 0
        self.slow_dist = 0.85            # front distance where slowing starts

        # Recurrent steering network
        self.tau = 0.18                  # neuron time constant (s)
        self.w_self_0 = 1.2              # baseline self-excitation
        self.w_self_max = 2.4            # plasticity ceiling
        self.w_cross = -1.6              # mutual inhibition
        self.w_in = 4.5                  # sensory input gain
        self.bias = -2.2                 # neuron bias
        self.w_free = 1.2                # free-space bias gain

        self.mu = 0.8                    # Hebbian learning rate (1/s)
        self.gamma = 0.30                # decay back to baseline (1/s)

        # Frustration / progress monitor
        self.progress_eps = 0.03         # improvement counted as progress (m)
        self.frust_rise = 0.35           # per second without progress
        self.frust_decay = 1.5           # per second after progress
        self.frust_grace = 1.0           # seconds without progress before rising
        self.open_hold = 0.3             # s of open space before frustration decays

        self.goal_tolerance = 0.25

        self.ray_angles = np.radians(RAY_ANGLES_DEG)
        c = np.cos(self.ray_angles)
        s = np.sin(self.ray_angles)
        a, b = self.react_front, self.react_side
        self.react = np.where(c > 0, a * b / np.sqrt((b * c) ** 2 + (a * s) ** 2), b)

        self._geomid_buffer = np.zeros(1, dtype=np.int32)
        self.reset()

    def reset(self):
        """Reset neural states and memory."""
        self.x = np.array([-2.0, -2.0])
        self.o = np.tanh(self.x)
        self.w_self = np.array([self.w_self_0, self.w_self_0])

        self.frustration = 0.0
        self.best_goal_dist = 6.0
        self.time_since_progress = 0.0
        self.time_in_open = 0.0
        self.committed_side = 1          # +1 = steer right (bypass right), -1 = steer left (bypass left)
        self.mode = "FORWARD"
        self.avoidance_events = 0
        self.prev_avoid = False
        self.emergency_reversals = 0

    def cast_rays_mujoco(self, model, data, position, goal_heading):
        """Cast 12 horizontal rays aligned with goal heading in MuJoCo physics engine."""
        world_angles = goal_heading + self.ray_angles
        distances = []
        rays_vis = []
        start_offset = 0.08

        for ang in world_angles:
            ray_dir = np.array([np.cos(ang), np.sin(ang), 0.0])
            ray_start = position + ray_dir * start_offset
            dist = mujoco.mj_ray(model, data, ray_start, ray_dir, None, 1, -1, self._geomid_buffer)
            
            if dist < 0 or dist > self.sensor_range:
                actual_dist = self.sensor_range
                hit = False
            else:
                actual_dist = float(dist) + start_offset
                hit = True

            distances.append(actual_dist)
            end_pt = position + ray_dir * actual_dist

            color = [1.0, 0.1, 0.1, 0.9] if actual_dist < 0.35 else ([1.0, 0.7, 0.1, 0.8] if actual_dist < 0.85 else [0.1, 0.9, 0.2, 0.5])
            rays_vis.append({
                "start": position,
                "end": end_pt,
                "distance": actual_dist,
                "color": color,
                "hit": hit
            })

        return np.array(distances, dtype=float), rays_vis

    def compute_step(self, current_pos, current_vel, perception_data, dt=1.0/48.0, model=None, data=None):
        pos = np.asarray(current_pos, dtype=float)[:2]
        to_goal = self.goal_position[:2] - pos
        goal_dist = float(np.linalg.norm(to_goal))

        if goal_dist < self.goal_tolerance:
            self.mode = "GOAL_REACHED"
            return self.goal_position.copy(), np.zeros(3), {
                "mode": "GOAL_REACHED",
                "status_text": "GOAL REACHED",
                "dist_to_goal": goal_dist,
                "frustration": self.frustration
            }

        goal_dir = to_goal / max(goal_dist, 1e-6)
        goal_heading = math.atan2(goal_dir[1], goal_dir[0])

        # Cast 12-ray fan in MuJoCo
        if model is not None and data is not None:
            d, rays_vis = self.cast_rays_mujoco(model, data, current_pos, goal_heading)
            perception_data["rays"] = rays_vis
        else:
            d = np.array([perception_data["distances"].get("front", 1.8)] * len(RAY_ANGLES_DEG))

        # ---------------- 1. SENSORY PREPROCESSING -------------------
        fwd = np.abs(self.ray_angles) < math.radians(100)
        prox = np.clip(1.0 - d / self.react, 0.0, 1.0) * fwd
        left = self.ray_angles > math.radians(1)
        right = self.ray_angles < math.radians(-1)
        front = ~left & ~right & fwd

        I_L_raw = float(np.max(prox[left])) if np.any(left) else 0.0
        I_R_raw = float(np.max(prox[right])) if np.any(right) else 0.0
        I_front = float(np.max(prox[front])) if np.any(front) else 0.0

        # Preserve Left vs Right asymmetry cleanly
        if I_L_raw > I_R_raw + 0.05:
            # Obstacle clearly on Left -> Excite Left neuron L (to steer Right)
            I_L = max(I_L_raw, I_front)
            I_R = I_R_raw
            self.committed_side = 1   # steer right
        elif I_R_raw > I_L_raw + 0.05:
            # Obstacle clearly on Right -> Excite Right neuron R (to steer Left)
            I_R = max(I_R_raw, I_front)
            I_L = I_L_raw
            self.committed_side = -1  # steer left
        else:
            # Symmetrical frontal obstacle: use free-space bias or committed side to break tie
            open_L_val = float(d[left & fwd].mean() if np.any(left & fwd) else self.sensor_range)
            open_R_val = float(d[right & fwd].mean() if np.any(right & fwd) else self.sensor_range)
            if open_R_val > open_L_val + 0.05:
                I_L = max(I_L_raw, I_front)
                I_R = I_R_raw * 0.3
                self.committed_side = 1
            elif open_L_val > open_R_val + 0.05:
                I_R = max(I_R_raw, I_front)
                I_L = I_L_raw * 0.3
                self.committed_side = -1
            else:
                # Absolute center: tie-break using committed_side
                if self.committed_side >= 0:
                    I_L = max(I_L_raw, I_front)
                    I_R = 0.0
                else:
                    I_R = max(I_R_raw, I_front)
                    I_L = 0.0

        # Free-space preference
        open_L = d[left & fwd].mean() / self.sensor_range if np.any(left & fwd) else 1.0
        open_R = d[right & fwd].mean() / self.sensor_range if np.any(right & fwd) else 1.0
        free_bias = self.w_free * (open_L - open_R)

        # ---------------- 2. RECURRENT STEERING NETWORK --------------
        a_prev = (self.o + 1.0) / 2.0
        drive_L = (self.w_self[0] * a_prev[0] + self.w_cross * a_prev[1]
                   + self.w_in * I_L + self.bias - free_bias)
        drive_R = (self.w_self[1] * a_prev[1] + self.w_cross * a_prev[0]
                   + self.w_in * I_R + self.bias + free_bias)

        self.x += (self.dt / self.tau) * (-self.x + np.array([drive_L, drive_R]))
        self.o = np.tanh(self.x)

        act = (self.o + 1.0) / 2.0
        steer = act[0] - act[1]    # >0 -> obstacle on Left -> steer Right (-Y); <0 -> obstacle on Right -> steer Left (+Y)
        avoid_activity = float(np.clip(act.max() * 1.4, 0.0, 1.0))

        # ---------------- 3. SYNAPTIC PLASTICITY ---------------------
        pre = np.array([I_L, I_R])
        hebb = self.mu * (1.0 + 2.0 * self.frustration) * pre * act
        self.w_self += (hebb - self.gamma * (self.w_self - self.w_self_0)) * self.dt
        self.w_self = np.clip(self.w_self, self.w_self_0, self.w_self_max)

        # ---------------- 4. FRUSTRATION (PROGRESS MONITOR) ----------
        if goal_dist < self.best_goal_dist - self.progress_eps:
            self.best_goal_dist = goal_dist
            self.time_since_progress = 0.0
        else:
            self.time_since_progress += self.dt

        if avoid_activity < 0.20:
            self.time_in_open += self.dt
        else:
            self.time_in_open = 0.0

        if self.time_since_progress > self.frust_grace and self.time_in_open < self.open_hold:
            self.frustration = min(1.0, self.frustration + self.frust_rise * self.dt)
        elif self.time_in_open >= self.open_hold:
            self.frustration = max(0.0, self.frustration - self.frust_decay * self.dt)

        # ---------------- 5. SPEED & DIRECTION SYNTHESIS -------------
        front_cone = np.abs(self.ray_angles) <= math.radians(21)
        d_front = d[front_cone].min()
        s_val = (d_front - self.stop_dist) / (self.slow_dist - self.stop_dist)
        s_val = float(np.clip(s_val, -1.0, 1.0))

        # Forward unit vector toward goal
        u_fwd = np.array([goal_dir[0], goal_dir[1], 0.0])
        # Lateral unit vector (+90 deg left)
        u_lat = np.array([-goal_dir[1], goal_dir[0], 0.0])

        if s_val >= 0:
            # Forward motion throttled by front proximity and goal arrival
            goal_brake = min(1.0, goal_dist / 0.60)
            v_fwd = self.v_max * s_val * goal_brake
            # Lateral motion: steer > 0 (obstacle Left) -> v_lat < 0 (steer Right)
            #                 steer < 0 (obstacle Right) -> v_lat > 0 (steer Left)
            v_lat = -self.v_side * steer
            desired_vel = v_fwd * u_fwd + v_lat * u_lat
            self.mode = "AVOID" if avoid_activity > 0.25 else "FORWARD"
        else:
            # Reversing when front is dangerously blocked
            desired_vel = -self.v_reverse * u_fwd + (-self.v_side * steer * 0.8) * u_lat
            self.mode = "EMERGENCY_REVERSE"

        if avoid_activity > 0.25:
            if not self.prev_avoid:
                self.avoidance_events += 1
            self.prev_avoid = True
        else:
            self.prev_avoid = False

        if goal_dist < 0.40:
            target_pos = self.goal_position.copy()
            desired_vel = np.zeros(3)
        else:
            target_pos = current_pos + desired_vel * 0.65
            target_pos[2] = self.flight_altitude

        status_text = (
            f"NEURAL AVOID (F={self.frustration:.2f})" if self.mode == "AVOID" else
            f"NEURAL REVERSE (F={self.frustration:.2f})" if self.mode == "EMERGENCY_REVERSE" else
            f"NEURAL TRACKING (F={self.frustration:.2f})"
        )

        return target_pos, desired_vel, {
            "mode": self.mode,
            "status_text": status_text,
            "dist_to_goal": goal_dist,
            "frustration": self.frustration,
            "avoidance_events": self.avoidance_events
        }
