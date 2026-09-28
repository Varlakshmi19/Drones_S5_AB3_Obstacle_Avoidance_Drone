"""
Adaptive Neural Brain for reactive drone navigation.

Simulator-agnostic: it only needs (a) range readings at a fixed fan of
body-relative angles and (b) the drone's horizontal position and the goal.
It returns a horizontal world-frame velocity command.

Inspired by the base paper (Devos, Ebeid, Manoonpong, DSD 2018), which uses a
small recurrent neural network with synaptic plasticity whose short-term
memory lets the drone keep turning after the obstacle leaves the sensor
field -> escapes corners and deadlocks.

Network (all discrete-time, non-spiking, tanh neurons):

    L, R  : steering neurons  (self-excitation w_ii = plastic memory,
                               mutual inhibition w_ij = decision / tie-break)
    S     : speed neuron      (driven by frontal proximity -> slow / reverse)
    F     : frustration unit  (leaky integrator of "no progress to goal")

    heading rate  omega = omega_goal * (1 - avoid_activity) - K_TURN*(o_L - o_R)
    speed         v     = V_MAX * speed_factor(S)
    command       v_cmd = v * [cos(psi), sin(psi)]

Plasticity (Hebbian growth + decay to baseline):
    w_ii <- w_ii + MU * (1 + F) * pre_i * post_i - GAMMA * (w_ii - W_SELF_0)
Repeated/persistent obstacle exposure (corners, pockets) strengthens the
self-connection -> longer hysteresis -> longer committed turn.

Frustration F grows while the drone fails to improve its best distance to
the goal; it (1) scales plasticity, (2) rotates the goal-attraction direction
toward the committed side (detour), and (3) decays once progress resumes.
"""

from dataclasses import dataclass, field
import math
import numpy as np


# Body-relative ray fan (degrees, +CCW / left). 0 = current heading.
RAY_ANGLES_DEG = np.array([-90, -60, -40, -20, -8, 0, 8, 20, 40, 60, 90, 180], dtype=float)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class BrainConfig:
    sensor_range: float = 1.6          # max ray length (m)
    # "Safety bubble": each ray's reaction range shrinks toward the sides,
    # so walls beside the drone in a corridor don't trigger turning.
    react_front: float = 1.10          # reaction range straight ahead (m)
    react_side: float = 0.32           # reaction range at +-90 deg (m)

    v_max: float = 0.40                # cruise speed (m/s)
    v_reverse: float = 0.18            # max backing speed (m/s)
    stop_dist: float = 0.28            # front distance at which speed hits 0
    slow_dist: float = 0.85            # front distance where slowing starts

    omega_max: float = 2.6             # max heading rate (rad/s)
    k_goal: float = 2.2                # goal attraction gain
    k_turn: float = 2.6                # neural steering gain

    # Recurrent steering network
    tau: float = 0.18                  # neuron time constant (s)
    w_self_0: float = 1.4              # baseline self-excitation (memory)
    w_self_max: float = 3.2            # plasticity ceiling (below the ~3.6 latch point)
    w_cross: float = -1.6              # mutual inhibition
    w_in: float = 4.0                  # sensory input gain
    bias: float = -2.2                 # neuron bias (keeps them off when clear)
    w_free: float = 0.8                # free-space bias gain (side preference)

    mu: float = 0.9                    # Hebbian learning rate (1/s)
    gamma: float = 0.15                # decay back to baseline (1/s)

    # Frustration / progress monitor
    progress_eps: float = 0.03         # improvement counted as progress (m)
    frust_rise: float = 0.35           # per second without progress
    frust_decay: float = 0.5           # per second after progress
    frust_grace: float = 1.0           # seconds without progress before rising
    open_hold: float = 1.0             # s of open space before frustration decays
    detour_max_deg: float = 80.0       # goal-direction rotation at F = 1

    goal_tolerance: float = 0.25


class AdaptiveNeuralBrain:

    def __init__(self, goal_xy, start_xy, dt, config: BrainConfig = None):
        self.cfg = config or BrainConfig()
        self.dt = float(dt)
        self.goal = np.asarray(goal_xy, dtype=float)[:2]
        start = np.asarray(start_xy, dtype=float)[:2]

        g = self.goal - start
        self.psi = math.atan2(g[1], g[0])   # heading (rad, world)

        self.x = np.array([-2.0, -2.0])     # [L, R] membrane potentials
        self.o = np.tanh(self.x)            # [L, R] neuron outputs
        self.w_self = np.array([self.cfg.w_self_0, self.cfg.w_self_0])

        self.frustration = 0.0
        self.best_goal_dist = float(np.linalg.norm(g))
        self.time_since_progress = 0.0
        self.time_in_open = 0.0
        self.committed_side = 1             # +1 = left, -1 = right (default tie-break: left)

        self.ray_angles = np.radians(RAY_ANGLES_DEG)
        # per-ray reaction range: elliptical bubble (long ahead, short sideways)
        c = np.cos(self.ray_angles)
        s = np.sin(self.ray_angles)
        a, b = self.cfg.react_front, self.cfg.react_side
        self.react = np.where(c > 0, a * b / np.sqrt((b * c) ** 2 + (a * s) ** 2), b)

        self.last = {}

    # ------------------------------------------------------------------
    def ray_world_angles(self):
        """World-frame angles at which the simulator should cast rays."""
        return self.psi + self.ray_angles

    # ------------------------------------------------------------------
    def step(self, position_xy, distances):
        """
        position_xy : current (x, y)
        distances   : array of ray distances, same order as RAY_ANGLES_DEG
        returns     : (vx, vy) world velocity command, done flag
        """
        cfg = self.cfg
        pos = np.asarray(position_xy, dtype=float)[:2]
        d = np.clip(np.asarray(distances, dtype=float), 0.0, cfg.sensor_range)

        to_goal = self.goal - pos
        goal_dist = float(np.linalg.norm(to_goal))
        if goal_dist < cfg.goal_tolerance:
            self.last = dict(mode="GOAL", goal_dist=goal_dist)
            return np.zeros(2), True

        # ---------------- sensory preprocessing ----------------------
        fwd = np.abs(self.ray_angles) < math.radians(100)       # exclude rear
        prox = np.clip(1.0 - d / self.react, 0.0, 1.0) * fwd     # 0 clear .. 1 contact
        left = self.ray_angles > math.radians(1)
        right = self.ray_angles < math.radians(-1)
        front = ~left & ~right & fwd

        # front ray counts for both sides (dead-ahead obstacle)
        I_L = max(prox[left].max(), prox[front].max())
        I_R = max(prox[right].max(), prox[front].max())

        # free-space preference: which side is more open further out
        open_L = d[left & fwd].mean() / cfg.sensor_range
        open_R = d[right & fwd].mean() / cfg.sensor_range
        free_bias = cfg.w_free * (open_L - open_R)   # >0 -> prefer turning left

        # ---------------- recurrent steering network -----------------
        # Obstacle on the LEFT should excite L, which steers RIGHT.
        # The free-space bias pushes the neuron on the *blocked* side, so
        # a symmetric dead-ahead obstacle resolves toward the open side.
        a_prev = (self.o + 1.0) / 2.0               # firing rates in [0, 1]
        side_hint = 0.08 * self.committed_side      # tiny tie-breaker (memory of last choice)
        drive_L = (self.w_self[0] * a_prev[0] + cfg.w_cross * a_prev[1]
                   + cfg.w_in * I_L + cfg.bias - free_bias - side_hint)
        drive_R = (self.w_self[1] * a_prev[1] + cfg.w_cross * a_prev[0]
                   + cfg.w_in * I_R + cfg.bias + free_bias + side_hint)
        # leaky integration -> memory measured in seconds, not control ticks
        self.x += (self.dt / cfg.tau) * (-self.x + np.array([drive_L, drive_R]))
        self.o = np.tanh(self.x)

        act = (self.o + 1.0) / 2.0                  # activity in [0, 1]
        steer = act[0] - act[1]                     # >0 -> obstacle left -> turn right
        avoid_activity = float(np.clip(act.max() * 1.4, 0.0, 1.0))

        if abs(steer) > 0.15:
            self.committed_side = -1 if steer > 0 else +1   # side we're turning toward

        # ---------------- synaptic plasticity ------------------------
        pre = np.array([I_L, I_R])
        hebb = cfg.mu * (1.0 + 2.0 * self.frustration) * pre * act
        self.w_self += (hebb - cfg.gamma * (self.w_self - cfg.w_self_0)) * self.dt
        self.w_self = np.clip(self.w_self, cfg.w_self_0, cfg.w_self_max)

        # ---------------- frustration (progress monitor) -------------
        if goal_dist < self.best_goal_dist - cfg.progress_eps:
            self.best_goal_dist = goal_dist
            self.time_since_progress = 0.0
        else:
            self.time_since_progress += self.dt

        if avoid_activity < 0.25:
            self.time_in_open += self.dt
        else:
            self.time_in_open = 0.0

        if self.time_since_progress > cfg.frust_grace and self.time_in_open < cfg.open_hold:
            # struggling near obstacles without getting closer -> frustration rises
            self.frustration = min(1.0, self.frustration + cfg.frust_rise * self.dt)
        elif self.time_in_open >= cfg.open_hold:
            # sustained open space -> calm down, goal attraction returns
            self.frustration = max(0.0, self.frustration - cfg.frust_decay * self.dt)

        # ---------------- heading dynamics ---------------------------
        goal_bearing = math.atan2(to_goal[1], to_goal[0])
        side = self.committed_side if self.committed_side != 0 else 1
        detour = side * math.radians(cfg.detour_max_deg) * self.frustration
        goal_err = wrap(goal_bearing + detour - self.psi)

        # Exploration drive (active only when frustrated): pull the heading
        # toward the most open direction, biased to the committed side.
        score = d / cfg.sensor_range * (1.0 + 0.35 * side * np.sin(self.ray_angles))
        open_err = wrap(float(self.ray_angles[int(np.argmax(score))]))

        F = self.frustration
        attract_gain = max(1.0 - avoid_activity, 0.6 * F)
        attract_err = (1.0 - F) * goal_err + F * open_err if avoid_activity > 0.25 else goal_err

        omega = (cfg.k_goal * attract_err * attract_gain
                 - cfg.k_turn * steer * (1.0 - 0.5 * F))
        omega = float(np.clip(omega, -cfg.omega_max, cfg.omega_max))
        self.psi = wrap(self.psi + omega * self.dt)

        # ---------------- speed neuron -------------------------------
        front_cone = np.abs(self.ray_angles) <= math.radians(21)
        d_front = d[front_cone].min()
        s = (d_front - cfg.stop_dist) / (cfg.slow_dist - cfg.stop_dist)
        s = float(np.clip(s, -1.0, 1.0))
        # also slow down while turning hard
        turn_factor = 1.0 - 0.5 * abs(omega) / cfg.omega_max
        if s >= 0:
            speed = cfg.v_max * s * turn_factor
            speed = max(speed, 0.06 if d_front > cfg.stop_dist else 0.0)
        else:
            rear_clear = d[-1] > 0.35
            speed = cfg.v_reverse * s if rear_clear else 0.0

        v = speed * np.array([math.cos(self.psi), math.sin(self.psi)])

        self.last = dict(
            mode=("AVOID" if avoid_activity > 0.3 else "GOAL_SEEK") if speed >= 0 else "BACK_OFF",
            I_L=I_L, I_R=I_R, o_L=self.o[0], o_R=self.o[1],
            w_LL=self.w_self[0], w_RR=self.w_self[1],
            frustration=self.frustration, psi=self.psi, omega=omega,
            speed=speed, d_front=d_front, goal_dist=goal_dist,
        )
        return v, False
