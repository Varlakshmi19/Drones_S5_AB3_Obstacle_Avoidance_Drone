"""
DSL PID Flight Controller for MuJoCo Quadcopter.
Directly implements the SE(3) Geometric and Cascaded PID control formulation
matching the Crazyflie 2.X DSLPID controller from gym-pybullet-drones.
"""

import numpy as np
import mujoco


def quat_to_rot_matrix(quat):
    """Convert MuJoCo quaternion [w, x, y, z] to 3x3 rotation matrix."""
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, quat)
    return mat.reshape(3, 3)


def quat_to_euler(quat):
    """Convert MuJoCo quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw)."""
    w, x, y, z = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = np.copysign(np.pi / 2.0, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class DSLPIDControlMuJoCo:
    """
    Precision Geometric & Cascaded PID Flight Controller for MuJoCo Quadcopter.
    Calibrated against MuJoCo physics body inertia, actuator sites, and aerodynamic drag.
    """

    def __init__(
        self,
        mass=0.0472,
        arm_length=0.065,
        gravity=9.81,
        control_freq=48.0
    ):
        self.mass = mass
        self.arm_length = arm_length
        self.gravity = gravity
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq

        self.max_thrust = 0.30  # Max thrust per motor (N)

        # Calibrated Motor Allocation Inverse Matrix
        # Maps [Total_Thrust, Alpha_Roll, Alpha_Pitch, Alpha_Yaw] -> [f1, f2, f3, f4]
        # Calibrated for Crazyflie X-geometry in MuJoCo
        self.B_inv = np.array([
            [ 0.25,       -0.00033536, -0.00033692,  0.00695429],
            [ 0.25,       -0.00033536,  0.00033692, -0.00695429],
            [ 0.25,        0.00033536,  0.00033692,  0.00695429],
            [ 0.25,        0.00033536, -0.00033692, -0.00695429]
        ])

        # Position Gains
        self.kp_pos = np.array([3.2, 3.2, 22.0])
        self.kd_pos = np.array([2.4, 2.4, 8.5])
        self.ki_pos = np.array([0.05, 0.05, 1.5])

        # Attitude Gains (Angular Accelerations)
        self.kp_att = np.array([65.0, 65.0, 45.0])
        self.kd_att = np.array([13.0, 13.0, 9.0])

        # Integral error accumulators
        self.integral_pos = np.zeros(3)

        # Max tilt angle (rad)
        self.max_tilt = np.radians(25.0)

    def reset(self):
        """Reset PID integral terms."""
        self.integral_pos = np.zeros(3)

    def compute_control(
        self,
        cur_pos,
        cur_quat,
        cur_vel,
        cur_ang_vel,
        target_pos,
        target_vel=None,
        target_rpy=None
    ):
        """
        Compute actuator motor thrusts for MuJoCo simulation.
        """
        if target_vel is None:
            target_vel = np.zeros(3)
        if target_rpy is None:
            target_rpy = np.zeros(3)

        # ------------------------------------------------------------
        # 1. POSITION & VELOCITY ERRORS
        # ------------------------------------------------------------
        pos_err = target_pos - cur_pos
        vel_err = target_vel - cur_vel

        self.integral_pos += pos_err * self.dt
        self.integral_pos = np.clip(self.integral_pos, -0.4, 0.4)

        acc_des = (
            self.kp_pos * pos_err
            + self.kd_pos * vel_err
            + self.ki_pos * self.integral_pos
        )
        acc_des[2] += self.gravity

        # ------------------------------------------------------------
        # 2. ORIENTATION & ATTITUDE EXTRACTION
        # ------------------------------------------------------------
        cur_euler = quat_to_euler(cur_quat)
        roll, pitch, yaw = cur_euler
        p, q, r = cur_ang_vel

        # ------------------------------------------------------------
        # 3. ATTITUDE REFERENCE GENERATION
        # ------------------------------------------------------------
        target_yaw = target_rpy[2]

        # In this MuJoCo body frame:
        # pitch > 0 (+Y rotation) tilts nose forward -> acc_x > 0
        # roll > 0 (+X rotation) tilts left side up / right side down -> acc_y < 0
        des_pitch = np.clip(acc_des[0] / self.gravity, -self.max_tilt, self.max_tilt)
        des_roll = np.clip(-acc_des[1] / self.gravity, -self.max_tilt, self.max_tilt)

        # ------------------------------------------------------------
        # 4. ATTITUDE PID & ANGULAR ACCELERATIONS
        # ------------------------------------------------------------
        err_roll = des_roll - roll
        err_pitch = des_pitch - pitch
        err_yaw = normalize_angle(target_yaw - yaw)

        alpha_x = self.kp_att[0] * err_roll - self.kd_att[0] * p
        alpha_y = self.kp_att[1] * err_pitch - self.kd_att[1] * q
        alpha_z = self.kp_att[2] * err_yaw - self.kd_att[2] * r

        # ------------------------------------------------------------
        # 5. COLLECTIVE THRUST & MOTOR ALLOCATION
        # ------------------------------------------------------------
        total_thrust = (self.mass * acc_des[2]) / (np.cos(roll) * np.cos(pitch) + 1e-4)
        total_thrust = np.clip(total_thrust, 0.05, 4.0 * self.max_thrust * 0.95)

        cmd_vec = np.array([total_thrust, alpha_x, alpha_y, alpha_z])
        motor_thrusts = self.B_inv @ cmd_vec
        motor_thrusts = np.clip(motor_thrusts, 0.0, self.max_thrust)

        return motor_thrusts, {
            "total_thrust": total_thrust,
            "des_roll": des_roll,
            "des_pitch": des_pitch,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "pos_error": pos_err
        }
