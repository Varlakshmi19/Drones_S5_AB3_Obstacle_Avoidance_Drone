"""
High-Performance Cascaded SE(3) / PID Flight Controller for Quadcopter in MuJoCo.
Handles Position, Velocity, Attitude (Roll/Pitch/Yaw), and Angular Rate loops with motor allocation.
"""

import numpy as np


def quat_to_euler(quat):
    """
    Convert MuJoCo quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw) in radians.
    """
    w, x, y, z = quat
    
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = np.copysign(np.pi / 2.0, sinp)
    else:
        pitch = np.arcsin(sinp)
        
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([roll, pitch, yaw])


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class DronePIDController:
    """
    Cascaded PID Controller for autonomous quadcopter navigation and stabilization.
    """

    def __init__(
        self,
        mass=0.038,  # Total mass of drone (kg)
        arm_length=0.065, # Moment arm length (m)
        max_thrust_per_motor=0.30, # Max thrust per motor (N)
        gravity=9.81
    ):
        self.mass = mass
        self.arm_length = arm_length
        self.max_thrust = max_thrust_per_motor
        self.gravity = gravity
        self.hover_thrust = (self.mass * self.gravity) / 4.0

        # Position Loop Gains (Horizontal x, y)
        self.kp_pos_xy = 1.8
        self.kd_pos_xy = 1.2
        self.ki_pos_xy = 0.05

        # Altitude Loop Gains (Vertical z)
        self.kp_pos_z = 18.0
        self.kd_pos_z = 8.5
        self.ki_pos_z = 2.5

        # Attitude Loop Gains (Roll, Pitch)
        self.kp_att_rp = 0.045
        self.kd_att_rp = 0.008
        self.ki_att_rp = 0.001

        # Yaw Loop Gains
        self.kp_yaw = 0.025
        self.kd_yaw = 0.006

        # Integral error accumulators
        self.integral_pos_xy = np.zeros(2)
        self.integral_pos_z = 0.0
        self.integral_att_rp = np.zeros(2)

        # Limits
        self.max_tilt_angle = np.radians(25.0) # max roll/pitch tilt in radians
        self.max_yaw_rate = np.radians(180.0)

    def reset(self):
        """Reset integrator states."""
        self.integral_pos_xy = np.zeros(2)
        self.integral_pos_z = 0.0
        self.integral_att_rp = np.zeros(2)

    def compute_control(
        self,
        current_pos,
        current_vel,
        current_quat,
        current_angvel,
        target_pos,
        target_vel=None,
        target_yaw=0.0,
        dt=0.002
    ):
        """
        Compute 4 individual motor thrust commands from state and target references.

        Args:
            current_pos: [x, y, z] in world frame
            current_vel: [vx, vy, vz] in world frame
            current_quat: [w, x, y, z] orientation quaternion
            current_angvel: [p, q, r] body angular rates (rad/s)
            target_pos: [x, y, z] target waypoint
            target_vel: [vx, vy, vz] feedforward target velocity (optional)
            target_yaw: desired heading in radians
            dt: controller timestep
        """
        if target_vel is None:
            target_vel = np.zeros(3)

        current_euler = quat_to_euler(current_quat)
        roll, pitch, yaw = current_euler
        p, q, r = current_angvel

        # ============================================================
        # 1. POSITION / VELOCITY LOOP
        # ============================================================
        pos_error = target_pos - current_pos
        vel_error = target_vel - current_vel

        # Integrate position error (with anti-windup clamping)
        self.integral_pos_xy += pos_error[:2] * dt
        self.integral_pos_xy = np.clip(self.integral_pos_xy, -1.0, 1.0)

        self.integral_pos_z += pos_error[2] * dt
        self.integral_pos_z = np.clip(self.integral_pos_z, -0.5, 0.5)

        # Target Horizontal Acceleration (world frame)
        acc_des_x = (
            self.kp_pos_xy * pos_error[0]
            + self.kd_pos_xy * vel_error[0]
            + self.ki_pos_xy * self.integral_pos_xy[0]
        )
        acc_des_y = (
            self.kp_pos_xy * pos_error[1]
            + self.kd_pos_xy * vel_error[1]
            + self.ki_pos_xy * self.integral_pos_xy[1]
        )

        # Target Vertical Acceleration
        acc_des_z = (
            self.kp_pos_z * pos_error[2]
            + self.kd_pos_z * vel_error[2]
            + self.ki_pos_z * self.integral_pos_z
        )

        # Total Desired Collective Thrust (N)
        # Compensate for gravity and tilt angle
        cos_tilt = max(np.cos(roll) * np.cos(pitch), 0.5)
        total_thrust = self.mass * (self.gravity + acc_des_z) / cos_tilt
        total_thrust = np.clip(total_thrust, 0.05 * self.mass * self.gravity, 4.0 * self.max_thrust * 0.95)

        # ============================================================
        # 2. DESIRED ATTITUDE (Roll & Pitch from Accel & Heading)
        # ============================================================
        # Rotate desired world accelerations into drone heading
        acc_fwd = acc_des_x * np.cos(yaw) + acc_des_y * np.sin(yaw)
        acc_lat = -acc_des_x * np.sin(yaw) + acc_des_y * np.cos(yaw)

        # Desired Pitch tilts nose down/up along forward direction (+X forward => negative pitch tilts down)
        des_pitch = np.clip(acc_fwd / self.gravity, -self.max_tilt_angle, self.max_tilt_angle)
        # Desired Roll tilts right/left along lateral direction (+Y left => positive roll tilts right)
        des_roll = np.clip(-acc_lat / self.gravity, -self.max_tilt_angle, self.max_tilt_angle)

        # ============================================================
        # 3. ATTITUDE & RATE LOOP (Torques)
        # ============================================================
        roll_error = des_roll - roll
        pitch_error = des_pitch - pitch
        yaw_error = normalize_angle(target_yaw - yaw)

        self.integral_att_rp[0] += roll_error * dt
        self.integral_att_rp[1] += pitch_error * dt
        self.integral_att_rp = np.clip(self.integral_att_rp, -0.2, 0.2)

        tau_x = (
            self.kp_att_rp * roll_error
            - self.kd_att_rp * p
            + self.ki_att_rp * self.integral_att_rp[0]
        )
        tau_y = (
            self.kp_att_rp * pitch_error
            - self.kd_att_rp * q
            + self.ki_att_rp * self.integral_att_rp[1]
        )
        tau_z = (
            self.kp_yaw * yaw_error
            - self.kd_yaw * r
        )

        # ============================================================
        # 4. MOTOR ALLOCATION MIXER (X-Configuration)
        # ============================================================
        # Motors:
        # 1: Front-Right (+x, -y) CW
        # 2: Rear-Right  (-x, -y) CCW
        # 3: Rear-Left   (-x, +y) CW
        # 4: Front-Left  (+x, +y) CCW
        d = self.arm_length / np.sqrt(2.0)
        c_yaw = 0.006

        base_t = total_thrust / 4.0
        t_roll = tau_x / (4.0 * d)
        t_pitch = tau_y / (4.0 * d)
        t_yaw = tau_z / (4.0 * c_yaw)

        # Combine
        f1 = base_t - t_roll + t_pitch + t_yaw
        f2 = base_t - t_roll - t_pitch - t_yaw
        f3 = base_t + t_roll - t_pitch + t_yaw
        f4 = base_t + t_roll + t_pitch - t_yaw

        motor_thrusts = np.array([f1, f2, f3, f4])
        motor_thrusts = np.clip(motor_thrusts, 0.0, self.max_thrust)

        return motor_thrusts, {
            "total_thrust": total_thrust,
            "des_roll": des_roll,
            "des_pitch": des_pitch,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "pos_error": pos_error
        }
