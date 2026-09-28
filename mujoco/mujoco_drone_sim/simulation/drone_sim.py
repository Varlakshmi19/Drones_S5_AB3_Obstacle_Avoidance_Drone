"""
Core Simulation Engine for Quadcopter Obstacle Detection & Avoidance in MuJoCo.
Coordinates physics sub-stepping, perception, avoidance state machines, and flight control.
"""

import time
import numpy as np
import mujoco

from ..models.scene_generator import generate_scene_xml
from ..controllers.dsl_pid_controller import DSLPIDControlMuJoCo
from ..perception.lidar_sensor import LiDARPerceptionSystem
from ..avoidance.reactive_avoidance import ReactiveObstacleAvoidance
from ..avoidance.apf_avoidance import APFObstacleAvoidance
from ..avoidance.dwa_avoidance import DWAObstacleAvoidance
from ..avoidance.avoidance_3d import Avoidance3D
from ..avoidance.sindy_dwa_avoidance import SINDyDWAAvoidance


class DroneObstacleSimulation:
    """
    MuJoCo Simulation environment for drone obstacle avoidance.
    """

    def __init__(
        self,
        scenario_id=1,
        algorithm="reactive",
        start_pos=(0.0, 0.0, 1.0),
        goal_pos=(6.0, 0.0, 1.0),
        control_freq=48.0,
        sim_timestep=0.002,
        sub_scenario="closed_ring",
        seed=42,
        intruder_speed_multiplier=1.0
    ):
        self.scenario_id = scenario_id
        self.algorithm_name = algorithm.lower()
        self.start_pos = np.array(start_pos, dtype=float)
        self.goal_pos = np.array(goal_pos, dtype=float)
        self.control_freq = control_freq
        self.sim_timestep = sim_timestep
        self.sub_scenario = sub_scenario
        self.seed = seed
        self.intruder_speed_multiplier = intruder_speed_multiplier
        self.substeps = int(round((1.0 / self.control_freq) / self.sim_timestep))

        # Build and load MuJoCo model
        self.xml_string = generate_scene_xml(
            scenario_id=self.scenario_id,
            start_pos=self.start_pos,
            goal_pos=self.goal_pos,
            sub_scenario=self.sub_scenario,
            seed=self.seed
        )
        self.model = mujoco.MjModel.from_xml_string(self.xml_string)
        self.data = mujoco.MjData(self.model)

        # Track drone body, geoms, and obstacle geoms
        self.drone_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "drone")
        self.drone_geom_ids = set()
        self.obstacle_geom_ids = set()

        for g_id in range(self.model.ngeom):
            g_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g_id)
            if not g_name:
                continue
            if g_name.startswith("drone") or g_name.startswith("arm") or g_name.startswith("m") or g_name.startswith("prop") or g_name.startswith("front_") or g_name.startswith("rear_"):
                self.drone_geom_ids.add(g_id)
            elif g_name.startswith("obs"):
                self.obstacle_geom_ids.add(g_id)

        # Initialize Perception
        self.perception = LiDARPerceptionSystem(self.model, max_range=2.0, danger_dist=0.75, emergency_dist=0.30)

        # Initialize Controller
        self.controller = DSLPIDControlMuJoCo(mass=0.0472, control_freq=self.control_freq)

        # Initialize Avoidance Strategy
        self._init_avoidance_algorithm()

        # Dynamic obstacle joint IDs (Scenario 7 or Scenario 9)
        self.dynamic_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "obs_dynamic_slide")
        if self.dynamic_joint_id < 0:
            self.dynamic_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "obs_intruder_slide")

        self.reset()

    def _init_avoidance_algorithm(self):
        """Create avoidance strategy instance."""
        if self.algorithm_name == "reactive":
            self.avoidance = ReactiveObstacleAvoidance(self.goal_pos, self.start_pos[2], self.control_freq)
        elif self.algorithm_name == "apf":
            self.avoidance = APFObstacleAvoidance(self.goal_pos, self.start_pos[2])
        elif self.algorithm_name == "dwa":
            self.avoidance = DWAObstacleAvoidance(self.goal_pos, self.start_pos[2])
        elif self.algorithm_name in ["3d", "volumetric"]:
            self.avoidance = Avoidance3D(self.goal_pos, self.start_pos[2])
        elif self.algorithm_name in ["sindy", "sindy_dwa"]:
            self.avoidance = SINDyDWAAvoidance(self.goal_pos, self.start_pos[2], self.control_freq)
        else:
            raise ValueError(f"Unknown avoidance algorithm: {self.algorithm_name}. Supported: reactive, apf, dwa, 3d, sindy_dwa")

    def set_algorithm(self, algorithm_name):
        """Switch active avoidance algorithm at runtime."""
        self.algorithm_name = algorithm_name.lower()
        self._init_avoidance_algorithm()
        self.avoidance.reset()

    def reset(self):
        """Reset simulation state to start configuration."""
        mujoco.mj_resetData(self.model, self.data)

        # Set drone initial position
        qpos_adr = self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")]
        self.data.qpos[qpos_adr:qpos_adr + 3] = self.start_pos
        self.data.qpos[qpos_adr + 3:qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]  # identity quaternion
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0

        mujoco.mj_forward(self.model, self.data)

        self.controller.reset()
        self.avoidance.reset()

        # Metrics
        self.step_count = 0
        self.sim_time = 0.0
        self.path_length = 0.0
        self.collision_steps = 0
        self.min_clearance = float("inf")
        self.previous_pos = self.start_pos.copy()
        self.result = "RUNNING"
        self.history = []

    def get_drone_state(self):
        """Read 6-DOF state from MuJoCo sensor arrays or qpos/qvel."""
        qpos_adr = self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")]
        qvel_adr = self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "root")]

        pos = self.data.qpos[qpos_adr:qpos_adr + 3].copy()
        quat = self.data.qpos[qpos_adr + 3:qpos_adr + 7].copy()
        vel = self.data.qvel[qvel_adr:qvel_adr + 3].copy()
        angvel = self.data.qvel[qvel_adr + 3:qvel_adr + 6].copy()

        return pos, quat, vel, angvel

    def check_collisions(self):
        """Check if any drone geom is currently in physical contact with an obstacle."""
        for c_idx in range(self.data.ncon):
            contact = self.data.contact[c_idx]
            g1, g2 = contact.geom1, contact.geom2
            if (g1 in self.drone_geom_ids and g2 in self.obstacle_geom_ids) or \
               (g2 in self.drone_geom_ids and g1 in self.obstacle_geom_ids):
                return True
        return False

    def update_dynamic_obstacles(self, t):
        """Animate dynamic obstacles if present."""
        if self.dynamic_joint_id >= 0:
            qpos_adr = self.model.jnt_qposadr[self.dynamic_joint_id]
            dof_adr = self.model.jnt_dofadr[self.dynamic_joint_id]
            if self.scenario_id == 9:
                # Scenario 9: Dynamic Lateral Intruder slides between y = -0.6m and y = +0.6m
                omega = 1.0 * self.intruder_speed_multiplier
                self.data.qpos[qpos_adr] = 0.60 * np.sin(omega * t)
                self.data.qvel[dof_adr] = 0.60 * omega * np.cos(omega * t)
            else:
                # Scenario 7: Slide joint
                self.data.qpos[qpos_adr] = 0.75 * np.sin(1.2 * t)
                self.data.qvel[dof_adr] = 0.75 * 1.2 * np.cos(1.2 * t)
            mujoco.mj_forward(self.model, self.data)

    def full_sweep(self, current_pos=None):
        """Perform on-demand 360-degree LiDAR sweep using perception system."""
        if current_pos is None:
            current_pos, _, _, _ = self.get_drone_state()
        return self.perception.full_sweep(self.data, current_pos)

    def step(self):
        """
        Execute one complete control step (including physics substeps).
        """
        if self.result != "RUNNING":
            return self.get_summary()

        current_pos, current_quat, current_vel, current_angvel = self.get_drone_state()

        # Update dynamic moving obstacles
        self.update_dynamic_obstacles(self.sim_time)

        # 1. Perception Step (with dynamic velocity estimation)
        perception_data = self.perception.update(
            self.data, current_pos, current_quat, current_vel=current_vel, dt=1.0 / self.control_freq
        )
        min_dist = perception_data["min_dist"]
        if min_dist < self.min_clearance:
            self.min_clearance = min_dist

        # 2. Collision Detection
        is_collision = self.check_collisions()
        if is_collision:
            self.collision_steps += 1

        # 3. Track Path Length
        step_disp = np.linalg.norm(current_pos - self.previous_pos)
        self.path_length += float(step_disp)
        self.previous_pos = current_pos.copy()

        # 4. Avoidance Strategy Step (with full sweep callable)
        if isinstance(self.avoidance, (ReactiveObstacleAvoidance, SINDyDWAAvoidance)):
            target_pos, desired_vel, avoidance_info = self.avoidance.compute_step(
                current_pos, current_vel, perception_data,
                dt=1.0 / self.control_freq,
                full_sweep_fn=lambda: self.full_sweep(current_pos)
            )
        else:
            target_pos, desired_vel, avoidance_info = self.avoidance.compute_step(
                current_pos, current_vel, perception_data, dt=1.0 / self.control_freq
            )

        # 5. DSL PID Flight Controller Step
        motor_thrusts, ctrl_info = self.controller.compute_control(
            cur_pos=current_pos,
            cur_quat=current_quat,
            cur_vel=current_vel,
            cur_ang_vel=current_angvel,
            target_pos=target_pos,
            target_vel=desired_vel
        )

        # Apply motor thrusts to actuators
        self.data.ctrl[:] = motor_thrusts

        # 6. Step MuJoCo Physics (Sub-stepping)
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        self.sim_time = self.step_count * (1.0 / self.control_freq)

        # Check Termination Criteria
        dist_to_goal = np.linalg.norm(self.goal_pos - current_pos)
        if dist_to_goal < 0.25:
            self.result = "SUCCESS"
        elif current_pos[2] < 0.10:
            self.result = "GROUND CRASH"
        elif self.sim_time > 60.0:
            self.result = "TIME LIMIT REACHED"

        step_info = {
            "step": self.step_count,
            "sim_time": self.sim_time,
            "pos": current_pos,
            "vel": current_vel,
            "dist_to_goal": dist_to_goal,
            "min_dist": min_dist,
            "target_pos": target_pos,
            "desired_vel": desired_vel,
            "mode": avoidance_info.get("mode", "FORWARD"),
            "status_text": avoidance_info.get("status_text", ""),
            "perception": perception_data,
            "is_collision": is_collision,
            "collision_steps": self.collision_steps,
            "path_length": self.path_length,
            "result": self.result,
            "avoidance_info": avoidance_info
        }

        self.history.append({
            "time": self.sim_time,
            "pos": current_pos.copy(),
            "mode": step_info["mode"]
        })

        return step_info

    def get_summary(self):
        """Return final experiment performance dictionary."""
        pos, _, _, _ = self.get_drone_state()
        return {
            "scenario_id": self.scenario_id,
            "sub_scenario": self.sub_scenario if self.scenario_id == 10 else "default",
            "seed": self.seed if self.scenario_id == 10 else 0,
            "algorithm": self.algorithm_name,
            "result": self.result,
            "control_steps": self.step_count,
            "completion_time": round(self.sim_time, 2),
            "final_position": [round(float(x), 3) for x in pos],
            "path_length": round(self.path_length, 3),
            "collision_steps": self.collision_steps,
            "min_clearance": round(self.min_clearance, 3),
            "avoidance_events": getattr(self.avoidance, "avoidance_events", 0),
            "emergency_reversals": getattr(self.avoidance, "emergency_reversals", 0),
            "ring_escape_triggered": getattr(self.avoidance, "ring_escape_triggered", False),
            "escape_method": getattr(self.avoidance, "escape_method", "none"),
            "max_altitude_reached": round(getattr(self.avoidance, "max_altitude_reached", float(self.start_pos[2])), 3),
            "gap_search_count": getattr(self.avoidance, "gap_search_count", 0),
            "dynamic_avoidance_events": getattr(self.avoidance, "dynamic_avoidance_events", 0)
        }
