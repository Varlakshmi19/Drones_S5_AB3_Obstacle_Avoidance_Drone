"""
LiDAR Sensor Array and Perception System for MuJoCo Drone.
Provides multi-directional rangefinder reading, 3D raycasting, danger assessment, and point-cloud extraction.
"""

import numpy as np
import mujoco


class LiDARPerceptionSystem:
    """
    Perception system leveraging native MuJoCo rangefinder sensors, dynamic velocity estimation,
    and on-demand 360-degree full-sweep raycasting.
    """

    def __init__(self, model, max_range=2.0, danger_dist=0.75, emergency_dist=0.30):
        self.model = model
        self.max_range = max_range
        self.danger_dist = danger_dist
        self.emergency_dist = emergency_dist
        self.drone_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

        # Map sensor names to their indices in data.sensordata
        self.sensor_ids = {}
        for i in range(model.nsensor):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
            if name:
                self.sensor_ids[name] = i

        # Standard horizontal sensor angles relative to forward (+X)
        self.sensor_angles_deg = {
            "front": 0.0,
            "left": 35.0,
            "far_left": 70.0,
            "side_left": 90.0,
            "right": -35.0,
            "far_right": -70.0,
            "side_right": -90.0,
            "rear": 180.0,
        }

        # Dynamic tracking state per sector
        self.prev_distances = {}
        self.closing_velocity_history = {k: [] for k in self.sensor_angles_deg.keys()}
        self.dynamic_speed_threshold = 0.05  # m/s

    def update(self, data, current_pos, current_quat, current_vel=None, dt=1.0 / 48.0):
        """
        Extract sensor readings, compute sector distances, and estimate closing velocities.
        """
        if current_vel is None:
            current_vel = np.zeros(3)
        distances = {}
        rays = []
        hit_points = []
        dynamic_sectors = {}

        # Drone orientation matrix (3x3)
        drone_rot = np.zeros(9)
        mujoco.mju_quat2Mat(drone_rot, current_quat)
        rot_matrix = drone_rot.reshape(3, 3)

        # 1. Cast Ray for each sensor with drone subtree exclusion
        geom_id_arr = np.zeros(1, dtype=np.int32)
        for sensor_name, angle_deg in self.sensor_angles_deg.items():
            angle_rad = np.radians(angle_deg)
            local_dir = np.array([np.cos(angle_rad), np.sin(angle_rad), 0.0])
            world_dir = rot_matrix @ local_dir
            world_dir /= np.linalg.norm(world_dir)

            start_pt = current_pos + world_dir * 0.22
            ray_dist = mujoco.mj_ray(
                self.model, data, start_pt, world_dir,
                None, 1, self.drone_body_id, geom_id_arr
            )

            if ray_dist < 0.0 or ray_dist > (self.max_range - 0.22):
                dist = self.max_range
                hit = False
            else:
                dist = float(ray_dist + 0.22)
                hit = True

            distances[sensor_name] = dist
            end_pt = start_pt + world_dir * ray_dist

            # Color coding
            if dist < self.emergency_dist:
                color = [1.0, 0.1, 0.1, 0.9]  # Red
            elif dist < self.danger_dist:
                color = [1.0, 0.7, 0.1, 0.9]  # Orange
            else:
                color = [0.1, 0.9, 0.2, 0.6]  # Green

            rays.append({
                "name": sensor_name,
                "start": start_pt,
                "end": end_pt,
                "distance": dist,
                "color": color,
                "hit": hit
            })

            if hit:
                hit_points.append(end_pt)
                if geom_id_arr[0] >= 0:
                    body_id = self.model.geom_bodyid[geom_id_arr[0]]
                    body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
                    if "dynamic" in body_name or "intruder" in body_name:
                        obs_vel = data.cvel[body_id][3:6]
                        v_rel = obs_vel - current_vel
                        v_closing = float(-np.dot(v_rel[:2], world_dir[:2]))
                        if v_closing > self.dynamic_speed_threshold and sensor_name in [
                            "front", "left", "right", "far_left", "far_right", "side_left", "side_right"
                        ]:
                            dynamic_sectors[sensor_name] = {
                                "v_closing": float(v_closing),
                                "is_closing": True
                            }

        # Read Vertical sensors (Up, Down)
        for v_name, v_dir in [("up", np.array([0.0, 0.0, 1.0])), ("down", np.array([0.0, 0.0, -1.0]))]:
            start_v = current_pos + v_dir * 0.10
            v_dist = mujoco.mj_ray(
                self.model, data, start_v, v_dir,
                None, 1, self.drone_body_id, geom_id_arr
            )
            dist_v = self.max_range if (v_dist < 0.0 or v_dist > (self.max_range - 0.10)) else float(v_dist + 0.10)
            distances[v_name] = dist_v

        self.prev_distances = distances.copy()
        min_dist = min([d for d in distances.values()])

        if min_dist < self.emergency_dist:
            danger_level = "EMERGENCY"
        elif min_dist < self.danger_dist:
            danger_level = "WARNING"
        elif min_dist < self.max_range * 0.7:
            danger_level = "CAUTION"
        else:
            danger_level = "SAFE"

        return {
            "distances": distances,
            "min_dist": min_dist,
            "danger_level": danger_level,
            "rays": rays,
            "hit_points": hit_points,
            "dynamic_sectors": dynamic_sectors,
            "is_dynamic": len(dynamic_sectors) > 0
        }

    def full_sweep(self, data, current_pos, num_rays=72, max_range=8.0):
        """
        Cast N rays (e.g. N=72 every 5 deg) around full 360 deg at current altitude,
        plus one additional straight-up ray to measure climb clearance.
        """
        angles = np.linspace(0, 360, num_rays, endpoint=False)
        geomid = np.zeros(1, dtype=np.int32)
        horizontal_scan = []

        for ang_deg in angles:
            ang_rad = np.radians(ang_deg)
            vec = np.array([np.cos(ang_rad), np.sin(ang_rad), 0.0], dtype=np.float64)
            pnt = np.array(current_pos, dtype=np.float64) + vec * 0.22

            dist = mujoco.mj_ray(
                self.model, data, pnt, vec, None, 1, self.drone_body_id, geomid
            )
            g_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geomid[0]) if geomid[0] >= 0 else ""
            is_obs = g_name.startswith("obs")

            if not is_obs or dist < 0 or dist >= (max_range - 0.22):
                d = max_range
                is_clear = True
            else:
                d = dist + 0.22
                is_clear = False

            horizontal_scan.append((float(ang_deg), float(d), is_clear))

        # Straight-up ray (+Z)
        vec_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        pnt_up = np.array(current_pos, dtype=np.float64) + vec_up * 0.10
        dist_up = mujoco.mj_ray(
            self.model, data, pnt_up, vec_up, None, 1, self.drone_body_id, geomid
        )
        g_name_up = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geomid[0]) if geomid[0] >= 0 else ""
        is_obs_up = g_name_up.startswith("obs")
        vertical_clearance = float(max_range if (dist_up < 0 or not is_obs_up) else dist_up + 0.10)

        num_clear = sum(1 for item in horizontal_scan if item[2])
        all_clear = (num_clear == num_rays)
        min_d = min(item[1] for item in horizontal_scan)

        return {
            "horizontal_scan": horizontal_scan,
            "vertical_clearance": vertical_clearance,
            "all_clear": all_clear,
            "num_clear_rays": num_clear,
            "min_dist": min_d
        }
