"""
Scene Generator for Drone Obstacle Detection and Avoidance Simulation in MuJoCo.
Supports 8 benchmark scenarios ranging from single obstacles to dynamic moving obstacles and 3D cluttered spaces.
"""

import os
import xml.etree.ElementTree as ET
import numpy as np


def get_drone_xml_path():
    """Return absolute path to drone.xml"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(current_dir, "drone.xml")


def generate_scene_xml(
    scenario_id=1,
    start_pos=(0.0, 0.0, 1.0),
    goal_pos=(6.0, 0.0, 1.0),
    custom_obstacles=None,
    sub_scenario="closed_ring",
    seed=42
):
    """
    Generate complete MJCF XML string for a specific obstacle scenario.

    Scenarios:
    1 - Single frontal obstacle
    2 - Obstacle offset left
    3 - Obstacle offset right
    4 - Narrow passage / corridor
    5 - Dead-end / U-Trap
    6 - Multi-obstacle staggered forest
    7 - Dynamic moving obstacle (sliding lateral block)
    8 - 3D Cluttered environment (different heights requiring 3D bypass/overflight)
    9 - Dynamic Lateral Intruder (0.30m cube sliding laterally between y = -0.6m and y = +0.6m)
    10 - Circular Dead-End Ring ("closed_ring" or "ring_with_gap")
    """

    z = start_pos[2]
    obstacles = []

    if custom_obstacles is not None:
        obstacles = custom_obstacles
    elif scenario_id == 1:
        # 1: Direct frontal obstacle
        obstacles.append({
            "name": "obs1",
            "pos": [2.5, 0.0, z],
            "size": [0.15, 0.15, 0.50],
            "type": "box",
            "rgba": [0.85, 0.25, 0.25, 1.0]
        })
    elif scenario_id == 2:
        # 2: Obstacle slightly toward the left
        obstacles.append({
            "name": "obs1",
            "pos": [2.5, 0.20, z],
            "size": [0.15, 0.15, 0.50],
            "type": "box",
            "rgba": [0.85, 0.25, 0.25, 1.0]
        })
    elif scenario_id == 3:
        # 3: Obstacle slightly toward the right
        obstacles.append({
            "name": "obs1",
            "pos": [2.5, -0.20, z],
            "size": [0.15, 0.15, 0.50],
            "type": "box",
            "rgba": [0.85, 0.25, 0.25, 1.0]
        })
    elif scenario_id == 4:
        # 4: Narrow passage (two pairs of obstacles forming a narrow slit)
        obstacles.extend([
            {"name": "obs1_l", "pos": [2.5, 0.38, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.3, 0.2, 1.0]},
            {"name": "obs1_r", "pos": [2.5, -0.38, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.3, 0.2, 1.0]},
            {"name": "obs2_l", "pos": [3.2, 0.42, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.3, 0.2, 1.0]},
            {"name": "obs2_r", "pos": [3.2, -0.42, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.3, 0.2, 1.0]}
        ])
    elif scenario_id == 5:
        # 5: Dead-end U-trap (matching PyBullet test scenario 5)
        obstacles.extend([
            {"name": "obs1", "pos": [2.5, 0.0, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.15, 0.15, 1.0]},
            {"name": "obs2_l", "pos": [2.35, 0.36, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.25, 0.25, 1.0]},
            {"name": "obs2_r", "pos": [2.35, -0.36, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.25, 0.25, 1.0]},
            {"name": "obs3_l", "pos": [2.0, 0.52, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.25, 0.25, 1.0]},
            {"name": "obs3_r", "pos": [2.0, -0.52, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.25, 0.25, 1.0]}
        ])
    elif scenario_id == 6:
        # 6: Multiple staggered obstacles (Forest maze)
        obstacles.extend([
            {"name": "obs1", "pos": [1.8, 0.0, z], "size": [0.15, 0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.3, 0.2, 1.0]},
            {"name": "obs2", "pos": [3.0, 0.48, z], "size": [0.15, 0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.3, 0.2, 1.0]},
            {"name": "obs3", "pos": [4.0, -0.42, z], "size": [0.15, 0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.3, 0.2, 1.0]},
            {"name": "obs4", "pos": [4.8, 0.10, z], "size": [0.15, 0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.3, 0.2, 1.0]}
        ])
    elif scenario_id == 7:
        # 7: Dynamic moving obstacle (sliding back and forth laterally)
        obstacles.append({
            "name": "obs_dynamic",
            "pos": [2.8, 0.0, z],
            "size": [0.18, 0.18, 0.50],
            "type": "box",
            "rgba": [0.95, 0.55, 0.1, 1.0],
            "is_dynamic": True
        })
    elif scenario_id == 8:
        # 8: 3D Cluttered space with low walls and high overhangs
        obstacles.extend([
            {"name": "obs1_low", "pos": [2.2, 0.0, 0.6], "size": [0.20, 0.40, 0.60], "type": "box", "rgba": [0.3, 0.7, 0.3, 1.0]},
            {"name": "obs2_high", "pos": [3.5, 0.0, 1.6], "size": [0.20, 0.60, 0.35], "type": "box", "rgba": [0.3, 0.5, 0.8, 1.0]},
            {"name": "obs3_pillar", "pos": [4.8, -0.25, z], "size": [0.15, 0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.2, 0.5, 1.0]}
        ])
    elif scenario_id == 9:
        # 9: Dynamic Lateral Intruder (0.30m cube sliding laterally between y = -0.6m and y = +0.6m)
        obstacles.append({
            "name": "obs_intruder",
            "pos": [2.8, 0.0, z],
            "size": [0.15, 0.15, 0.15],
            "type": "box",
            "rgba": [0.95, 0.35, 0.1, 1.0],
            "is_dynamic": True
        })
    elif scenario_id == 10:
        # 10: Circular Dead-End Ring around start_pos
        # 54 cubes with tangential orientation at radius 2.6m provide continuous overlapping wall coverage
        radius = 2.6
        num_cubes = 54
        cube_half_size = [0.16, 0.20, 0.15]  # 0.30m height (z=0.85m to 1.15m at z=1.0m)
        
        # In ring_with_gap, omit cubes within 36 deg arc (~1.63m width) at seeded random angle
        gap_center_deg = None
        if sub_scenario == "ring_with_gap":
            rng = np.random.RandomState(seed)
            gap_center_deg = float(rng.uniform(0.0, 360.0))
            print(f"[RING] Scenario 10 ring_with_gap initialized with seed={seed}, gap centered at {gap_center_deg:.1f}°")
        else:
            print(f"[RING] Scenario 10 closed_ring initialized (full solid ring, 54 cubes, radius={radius}m, height=0.85-1.15m)")

        for i in range(num_cubes):
            deg = i * (360.0 / num_cubes)
            if gap_center_deg is not None:
                # Calculate angular distance to gap center
                ang_diff = abs((deg - gap_center_deg + 180.0) % 360.0 - 180.0)
                if ang_diff < 18.0:  # ~36 deg gap window
                    continue

            rad = np.radians(deg)
            cx = start_pos[0] + radius * np.cos(rad)
            cy = start_pos[1] + radius * np.sin(rad)
            obstacles.append({
                "name": f"obs_ring_{i}",
                "pos": [round(cx, 3), round(cy, 3), z],
                "size": cube_half_size,
                "type": "box",
                "rgba": [0.85, 0.20, 0.20, 1.0],
                "euler": f"0 0 {deg:.1f}"
            })

    # Parse Drone XML
    drone_xml_file = get_drone_xml_path()
    tree = ET.parse(drone_xml_file)
    root_drone = tree.getroot()

    # Extract assets from drone.xml
    drone_assets_str = ""
    asset_elem = root_drone.find("asset")
    if asset_elem is not None:
        for child in asset_elem:
            drone_assets_str += f"    {ET.tostring(child, encoding='unicode')}\n"

    # Base Scene XML
    scene_xml = f"""<mujoco model="drone_obstacle_avoidance_scenario_{scenario_id}">
  <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4">
    <flag contact="enable" energy="enable"/>
  </option>

  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.8 0.8 0.8" specular="0.1 0.1 0.1"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global fovy="55"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0.1 0.15 0.25" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.18 0.22 0.28" rgb2="0.25 0.30 0.38"/>
    <material name="grid_mat" texture="grid" texrepeat="20 20" reflectance="0.2"/>
    <material name="target_mat" rgba="0.1 0.8 0.3 0.75" specular="0.9" shininess="0.9"/>
    <material name="wall_mat" rgba="0.4 0.45 0.5 0.3"/>
{drone_assets_str}  </asset>

  <worldbody>
    <!-- Ground Floor Grid -->
    <geom name="ground" type="plane" size="15 15 0.1" pos="3.0 0 0" material="grid_mat"/>

    <!-- Arena Boundary Markers (Visual low curbs) -->
    <geom name="wall_north" type="box" size="7.0 0.05 0.1" pos="3.0 4.0 0.1" material="wall_mat"/>
    <geom name="wall_south" type="box" size="7.0 0.05 0.1" pos="3.0 -4.0 0.1" material="wall_mat"/>
    <geom name="wall_east" type="box" size="0.05 4.0 0.1" pos="9.0 0.0 0.1" material="wall_mat"/>
    <geom name="wall_west" type="box" size="0.05 4.0 0.1" pos="-3.0 0.0 0.1" material="wall_mat"/>

    <!-- Goal Target Beacon (Sites are visual-only, no collision or lidar reflection) -->
    <body name="goal_beacon" pos="{goal_pos[0]} {goal_pos[1]} {goal_pos[2]}">
      <site name="goal_sphere" type="sphere" size="0.12" material="target_mat"/>
      <site name="goal_ring" type="cylinder" size="0.22 0.01" rgba="0.1 0.9 0.4 0.5"/>
      <site name="goal_post" type="cylinder" size="0.015 {goal_pos[2]/2.0}" pos="0 0 {-goal_pos[2]/2.0}" rgba="0.2 0.9 0.4 0.3"/>
      <site name="goal_site" pos="0 0 0" size="0.02" rgba="0 1 0 1"/>
    </body>

    <!-- Obstacles for Current Scenario -->
"""

    for i, obs in enumerate(obstacles):
        pos = obs.get("pos", [3.0, 0.0, 1.0])
        pos_str = f"{pos[0]} {pos[1]} {pos[2]}"
        
        obs_type = obs.get("type", "box")
        is_dynamic = obs.get("is_dynamic", False)
        
        name = obs.get("name")
        if not name:
            name = f"obs_dynamic_{i+1}" if is_dynamic else f"obs_{obs_type}_{i+1}"
        elif is_dynamic and "dynamic" not in name and "intruder" not in name and "slide" not in name:
            name = f"{name}_dynamic"

        # Safe RGBA retrieval with vivid defaults
        rgba = obs.get("rgba")
        if not rgba or len(rgba) < 4:
            if is_dynamic:
                rgba = [0.95, 0.45, 0.1, 1.0]
            elif obs_type == "cylinder":
                rgba = [0.75, 0.25, 0.85, 1.0]
            else:
                rgba = [0.85, 0.25, 0.25, 1.0]
        rgba_str = f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"
        
        size = obs.get("size", [0.18, 0.18, 0.50])
        if obs_type == 'box':
            size_str = f"{size[0]} {size[1]} {size[2]}"
            if is_dynamic:
                # Dynamic slide joint along Y axis
                scene_xml += f"""    <body name="{name}" pos="{pos_str}">
      <joint name="{name}_slide" type="slide" axis="0 1 0" range="-1.2 1.2" damping="1.0"/>
      <geom name="{name}_geom" type="box" size="{size_str}" rgba="{rgba_str}" density="500"/>
    </body>\n"""
            else:
                euler_attr = f' euler="{obs["euler"]}"' if "euler" in obs else ""
                scene_xml += f"""    <body name="{name}" pos="{pos_str}">
      <geom name="{name}_geom" type="box" size="{size_str}" rgba="{rgba_str}"{euler_attr}/>
    </body>\n"""
        elif obs_type == 'cylinder':
            # MuJoCo cylinder size: radius, half-height
            if len(size) >= 3:
                size_str = f"{size[0]} {size[2]}"
            else:
                size_str = f"{size[0]} {size[1]}"
            if is_dynamic:
                scene_xml += f"""    <body name="{name}" pos="{pos_str}">
      <joint name="{name}_slide" type="slide" axis="0 1 0" range="-1.2 1.2" damping="1.0"/>
      <geom name="{name}_geom" type="cylinder" size="{size_str}" rgba="{rgba_str}" density="500"/>
    </body>\n"""
            else:
                scene_xml += f"""    <body name="{name}" pos="{pos_str}">
      <geom name="{name}_geom" type="cylinder" size="{size_str}" rgba="{rgba_str}"/>
    </body>\n"""

    # Add Drone body
    drone_body = root_drone.find(".//body[@name='drone']")
    # Update initial position in drone body
    drone_body.set("pos", f"{start_pos[0]} {start_pos[1]} {start_pos[2]}")
    drone_body_xml = ET.tostring(drone_body, encoding="unicode")
    scene_xml += f"    {drone_body_xml}\n  </worldbody>\n\n"

    # Add Drone Actuators & Sensors
    actuator_elem = root_drone.find(".//actuator")
    if actuator_elem is not None:
        scene_xml += f"  {ET.tostring(actuator_elem, encoding='unicode')}\n"

    sensor_elem = root_drone.find(".//sensor")
    if sensor_elem is not None:
        scene_xml += f"  {ET.tostring(sensor_elem, encoding='unicode')}\n"

    scene_xml += "</mujoco>"
    return scene_xml
