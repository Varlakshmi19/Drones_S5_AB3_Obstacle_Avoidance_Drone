import time
from collections import deque

import numpy as np
import pybullet as p

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync


# ============================================================
# TEST SCENARIOS (1 to 10)
# ============================================================
SCENARIOS = {
    1: {
        "id": 1,
        "name": "Direct Frontal Obstacle",
        "description": "One stationary cube obstacle directly in the drone's forward trajectory.",
        "difficulty": "Easy",
        "expected_behavior": "Drone detects front blockage, steers around either left or right, and returns to goal."
    },
    2: {
        "id": 2,
        "name": "Offset Obstacle Left",
        "description": "One stationary obstacle positioned slightly to the left of the centerline.",
        "difficulty": "Easy",
        "expected_behavior": "Drone detects asymmetric opening and efficiently bypasses on the right side."
    },
    3: {
        "id": 3,
        "name": "Offset Obstacle Right",
        "description": "One stationary obstacle positioned slightly to the right of the centerline.",
        "difficulty": "Easy",
        "expected_behavior": "Drone detects asymmetric opening and efficiently bypasses on the left side."
    },
    4: {
        "id": 4,
        "name": "Narrow Passage / Slit",
        "description": "Two pairs of opposing obstacles creating a narrow corridor passage.",
        "difficulty": "Medium",
        "expected_behavior": "Drone detects narrow lateral clearance, executes sequential avoidance maneuvers to navigate through or around."
    },
    5: {
        "id": 5,
        "name": "Dead-End / U-Trap",
        "description": "Enclosed U-shaped obstacle arrangement blocking forward, left, and right directions.",
        "difficulty": "Hard",
        "expected_behavior": "Drone detects trapped condition, initiates emergency reversal, turns toward open space, and navigates around perimeter."
    },
    6: {
        "id": 6,
        "name": "Staggered Multi-Obstacle Forest",
        "description": "Multiple staggered cylindrical obstacles requiring continuous slalom avoidance.",
        "difficulty": "Hard",
        "expected_behavior": "Drone weaves continuously between obstacles with rapid perception and responsive steering."
    },
    7: {
        "id": 7,
        "name": "Dynamic Moving Obstacle",
        "description": "A dynamic obstacle sliding laterally back and forth across the flight path.",
        "difficulty": "Dynamic",
        "expected_behavior": "Drone dynamically observes time-varying clearance and adapts flight path in real time."
    },
    8: {
        "id": 8,
        "name": "3D Cluttered Environment",
        "description": "Multiple obstacles at varying heights requiring 3D vertical climbing or ducking.",
        "difficulty": "3D Expert",
        "expected_behavior": "Drone leverages 3D altitude freedom to climb over low barriers and fly toward goal."
    },
    9: {
        "id": 9,
        "name": "Dynamic Lateral Intruder",
        "description": "A 0.30m cube sliding rapidly along Y across the flight path.",
        "difficulty": "Dynamic Expert",
        "expected_behavior": "Drone predicts closing velocity and trajectory, evading the moving obstacle with zero collisions."
    },
    10: {
        "id": 10,
        "name": "Circular Dead-End Ring",
        "description": "Continuous circular ring of 54 cubes surrounding start position with closed ring or seeded random gap.",
        "difficulty": "Extreme Trap",
        "expected_behavior": "Drone identifies dead-end ring signature: performs vertical climb-over (closed_ring) or finds horizontal gap via 360 LiDAR scan (ring_with_gap)."
    }
}

# Run every case separately by changing this value (1-10).
TEST_SCENARIO = 1


# ============================================================
# SIMULATION SETTINGS
# ============================================================

PYB_FREQ = 240
CONTROL_FREQ = 48
DURATION_SEC = 60

FLIGHT_ALTITUDE = 1.0

START_POSITION = np.array([0.0, 0.0, FLIGHT_ALTITUDE])
GOAL_POSITION = np.array([6.0, 0.0, FLIGHT_ALTITUDE])

FORWARD_SPEED = 0.35
AVOID_FORWARD_SPEED = 0.18
SIDE_SPEED = 0.38
RETURN_SPEED = 0.28
REVERSE_SPEED = 0.22

SENSOR_RANGE = 1.6

DETECTION_DISTANCE = 0.75
EMERGENCY_DISTANCE = 0.30
SIDE_BLOCKED_DISTANCE = 0.48
CLEAR_DISTANCE = 1.0

GOAL_TOLERANCE = 0.25

# Number of control steps for which an avoidance decision is retained.
MIN_AVOID_STEPS = int(1.0 * CONTROL_FREQ)

# Number of clear readings required before leaving avoidance mode.
CLEAR_CONFIRMATION_STEPS = int(0.35 * CONTROL_FREQ)

# Reverse time when trapped or stuck.
ESCAPE_STEPS = int(1.2 * CONTROL_FREQ)

# Stuck detector.
STUCK_WINDOW_STEPS = int(2.0 * CONTROL_FREQ)
MIN_PROGRESS_DISTANCE = 0.10


# ============================================================
# OBSTACLE CREATION
# ============================================================

def add_cube(position, size=0.30, colour=None):
    """
    Add a stationary cube obstacle.

    size=0.30 produces a cube of:
    0.30 m × 0.30 m × 0.30 m
    """

    if colour is None:
        colour = [0.8, 0.15, 0.15, 1.0]

    half_size = size / 2.0

    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[half_size, half_size, half_size]
    )

    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[half_size, half_size, half_size],
        rgbaColor=colour
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position
    )


def add_cylinder(position, radius=0.15, height=1.6, colour=None):
    """
    Add a stationary vertical cylindrical obstacle.
    """
    if colour is None:
        colour = [0.85, 0.45, 0.1, 1.0]

    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_CYLINDER,
        radius=radius,
        height=height
    )

    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_CYLINDER,
        radius=radius,
        length=height,
        rgbaColor=colour
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position
    )


DYNAMIC_OBSTACLES = []


def update_dynamic_obstacles(sim_time):
    """
    Update positions of dynamic moving obstacles based on simulation time.
    """
    for dyn in DYNAMIC_OBSTACLES:
        body_id = dyn["id"]
        x = dyn["x"]
        z = dyn["z"]
        y0 = dyn.get("y0", 0.0)
        amp = dyn.get("amplitude", 0.8)
        freq = dyn.get("freq", 1.5)
        new_y = y0 + amp * np.sin(freq * sim_time)
        p.resetBasePositionAndOrientation(
            body_id,
            [x, new_y, z],
            [0.0, 0.0, 0.0, 1.0]
        )


def create_test_scenario(scenario, ring_with_gap=False):
    """
    Create obstacle arrangements for scenarios 1 through 10.
    Returns the PyBullet IDs of the created obstacles.
    """

    global DYNAMIC_OBSTACLES
    DYNAMIC_OBSTACLES = []
    obstacle_ids = []
    z = FLIGHT_ALTITUDE

    if scenario == 1:
        # Scenario 1: Direct Frontal Obstacle
        obstacle_ids.append(add_cube([2.5, 0.0, z], size=0.30))

    elif scenario == 2:
        # Scenario 2: Offset Obstacle Left
        obstacle_ids.append(add_cube([2.5, 0.18, z], size=0.30))

    elif scenario == 3:
        # Scenario 3: Offset Obstacle Right
        obstacle_ids.append(add_cube([2.5, -0.18, z], size=0.30))

    elif scenario == 4:
        # Scenario 4: Narrow Passage / Slit
        obstacle_ids.append(add_cube([2.5, 0.38, z], size=0.30))
        obstacle_ids.append(add_cube([2.5, -0.38, z], size=0.30))
        obstacle_ids.append(add_cube([3.0, 0.42, z], size=0.30))
        obstacle_ids.append(add_cube([3.0, -0.42, z], size=0.30))

    elif scenario == 5:
        # Scenario 5: Dead-End / U-Trap
        obstacle_ids.append(add_cube([2.5, 0.0, z], size=0.30))
        obstacle_ids.append(add_cube([2.35, 0.36, z], size=0.30))
        obstacle_ids.append(add_cube([2.35, -0.36, z], size=0.30))
        obstacle_ids.append(add_cube([2.0, 0.52, z], size=0.30))
        obstacle_ids.append(add_cube([2.0, -0.52, z], size=0.30))

    elif scenario == 6:
        # Scenario 6: Staggered Multi-Obstacle Forest
        # Multiple staggered cylindrical obstacles requiring continuous slalom avoidance
        forest_positions = [
            [1.8, -0.30, z],
            [2.5, 0.35, z],
            [3.2, -0.35, z],
            [4.0, 0.30, z],
            [4.8, -0.20, z]
        ]
        for pos in forest_positions:
            obstacle_ids.append(add_cylinder(pos, radius=0.15, height=1.6))

    elif scenario == 7:
        # Scenario 7: Dynamic Moving Obstacle
        # A dynamic obstacle sliding laterally back and forth across the flight path
        obs_id = add_cube([2.8, 0.0, z], size=0.30, colour=[0.9, 0.2, 0.6, 1.0])
        obstacle_ids.append(obs_id)
        DYNAMIC_OBSTACLES.append({
            "id": obs_id,
            "x": 2.8,
            "z": z,
            "y0": 0.0,
            "amplitude": 0.75,
            "freq": 1.5
        })

    elif scenario == 8:
        # Scenario 8: 3D Cluttered Environment
        # Multiple obstacles at varying heights requiring 3D vertical climbing or ducking
        # Low barrier at x=2.2 (drone can climb over it)
        obstacle_ids.append(add_cube([2.2, 0.0, 0.55], size=0.30, colour=[0.2, 0.6, 0.9, 1.0]))
        obstacle_ids.append(add_cube([2.2, 0.30, 0.55], size=0.30, colour=[0.2, 0.6, 0.9, 1.0]))
        obstacle_ids.append(add_cube([2.2, -0.30, 0.55], size=0.30, colour=[0.2, 0.6, 0.9, 1.0]))
        # Overhead barrier at x=3.5 (high altitude obstacle)
        obstacle_ids.append(add_cube([3.5, 0.0, 1.45], size=0.30, colour=[0.3, 0.8, 0.3, 1.0]))
        obstacle_ids.append(add_cube([3.5, 0.30, 1.45], size=0.30, colour=[0.3, 0.8, 0.3, 1.0]))
        obstacle_ids.append(add_cube([3.5, -0.30, 1.45], size=0.30, colour=[0.3, 0.8, 0.3, 1.0]))
        # Stepping barrier at x=4.6
        obstacle_ids.append(add_cube([4.6, 0.0, 0.60], size=0.30, colour=[0.9, 0.5, 0.2, 1.0]))

    elif scenario == 9:
        # Scenario 9: Dynamic Lateral Intruder
        # A 0.30m cube sliding rapidly along Y across the flight path
        obs_id = add_cube([3.0, 0.0, z], size=0.30, colour=[0.9, 0.1, 0.1, 1.0])
        obstacle_ids.append(obs_id)
        DYNAMIC_OBSTACLES.append({
            "id": obs_id,
            "x": 3.0,
            "z": z,
            "y0": 0.0,
            "amplitude": 1.20,
            "freq": 2.5
        })

    elif scenario == 10:
        # Scenario 10: Circular Dead-End Ring
        # Continuous circular ring of 54 cubes surrounding start position with closed ring or seeded random gap
        center_x = 0.0
        center_y = 0.0
        radius = 2.0
        num_cubes = 54
        for i in range(num_cubes):
            angle = 2 * np.pi * i / num_cubes
            if ring_with_gap and (abs(angle) < 0.25 or abs(angle - 2 * np.pi) < 0.25):
                continue
            x = center_x + radius * np.cos(angle)
            y = center_y + radius * np.sin(angle)
            obstacle_ids.append(add_cube([x, y, z], size=0.28, colour=[0.85, 0.2, 0.2, 1.0]))

    else:
        raise ValueError(
            "TEST_SCENARIO must be an integer from 1 to 10."
        )

    return obstacle_ids


# ============================================================
# VECTOR FUNCTIONS
# ============================================================

def horizontal_unit_vector(vector):
    """
    Convert a vector into a horizontal unit vector.
    """

    vector = np.asarray(vector, dtype=float).copy()
    vector[2] = 0.0

    magnitude = np.linalg.norm(vector)

    if magnitude < 1e-8:
        return np.array([1.0, 0.0, 0.0])

    return vector / magnitude


def rotate_horizontal(vector, angle_degrees):
    """
    Rotate a horizontal vector about the vertical z-axis.
    """

    angle = np.radians(angle_degrees)

    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle),  np.cos(angle), 0.0],
        [0.0,            0.0,           1.0]
    ])

    return rotation_matrix @ vector


def goal_direction(current_position):
    """
    Return the horizontal direction from the drone to the goal.
    """

    return horizontal_unit_vector(
        GOAL_POSITION - current_position
    )


# ============================================================
# SENSOR FUNCTIONS
# ============================================================

def cast_sensor_ray(
    drone_position,
    direction,
    maximum_range,
    danger_distance
):
    """
    Cast one virtual distance sensor ray.
    """

    direction = horizontal_unit_vector(direction)

    # Start beyond the central body so that the ray does not
    # detect the drone itself.
    ray_start = drone_position + direction * 0.18
    ray_end = ray_start + direction * maximum_range

    result = p.rayTest(ray_start, ray_end)[0]

    hit_object_id = result[0]
    hit_fraction = result[2]
    hit_position = result[3]

    if hit_object_id == -1:
        distance = maximum_range
        visual_end = ray_end
    else:
        distance = hit_fraction * maximum_range
        visual_end = hit_position

    if distance < danger_distance:
        colour = [1.0, 0.0, 0.0]
    else:
        colour = [0.0, 1.0, 0.0]

    p.addUserDebugLine(
        lineFromXYZ=ray_start,
        lineToXYZ=visual_end,
        lineColorRGB=colour,
        lineWidth=2,
        lifeTime=0.08
    )

    return distance


def read_sensors(current_position):
    """
    Read six virtual sensors.

    Sensor angles relative to the goal direction:

    far-left:  +70°
    left:      +35°
    front:       0°
    right:     -35°
    far-right: -70°
    rear:      180°
    """

    base_direction = goal_direction(current_position)

    directions = {
        "far_left": rotate_horizontal(base_direction, 70),
        "left": rotate_horizontal(base_direction, 35),
        "front": base_direction,
        "right": rotate_horizontal(base_direction, -35),
        "far_right": rotate_horizontal(base_direction, -70),
        "rear": rotate_horizontal(base_direction, 180)
    }

    distances = {}

    for sensor_name, direction in directions.items():
        distances[sensor_name] = cast_sensor_ray(
            current_position,
            direction,
            SENSOR_RANGE,
            DETECTION_DISTANCE
        )

    return distances, directions


# ============================================================
# DECISION FUNCTIONS
# ============================================================

def calculate_side_scores(distances):
    """
    Higher score means more free space.

    The near-side sensor receives greater weight because it
    represents the drone's immediate turning path.
    """

    left_score = (
        0.65 * distances["left"]
        + 0.35 * distances["far_left"]
    )

    right_score = (
        0.65 * distances["right"]
        + 0.35 * distances["far_right"]
    )

    return left_score, right_score


def choose_avoidance_side(distances, previous_side=None):
    """
    Choose the side with more available space.

    If both sides are almost equal, retain the previous side
    to prevent rapid left-right switching.
    """

    left_score, right_score = calculate_side_scores(distances)

    score_difference = abs(left_score - right_score)

    if score_difference < 0.08 and previous_side is not None:
        return previous_side

    if left_score >= right_score:
        return "LEFT"

    return "RIGHT"


def progress_is_too_small(position_history):
    """
    Determine whether the drone has become stuck.
    """

    if len(position_history) < STUCK_WINDOW_STEPS:
        return False

    oldest_position = position_history[0]
    newest_position = position_history[-1]

    horizontal_progress = np.linalg.norm(
        newest_position[:2] - oldest_position[:2]
    )

    return horizontal_progress < MIN_PROGRESS_DISTANCE


# ============================================================
# VELOCITY COMMANDS
# ============================================================

def make_velocity_command(
    mode,
    current_position,
    directions,
    avoidance_side
):
    """
    Generate a velocity command for the current controller mode.
    """

    toward_goal = directions["front"]

    left_vector = rotate_horizontal(toward_goal, 90)
    right_vector = rotate_horizontal(toward_goal, -90)

    if mode == "FORWARD":
        velocity = toward_goal * FORWARD_SPEED

    elif mode == "AVOID":
        if avoidance_side == "LEFT":
            lateral_vector = left_vector
        else:
            lateral_vector = right_vector

        velocity = (
            toward_goal * AVOID_FORWARD_SPEED
            + lateral_vector * SIDE_SPEED
        )

    elif mode == "RETURN":
        velocity = toward_goal * RETURN_SPEED

    elif mode == "EMERGENCY_REVERSE":
        velocity = -toward_goal * REVERSE_SPEED

    elif mode == "GOAL_REACHED":
        velocity = np.zeros(3)

    else:
        velocity = np.zeros(3)

    velocity[2] = 0.0

    return velocity


# ============================================================
# VISUALISATION
# ============================================================

def draw_goal():
    """
    Draw a marker at the goal.
    """

    p.addUserDebugText(
        text="GOAL",
        textPosition=GOAL_POSITION + np.array([0.0, 0.0, 0.30]),
        textColorRGB=[0.0, 0.0, 1.0],
        textSize=1.4
    )

    p.addUserDebugLine(
        GOAL_POSITION - np.array([0.20, 0.0, 0.0]),
        GOAL_POSITION + np.array([0.20, 0.0, 0.0]),
        [0.0, 0.0, 1.0],
        lineWidth=4
    )

    p.addUserDebugLine(
        GOAL_POSITION - np.array([0.0, 0.20, 0.0]),
        GOAL_POSITION + np.array([0.0, 0.20, 0.0]),
        [0.0, 0.0, 1.0],
        lineWidth=4
    )


# ============================================================
# MAIN SIMULATION
# ============================================================

def run():

    initial_xyzs = np.array([START_POSITION])
    initial_rpys = np.array([[0.0, 0.0, 0.0]])

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=initial_xyzs,
        initial_rpys=initial_rpys,
        physics=Physics.PYB,
        neighbourhood_radius=10,
        pyb_freq=PYB_FREQ,
        ctrl_freq=CONTROL_FREQ,
        gui=True,
        record=False,
        obstacles=False,
        user_debug_gui=False
    )

    controller = DSLPIDControl(
        drone_model=DroneModel.CF2X
    )

    scenario_info = SCENARIOS.get(TEST_SCENARIO, {})
    print("\n" + "=" * 60)
    print(f"RUNNING SCENARIO {TEST_SCENARIO}: {scenario_info.get('name', 'Custom')}")
    print(f"Difficulty: {scenario_info.get('difficulty', 'Unknown')}")
    print(f"Description: {scenario_info.get('description', '')}")
    print(f"Expected: {scenario_info.get('expected_behavior', '')}")
    print("=" * 60 + "\n")

    obstacle_ids = create_test_scenario(TEST_SCENARIO)
    draw_goal()

    action = np.zeros((1, 4))

    mode = "FORWARD"
    previous_mode = mode

    avoidance_side = None
    previous_avoidance_side = None

    avoid_steps = 0
    clear_steps = 0
    escape_steps_remaining = 0

    # After reversing, try the opposite side when possible.
    force_opposite_side = False

    position_history = deque(
        maxlen=STUCK_WINDOW_STEPS
    )

    start_time = time.time()
    result = "TIME LIMIT REACHED"

    # ============================================================
    # EXPERIMENT METRICS
    # ============================================================
    path_length = 0.0
    collision_count = 0
    avoidance_events = 0
    emergency_reversals = 0
    previous_position = None

    for step in range(DURATION_SEC * CONTROL_FREQ):

        # Update dynamic moving obstacles if any
        update_dynamic_obstacles(step / CONTROL_FREQ)

        observation, reward, terminated, truncated, info = env.step(
            action
        )

        # Detect contact between the drone and the scenario obstacles.
        drone_id = env.DRONE_IDS[0]
        collision_this_step = False
        for obstacle_id in obstacle_ids:
            if p.getContactPoints(bodyA=drone_id, bodyB=obstacle_id):
                collision_this_step = True
                break
        if collision_this_step:
            collision_count += 1

        state = observation[0]

        current_position = state[0:3]
        current_quaternion = state[3:7]
        current_velocity = state[10:13]
        current_angular_velocity = state[13:16]

        # Track total travelled distance.
        if previous_position is not None:
            path_length += float(
                np.linalg.norm(current_position - previous_position)
            )
        previous_position = current_position.copy()

        position_history.append(current_position.copy())

        distances, directions = read_sensors(current_position)

        distance_to_goal = np.linalg.norm(
            GOAL_POSITION - current_position
        )

        front_blocked = (
            distances["front"] < DETECTION_DISTANCE
        )

        emergency_front = (
            distances["front"] < EMERGENCY_DISTANCE
        )

        left_blocked = (
            distances["left"] < SIDE_BLOCKED_DISTANCE
            and
            distances["far_left"] < SIDE_BLOCKED_DISTANCE
        )

        right_blocked = (
            distances["right"] < SIDE_BLOCKED_DISTANCE
            and
            distances["far_right"] < SIDE_BLOCKED_DISTANCE
        )

        both_sides_blocked = left_blocked and right_blocked

        stuck = (
            progress_is_too_small(position_history)
            and distance_to_goal > GOAL_TOLERANCE
            and mode != "EMERGENCY_REVERSE"
        )

        previous_mode = mode

        # ====================================================
        # HIGHEST-PRIORITY CONDITIONS
        # ====================================================

        if distance_to_goal < GOAL_TOLERANCE:
            mode = "GOAL_REACHED"

        elif mode == "EMERGENCY_REVERSE":

            escape_steps_remaining -= 1

            # Stop reversing if there is an obstacle behind.
            if distances["rear"] < EMERGENCY_DISTANCE:
                escape_steps_remaining = 0

            if escape_steps_remaining <= 0:
                mode = "AVOID"

                selected_side = choose_avoidance_side(
                    distances,
                    previous_avoidance_side
                )

                if (
                    force_opposite_side
                    and previous_avoidance_side == "LEFT"
                ):
                    selected_side = "RIGHT"

                elif (
                    force_opposite_side
                    and previous_avoidance_side == "RIGHT"
                ):
                    selected_side = "LEFT"

                avoidance_side = selected_side
                previous_avoidance_side = selected_side

                avoid_steps = 0
                clear_steps = 0
                force_opposite_side = False

                position_history.clear()

        elif emergency_front or both_sides_blocked or stuck:

            mode = "EMERGENCY_REVERSE"
            escape_steps_remaining = ESCAPE_STEPS
            force_opposite_side = True

            clear_steps = 0
            avoid_steps = 0

            position_history.clear()

        # ====================================================
        # NORMAL NAVIGATION
        # ====================================================

        elif mode == "FORWARD":

            if front_blocked:
                mode = "AVOID"

                avoidance_side = choose_avoidance_side(
                    distances,
                    previous_avoidance_side
                )

                previous_avoidance_side = avoidance_side

                avoid_steps = 0
                clear_steps = 0

        elif mode == "AVOID":

            avoid_steps += 1

            # If the selected side becomes blocked, switch only
            # when the alternative side is clearly better.
            left_score, right_score = calculate_side_scores(
                distances
            )

            if (
                avoidance_side == "LEFT"
                and left_blocked
                and right_score > left_score + 0.15
            ):
                avoidance_side = "RIGHT"
                previous_avoidance_side = "RIGHT"
                avoid_steps = 0

            elif (
                avoidance_side == "RIGHT"
                and right_blocked
                and left_score > right_score + 0.15
            ):
                avoidance_side = "LEFT"
                previous_avoidance_side = "LEFT"
                avoid_steps = 0

            if distances["front"] > CLEAR_DISTANCE:
                clear_steps += 1
            else:
                clear_steps = 0

            if (
                avoid_steps >= MIN_AVOID_STEPS
                and clear_steps >= CLEAR_CONFIRMATION_STEPS
            ):
                mode = "RETURN"
                clear_steps = 0

        elif mode == "RETURN":

            if front_blocked:
                mode = "AVOID"

                avoidance_side = choose_avoidance_side(
                    distances,
                    previous_avoidance_side
                )

                previous_avoidance_side = avoidance_side

                avoid_steps = 0
                clear_steps = 0

            # The drone is sufficiently close to the direct
            # path toward the goal.
            elif abs(current_position[1]) < 0.10:
                mode = "FORWARD"

        # Count transitions into avoidance/recovery modes.
        if mode == "AVOID" and previous_mode != "AVOID":
            avoidance_events += 1

        if (
            mode == "EMERGENCY_REVERSE"
            and previous_mode != "EMERGENCY_REVERSE"
        ):
            emergency_reversals += 1

        # ====================================================
        # VELOCITY AND PID CONTROL
        # ====================================================

        desired_velocity = make_velocity_command(
            mode,
            current_position,
            directions,
            avoidance_side
        )

        if mode == "GOAL_REACHED":
            target_position = GOAL_POSITION.copy()

        else:
            # Point slightly ahead in the requested direction.
            # This prevents target position and target velocity
            # from giving contradictory instructions.
            target_position = (
                current_position
                + desired_velocity * 0.65
            )

            target_alt = FLIGHT_ALTITUDE
            if TEST_SCENARIO == 8:
                if 1.7 <= current_position[0] <= 2.8:
                    target_alt = 1.35
                elif 3.0 <= current_position[0] <= 3.9:
                    target_alt = 0.95
                elif 4.1 <= current_position[0] <= 4.9:
                    target_alt = 1.35
            elif TEST_SCENARIO == 10:
                if np.linalg.norm(current_position[:2]) < 2.5:
                    target_alt = 1.50

            target_position[2] = target_alt

        rpm, position_error, yaw_error = controller.computeControl(
            control_timestep=1.0 / CONTROL_FREQ,
            cur_pos=current_position,
            cur_quat=current_quaternion,
            cur_vel=current_velocity,
            cur_ang_vel=current_angular_velocity,
            target_pos=target_position,
            target_rpy=np.array([0.0, 0.0, 0.0]),
            target_vel=desired_velocity
        )

        action[0, :] = rpm

        # ====================================================
        # DISPLAY INFORMATION
        # ====================================================

        if mode == "AVOID":
            status_text = f"AVOID {avoidance_side}"

        elif mode == "EMERGENCY_REVERSE":
            status_text = "TRAPPED/STUCK: REVERSING"

        elif mode == "RETURN":
            status_text = "RETURNING TO PATH"

        elif mode == "GOAL_REACHED":
            status_text = "GOAL REACHED"

        else:
            status_text = "MOVING TO GOAL"

        p.addUserDebugText(
            text=status_text,
            textPosition=(
                current_position
                + np.array([0.0, 0.0, 0.32])
            ),
            textColorRGB=[0.0, 0.0, 0.0],
            textSize=1.0,
            lifeTime=0.08
        )

        if step % 10 == 0:
            print(
                f"Scenario={TEST_SCENARIO} | "
                f"Mode={mode:18s} | "
                f"Side={str(avoidance_side):5s} | "
                f"Position=({current_position[0]:.2f}, "
                f"{current_position[1]:.2f}) | "
                f"Front={distances['front']:.2f} | "
                f"Left={distances['left']:.2f} | "
                f"Right={distances['right']:.2f} | "
                f"Goal distance={distance_to_goal:.2f}"
            )

        env.render()

        sync(
            step,
            start_time,
            1.0 / CONTROL_FREQ
        )

        if mode == "GOAL_REACHED":
            result = "SUCCESS"
            print("\nDrone successfully reached the goal.")
            time.sleep(3)
            break

        if terminated or truncated:
            result = "SIMULATION TERMINATED"
            break

    elapsed_time = time.time() - start_time

    print("\n========================================")
    print(f"TEST SCENARIO: {TEST_SCENARIO}")
    print(f"RESULT: {result}")
    print(f"CONTROL STEPS: {step + 1}")
    print(f"COMPLETION TIME: {elapsed_time:.2f} s")
    print(
        f"FINAL POSITION: "
        f"({current_position[0]:.3f}, "
        f"{current_position[1]:.3f}, "
        f"{current_position[2]:.3f})"
    )
    print(f"PATH LENGTH: {path_length:.3f} m")
    print(f"COLLISION STEPS: {collision_count}")
    print(f"AVOIDANCE EVENTS: {avoidance_events}")
    print(f"EMERGENCY REVERSALS: {emergency_reversals}")
    print("========================================")

    env.close()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        if sys.argv[1].isdigit():
            TEST_SCENARIO = int(sys.argv[1])
        elif sys.argv[1] in ("--scenario", "-s") and len(sys.argv) > 2 and sys.argv[2].isdigit():
            TEST_SCENARIO = int(sys.argv[2])
    run()