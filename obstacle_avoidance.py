import time
from collections import deque

import numpy as np
import pybullet as p
import math
import cv2

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync


# ============================================================
# TEST SCENARIO
# ============================================================
# 1 = one obstacle directly ahead
# 2 = obstacle slightly left
# 3 = obstacle slightly right
# 4 = narrow passage
# 5 = dead end / both sides blocked
# 6 = multiple obstacles
# 7 = circular arrangement

TEST_SCENARIO = 7


# ============================================================
# SIMULATION SETTINGS
# ============================================================

PYB_FREQ = 240
CONTROL_FREQ = 48
DURATION_SEC = 60

FLIGHT_ALTITUDE = 1.0

START_POSITION = np.array([2.5, 0.0, FLIGHT_ALTITUDE])
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

MIN_AVOID_STEPS = int(1.0 * CONTROL_FREQ)

CLEAR_CONFIRMATION_STEPS = int(0.35 * CONTROL_FREQ)

ESCAPE_STEPS = int(1.2 * CONTROL_FREQ)

STUCK_WINDOW_STEPS = int(2.0 * CONTROL_FREQ)
MIN_PROGRESS_DISTANCE = 0.10


# ============================================================
# CAMERA SETTINGS
# ============================================================

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

CAMERA_FOV = 60
CAMERA_NEAR = 0.05
CAMERA_FAR = 10.0

# Camera position relative to drone
CAMERA_FORWARD_OFFSET = 0.15
CAMERA_HEIGHT_OFFSET = 0.02


# ============================================================
# OBSTACLE CREATION
# ============================================================

def add_cube(position, size=0.30, colour=None):

    """
    Add a stationary cube obstacle.
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


def create_test_scenario(scenario):

    """
    Create different obstacle arrangements.
    Returns PyBullet IDs of the obstacles.
    """

    obstacle_ids = []

    z = FLIGHT_ALTITUDE

    if scenario == 1:

        # Direct frontal obstacle
        obstacle_ids.append(
            add_cube([2.5, 0.0, z], size=0.30)
        )

    elif scenario == 2:

        # Obstacle slightly toward the left
        obstacle_ids.append(
            add_cube([2.5, 0.18, z], size=0.30)
        )

    elif scenario == 3:

        # Obstacle slightly toward the right
        obstacle_ids.append(
            add_cube([2.5, -0.18, z], size=0.30)
        )

    elif scenario == 4:

        # Narrow passage

        obstacle_ids.append(
            add_cube([2.5, 0.38, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([2.5, -0.38, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([3.0, 0.42, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([3.0, -0.42, z], size=0.30)
        )

    elif scenario == 5:

        # Dead-end-like arrangement

        obstacle_ids.append(
            add_cube([2.5, 0.0, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([2.35, 0.36, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([2.35, -0.36, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([2.0, 0.52, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([2.0, -0.52, z], size=0.30)
        )

    elif scenario == 6:

        # Multiple staggered obstacles

        obstacle_ids.append(
            add_cube([1.8, 0.0, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([3.0, 0.45, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([4.0, -0.40, z], size=0.30)
        )

        obstacle_ids.append(
            add_cube([5.0, 0.05, z], size=0.30)
        )

    elif scenario == 7:

        # Circular arrangement

        center_x = 2.5
        center_y = 0.0

        radius = 2.8
        num_cubes = 80

        for i in range(num_cubes):

            angle = 2 * math.pi * i / num_cubes

            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)

            obstacle_ids.append(
                add_cube([x, y, z], size=0.30)
            )

    else:

        raise ValueError(
            "TEST_SCENARIO must be an integer from 1 to 7."
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
    Return horizontal direction from drone to goal.
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

    # Start beyond central drone body
    ray_start = drone_position + direction * 0.18

    ray_end = ray_start + direction * maximum_range

    result = p.rayTest(
        ray_start,
        ray_end
    )[0]

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

    far-left : +70°
    left     : +35°
    front    : 0°
    right    : -35°
    far-right: -70°
    rear     : 180°
    """

    base_direction = goal_direction(
        current_position
    )

    directions = {

        "far_left":
            rotate_horizontal(
                base_direction,
                70
            ),

        "left":
            rotate_horizontal(
                base_direction,
                35
            ),

        "front":
            base_direction,

        "right":
            rotate_horizontal(
                base_direction,
                -35
            ),

        "far_right":
            rotate_horizontal(
                base_direction,
                -70
            ),

        "rear":
            rotate_horizontal(
                base_direction,
                180
            )
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
# CAMERA FUNCTION
# ============================================================

def get_drone_camera(
    current_position,
    current_quaternion
):

    """
    Generate a forward-facing camera image
    attached to the drone.
    """

    # Convert quaternion to rotation matrix

    rotation_matrix = np.array(
        p.getMatrixFromQuaternion(
            current_quaternion
        )
    ).reshape(3, 3)


    # --------------------------------------------------------
    # Camera position
    # --------------------------------------------------------

    local_camera_position = np.array([
        CAMERA_FORWARD_OFFSET,
        0.0,
        CAMERA_HEIGHT_OFFSET
    ])

    camera_position = (
        current_position
        + rotation_matrix @ local_camera_position
    )


    # --------------------------------------------------------
    # Camera viewing direction
    # --------------------------------------------------------

    # Drone local X-axis = forward

    forward_vector = (
        rotation_matrix
        @ np.array([1.0, 0.0, 0.0])
    )

    target_position = (
        camera_position
        + forward_vector
    )


    # --------------------------------------------------------
    # Camera up direction
    # --------------------------------------------------------

    up_vector = (
        rotation_matrix
        @ np.array([0.0, 0.0, 1.0])
    )


    # --------------------------------------------------------
    # View matrix
    # --------------------------------------------------------

    view_matrix = p.computeViewMatrix(

        cameraEyePosition=camera_position,

        cameraTargetPosition=target_position,

        cameraUpVector=up_vector
    )


    # --------------------------------------------------------
    # Projection matrix
    # --------------------------------------------------------

    projection_matrix = p.computeProjectionMatrixFOV(

        fov=CAMERA_FOV,

        aspect=CAMERA_WIDTH / CAMERA_HEIGHT,

        nearVal=CAMERA_NEAR,

        farVal=CAMERA_FAR
    )


    # --------------------------------------------------------
    # Capture image
    # --------------------------------------------------------

    image = p.getCameraImage(

        width=CAMERA_WIDTH,

        height=CAMERA_HEIGHT,

        viewMatrix=view_matrix,

        projectionMatrix=projection_matrix,

        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )


    # RGB image

    rgb_image = np.array(
        image[2],
        dtype=np.uint8
    )

    rgb_image = rgb_image.reshape(
        CAMERA_HEIGHT,
        CAMERA_WIDTH,
        4
    )

    rgb_image = rgb_image[:, :, :3]


    # PyBullet returns RGB.
    # OpenCV displays BGR.

    bgr_image = cv2.cvtColor(
        rgb_image,
        cv2.COLOR_RGB2BGR
    )

    return bgr_image


# ============================================================
# DECISION FUNCTIONS
# ============================================================

def calculate_side_scores(distances):

    """
    Higher score = more free space.
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


def choose_avoidance_side(
    distances,
    previous_side=None
):

    """
    Choose side with more available space.
    """

    left_score, right_score = (
        calculate_side_scores(distances)
    )

    score_difference = abs(
        left_score - right_score
    )


    if (
        score_difference < 0.08
        and previous_side is not None
    ):

        return previous_side


    if left_score >= right_score:

        return "LEFT"


    return "RIGHT"


def progress_is_too_small(
    position_history
):

    """
    Determine whether drone has become stuck.
    """

    if len(position_history) < STUCK_WINDOW_STEPS:

        return False


    oldest_position = position_history[0]

    newest_position = position_history[-1]


    horizontal_progress = np.linalg.norm(

        newest_position[:2]
        - oldest_position[:2]
    )


    return (
        horizontal_progress
        < MIN_PROGRESS_DISTANCE
    )


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
    Generate velocity command for
    current controller mode.
    """

    toward_goal = directions["front"]


    left_vector = rotate_horizontal(
        toward_goal,
        90
    )


    right_vector = rotate_horizontal(
        toward_goal,
        -90
    )


    if mode == "FORWARD":

        velocity = (
            toward_goal
            * FORWARD_SPEED
        )


    elif mode == "AVOID":

        if avoidance_side == "LEFT":

            lateral_vector = left_vector

        else:

            lateral_vector = right_vector


        velocity = (

            toward_goal
            * AVOID_FORWARD_SPEED

            +

            lateral_vector
            * SIDE_SPEED
        )


    elif mode == "RETURN":

        velocity = (
            toward_goal
            * RETURN_SPEED
        )


    elif mode == "EMERGENCY_REVERSE":

        velocity = (
            -toward_goal
            * REVERSE_SPEED
        )


    elif mode == "GOAL_REACHED":

        velocity = np.zeros(3)


    else:

        velocity = np.zeros(3)


    # Keep flight at constant altitude

    velocity[2] = 0.0


    return velocity


# ============================================================
# VISUALISATION
# ============================================================

def draw_goal():

    """
    Draw goal marker.
    """

    p.addUserDebugText(

        text="GOAL",

        textPosition=(
            GOAL_POSITION
            + np.array([0.0, 0.0, 0.30])
        ),

        textColorRGB=[
            0.0,
            0.0,
            1.0
        ],

        textSize=1.4
    )


    p.addUserDebugLine(

        GOAL_POSITION
        - np.array([0.20, 0.0, 0.0]),

        GOAL_POSITION
        + np.array([0.20, 0.0, 0.0]),

        [0.0, 0.0, 1.0],

        lineWidth=4
    )


    p.addUserDebugLine(

        GOAL_POSITION
        - np.array([0.0, 0.20, 0.0]),

        GOAL_POSITION
        + np.array([0.0, 0.20, 0.0]),

        [0.0, 0.0, 1.0],

        lineWidth=4
    )


# ============================================================
# MAIN SIMULATION
# ============================================================

def run():

    initial_xyzs = np.array([
        START_POSITION
    ])


    initial_rpys = np.array([
        [0.0, 0.0, 0.0]
    ])


    # --------------------------------------------------------
    # Create drone environment
    # --------------------------------------------------------

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


    # --------------------------------------------------------
    # PID controller
    # --------------------------------------------------------

    controller = DSLPIDControl(

        drone_model=DroneModel.CF2X
    )


    # --------------------------------------------------------
    # Create obstacles
    # --------------------------------------------------------

    obstacle_ids = create_test_scenario(
        TEST_SCENARIO
    )


    draw_goal()


    # --------------------------------------------------------
    # Action
    # --------------------------------------------------------

    action = np.zeros(
        (1, 4)
    )


    # --------------------------------------------------------
    # Navigation state
    # --------------------------------------------------------

    mode = "FORWARD"

    previous_mode = mode

    avoidance_side = None

    previous_avoidance_side = None

    avoid_steps = 0

    clear_steps = 0

    escape_steps_remaining = 0

    force_opposite_side = False


    position_history = deque(
        maxlen=STUCK_WINDOW_STEPS
    )


    start_time = time.time()

    result = "TIME LIMIT REACHED"


    # --------------------------------------------------------
    # Experiment metrics
    # --------------------------------------------------------

    path_length = 0.0

    collision_count = 0

    avoidance_events = 0

    emergency_reversals = 0

    previous_position = None


    # ========================================================
    # CONTROL LOOP
    # ========================================================

    for step in range(
        DURATION_SEC * CONTROL_FREQ
    ):


        # ----------------------------------------------------
        # Step simulation
        # ----------------------------------------------------

        observation, reward, terminated, truncated, info = (
            env.step(action)
        )


        # ----------------------------------------------------
        # Drone ID
        # ----------------------------------------------------

        drone_id = env.DRONE_IDS[0]


        # ----------------------------------------------------
        # Collision detection
        # ----------------------------------------------------

        collision_this_step = False


        for obstacle_id in obstacle_ids:

            if p.getContactPoints(
                bodyA=drone_id,
                bodyB=obstacle_id
            ):

                collision_this_step = True

                break


        if collision_this_step:

            collision_count += 1


        # ----------------------------------------------------
        # Read drone state
        # ----------------------------------------------------

        state = observation[0]


        current_position = state[0:3]

        current_quaternion = state[3:7]

        current_velocity = state[10:13]

        current_angular_velocity = state[13:16]


        # ----------------------------------------------------
        # CAMERA
        # ----------------------------------------------------

        camera_image = get_drone_camera(

            current_position,

            current_quaternion
        )


        # ----------------------------------------------------
        # Display camera
        # ----------------------------------------------------

        cv2.imshow(
            "Drone Camera",
            camera_image
        )


        # Press Q to stop

        if cv2.waitKey(1) & 0xFF == ord("q"):

            result = "STOPPED BY USER"

            break


        # ----------------------------------------------------
        # Track travelled distance
        # ----------------------------------------------------

        if previous_position is not None:

            path_length += float(

                np.linalg.norm(

                    current_position
                    - previous_position
                )
            )


        previous_position = (
            current_position.copy()
        )


        position_history.append(
            current_position.copy()
        )


        # ----------------------------------------------------
        # Read virtual sensors
        # ----------------------------------------------------

        distances, directions = read_sensors(
            current_position
        )


        # ----------------------------------------------------
        # Distance to goal
        # ----------------------------------------------------

        distance_to_goal = np.linalg.norm(

            GOAL_POSITION
            - current_position
        )


        # ----------------------------------------------------
        # Obstacle conditions
        # ----------------------------------------------------

        front_blocked = (

            distances["front"]
            < DETECTION_DISTANCE
        )


        emergency_front = (

            distances["front"]
            < EMERGENCY_DISTANCE
        )


        left_blocked = (

            distances["left"]
            < SIDE_BLOCKED_DISTANCE

            and

            distances["far_left"]
            < SIDE_BLOCKED_DISTANCE
        )


        right_blocked = (

            distances["right"]
            < SIDE_BLOCKED_DISTANCE

            and

            distances["far_right"]
            < SIDE_BLOCKED_DISTANCE
        )


        both_sides_blocked = (

            left_blocked
            and right_blocked
        )


        stuck = (

            progress_is_too_small(
                position_history
            )

            and

            distance_to_goal
            > GOAL_TOLERANCE

            and

            mode
            != "EMERGENCY_REVERSE"
        )


        previous_mode = mode


        # ====================================================
        # HIGHEST PRIORITY CONDITIONS
        # ====================================================

        if distance_to_goal < GOAL_TOLERANCE:

            mode = "GOAL_REACHED"


        elif mode == "EMERGENCY_REVERSE":

            escape_steps_remaining -= 1


            # Stop reversing if obstacle behind

            if (
                distances["rear"]
                < EMERGENCY_DISTANCE
            ):

                escape_steps_remaining = 0


            if escape_steps_remaining <= 0:

                mode = "AVOID"


                selected_side = (
                    choose_avoidance_side(

                        distances,

                        previous_avoidance_side
                    )
                )


                # Try opposite side

                if (

                    force_opposite_side

                    and

                    previous_avoidance_side
                    == "LEFT"

                ):

                    selected_side = "RIGHT"


                elif (

                    force_opposite_side

                    and

                    previous_avoidance_side
                    == "RIGHT"

                ):

                    selected_side = "LEFT"


                avoidance_side = (
                    selected_side
                )


                previous_avoidance_side = (
                    selected_side
                )


                avoid_steps = 0

                clear_steps = 0

                force_opposite_side = False

                position_history.clear()


        elif (

            emergency_front

            or

            both_sides_blocked

            or

            stuck

        ):

            mode = "EMERGENCY_REVERSE"

            escape_steps_remaining = (
                ESCAPE_STEPS
            )

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


                avoidance_side = (
                    choose_avoidance_side(

                        distances,

                        previous_avoidance_side
                    )
                )


                previous_avoidance_side = (
                    avoidance_side
                )


                avoid_steps = 0

                clear_steps = 0


        elif mode == "AVOID":

            avoid_steps += 1


            # Recalculate side scores

            left_score, right_score = (
                calculate_side_scores(
                    distances
                )
            )


            # Switch from LEFT to RIGHT

            if (

                avoidance_side == "LEFT"

                and

                left_blocked

                and

                right_score
                > left_score + 0.15

            ):

                avoidance_side = "RIGHT"

                previous_avoidance_side = "RIGHT"

                avoid_steps = 0


            # Switch from RIGHT to LEFT

            elif (

                avoidance_side == "RIGHT"

                and

                right_blocked

                and

                left_score
                > right_score + 0.15

            ):

                avoidance_side = "LEFT"

                previous_avoidance_side = "LEFT"

                avoid_steps = 0


            # Check clear path

            if (

                distances["front"]
                > CLEAR_DISTANCE

            ):

                clear_steps += 1

            else:

                clear_steps = 0


            if (

                avoid_steps
                >= MIN_AVOID_STEPS

                and

                clear_steps
                >= CLEAR_CONFIRMATION_STEPS

            ):

                mode = "RETURN"

                clear_steps = 0


        elif mode == "RETURN":

            if front_blocked:

                mode = "AVOID"


                avoidance_side = (
                    choose_avoidance_side(

                        distances,

                        previous_avoidance_side
                    )
                )


                previous_avoidance_side = (
                    avoidance_side
                )


                avoid_steps = 0

                clear_steps = 0


            # Return to direct path

            elif abs(
                current_position[1]
            ) < 0.10:

                mode = "FORWARD"


        # ====================================================
        # COUNT EVENTS
        # ====================================================

        if (

            mode == "AVOID"

            and

            previous_mode != "AVOID"

        ):

            avoidance_events += 1


        if (

            mode == "EMERGENCY_REVERSE"

            and

            previous_mode
            != "EMERGENCY_REVERSE"

        ):

            emergency_reversals += 1


        # ====================================================
        # VELOCITY AND PID CONTROL
        # ====================================================

        desired_velocity = (
            make_velocity_command(

                mode,

                current_position,

                directions,

                avoidance_side
            )
        )


        if mode == "GOAL_REACHED":

            target_position = (
                GOAL_POSITION.copy()
            )

        else:

            # Point slightly ahead

            target_position = (

                current_position

                +

                desired_velocity
                * 0.65
            )


            target_position[2] = (
                FLIGHT_ALTITUDE
            )


        # ----------------------------------------------------
        # PID controller
        # ----------------------------------------------------

        rpm, position_error, yaw_error = (
            controller.computeControl(

                control_timestep=(
                    1.0 / CONTROL_FREQ
                ),

                cur_pos=current_position,

                cur_quat=current_quaternion,

                cur_vel=current_velocity,

                cur_ang_vel=current_angular_velocity,

                target_pos=target_position,

                target_rpy=np.array([
                    0.0,
                    0.0,
                    0.0
                ]),

                target_vel=desired_velocity
            )
        )


        action[0, :] = rpm


        # ====================================================
        # DISPLAY INFORMATION
        # ====================================================

        if mode == "AVOID":

            status_text = (
                f"AVOID {avoidance_side}"
            )


        elif mode == "EMERGENCY_REVERSE":

            status_text = (
                "TRAPPED/STUCK: REVERSING"
            )


        elif mode == "RETURN":

            status_text = (
                "RETURNING TO PATH"
            )


        elif mode == "GOAL_REACHED":

            status_text = (
                "GOAL REACHED"
            )


        else:

            status_text = (
                "MOVING TO GOAL"
            )


        p.addUserDebugText(

            text=status_text,

            textPosition=(

                current_position

                +

                np.array([
                    0.0,
                    0.0,
                    0.32
                ])
            ),

            textColorRGB=[
                0.0,
                0.0,
                0.0
            ],

            textSize=1.0,

            lifeTime=0.08
        )


        # ----------------------------------------------------
        # Terminal information
        # ----------------------------------------------------

        if step % 10 == 0:

            print(

                f"Scenario={TEST_SCENARIO} | "

                f"Mode={mode:18s} | "

                f"Side={str(avoidance_side):5s} | "

                f"Position=("

                f"{current_position[0]:.2f}, "

                f"{current_position[1]:.2f}) | "

                f"Front="
                f"{distances['front']:.2f} | "

                f"Left="
                f"{distances['left']:.2f} | "

                f"Right="
                f"{distances['right']:.2f} | "

                f"Goal distance="
                f"{distance_to_goal:.2f}"
            )


        # ----------------------------------------------------
        # Render simulation
        # ----------------------------------------------------

        env.render()


        sync(

            step,

            start_time,

            1.0 / CONTROL_FREQ
        )


        # ----------------------------------------------------
        # Goal reached
        # ----------------------------------------------------

        if mode == "GOAL_REACHED":

            result = "SUCCESS"

            print(
                "\nDrone successfully "
                "reached the goal."
            )

            time.sleep(3)

            break


        if terminated or truncated:

            result = (
                "SIMULATION TERMINATED"
            )

            break


    # ========================================================
    # FINAL RESULTS
    # ========================================================

    elapsed_time = (
        time.time()
        - start_time
    )


    print(
        "\n========================================"
    )

    print(
        f"TEST SCENARIO: "
        f"{TEST_SCENARIO}"
    )

    print(
        f"RESULT: "
        f"{result}"
    )

    print(
        f"CONTROL STEPS: "
        f"{step + 1}"
    )

    print(
        f"COMPLETION TIME: "
        f"{elapsed_time:.2f} s"
    )

    print(

        f"FINAL POSITION: "

        f"({current_position[0]:.3f}, "

        f"{current_position[1]:.3f}, "

        f"{current_position[2]:.3f})"
    )

    print(
        f"PATH LENGTH: "
        f"{path_length:.3f} m"
    )

    print(
        f"COLLISION STEPS: "
        f"{collision_count}"
    )

    print(
        f"AVOIDANCE EVENTS: "
        f"{avoidance_events}"
    )

    print(
        f"EMERGENCY REVERSALS: "
        f"{emergency_reversals}"
    )

    print(
        "========================================"
    )


    # --------------------------------------------------------
    # Close everything
    # --------------------------------------------------------

    cv2.destroyAllWindows()

    env.close()


# ============================================================
# PROGRAM ENTRY POINT
# ============================================================

if __name__ == "__main__":

    run()