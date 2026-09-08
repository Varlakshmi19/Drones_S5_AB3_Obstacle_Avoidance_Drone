#!/usr/bin/env python3

import math
import time
from collections import deque

import numpy as np

from pymavlink import mavutil

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan


# ==============================================================
# CONFIGURATION
# ==============================================================

MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

LIDAR_TOPIC = "/iris/lidar"

# --------------------------------------------------------------
# Flight
# --------------------------------------------------------------

TAKEOFF_ALTITUDE = 1.0

FORWARD_SPEED = 0.35
AVOID_FORWARD_SPEED = 0.18
SIDE_SPEED = 0.38
RETURN_SPEED = 0.28
REVERSE_SPEED = 0.22

# --------------------------------------------------------------
# Goal
# --------------------------------------------------------------

# Gazebo world coordinates
GOAL_X = 0.0
GOAL_Y = 6.0

# NED coordinates:
# Gazebo +Z is up, MAVLink LOCAL_POSITION_NED +Z is down.
GOAL_Z_NED = -TAKEOFF_ALTITUDE

GOAL_TOLERANCE = 0.25

# --------------------------------------------------------------
# LiDAR
# --------------------------------------------------------------

MAX_LIDAR_RANGE = 10.0

SENSOR_RANGE = 1.6

DETECTION_DISTANCE = 0.75
EMERGENCY_DISTANCE = 0.30
SIDE_BLOCKED = 0.48
CLEAR_DISTANCE = 1.0

# The LiDAR is rotated approximately 90 degrees
# relative to the drone body frame.
LIDAR_YAW_OFFSET = math.radians(90.0)

# Six virtual sensors used by the original PyBullet controller.
SENSOR_ANGLES = {
    "FAR_LEFT": math.radians(70.0),
    "LEFT": math.radians(35.0),
    "FRONT": math.radians(0.0),
    "RIGHT": math.radians(-35.0),
    "FAR_RIGHT": math.radians(-70.0),
    "REAR": math.radians(180.0),
}

SENSOR_WINDOW = math.radians(5.0)

# --------------------------------------------------------------
# Controller frequency
# --------------------------------------------------------------

CONTROL_HZ = 10.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ

MAX_CONTROL_TIME = 60.0

# --------------------------------------------------------------
# Timing values
#
# These are scaled for a 10 Hz controller.
# The original PyBullet controller ran at 48 Hz.
# --------------------------------------------------------------

MIN_AVOID_TIME = 1.0

CLEAR_CONFIRMATION_TIME = 0.35

ESCAPE_TIME = 1.2

STUCK_WINDOW_TIME = 2.0

MIN_PROGRESS_DISTANCE = 0.10

MIN_AVOID_STEPS = max(
    1,
    int(MIN_AVOID_TIME * CONTROL_HZ)
)

CLEAR_CONFIRMATION_STEPS = max(
    1,
    int(CLEAR_CONFIRMATION_TIME * CONTROL_HZ)
)

ESCAPE_STEPS = max(
    1,
    int(ESCAPE_TIME * CONTROL_HZ)
)

STUCK_WINDOW_STEPS = max(
    2,
    int(STUCK_WINDOW_TIME * CONTROL_HZ)
)


# ==============================================================
# STATES
# ==============================================================

FORWARD = "FORWARD"
AVOID = "AVOID"
RETURN = "RETURN"
EMERGENCY_REVERSE = "EMERGENCY_REVERSE"
GOAL_REACHED = "GOAL_REACHED"


# ==============================================================
# GLOBAL LIDAR DATA
# ==============================================================

latest_scan = None
latest_scan_time = 0.0


# ==============================================================
# HELPERS
# ==============================================================

def wrap_angle(angle):
    """
    Wrap angle to [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def distance_2d(p1, p2):
    """
    Horizontal Euclidean distance.
    """

    dx = float(p1[0]) - float(p2[0])
    dy = float(p1[1]) - float(p2[1])

    return math.sqrt(dx * dx + dy * dy)


def normalize_vector(v):
    """
    Normalize a 2D vector.
    """

    norm = math.sqrt(v[0] ** 2 + v[1] ** 2)

    if norm < 1e-6:
        return np.array([1.0, 0.0])

    return np.array([
        v[0] / norm,
        v[1] / norm
    ])


# ==============================================================
# LIDAR CALLBACK
# ==============================================================

def lidar_callback(msg):
    """
    Store the latest Gazebo LaserScan message.
    """

    global latest_scan
    global latest_scan_time

    latest_scan = msg
    latest_scan_time = time.time()


# ==============================================================
# MAVLINK CONNECTION
# ==============================================================

def connect_vehicle():

    print("Connecting to ArduPilot...")

    master = mavutil.mavlink_connection(
        MAVLINK_CONNECTION
    )

    master.wait_heartbeat()

    print(
        f"Connected to system "
        f"{master.target_system}, "
        f"component "
        f"{master.target_component}"
    )

    return master


# ==============================================================
# GUIDED MODE
# ==============================================================

def set_guided_mode(master):

    print("Setting GUIDED mode...")

    mode_mapping = master.mode_mapping()

    if "GUIDED" not in mode_mapping:
        raise RuntimeError(
            "GUIDED mode is not available."
        )

    guided_mode = mode_mapping["GUIDED"]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        guided_mode
    )

    time.sleep(1.0)

    print("GUIDED mode requested.")


# ==============================================================
# ARM
# ==============================================================

def arm_vehicle(master):

    print("Arming vehicle...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0
    )

    master.motors_armed_wait()

    print("Vehicle armed.")


# ==============================================================
# TAKEOFF
# ==============================================================

def takeoff(master):

    print(
        f"Taking off to "
        f"{TAKEOFF_ALTITUDE:.1f} m..."
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        TAKEOFF_ALTITUDE
    )

    start = time.time()

    while time.time() - start < 15.0:

        position = get_local_position(master)

        if position is not None:

            altitude = -float(position[2])

            print(
                f"Altitude: "
                f"{altitude:.2f} m"
            )

            if altitude >= TAKEOFF_ALTITUDE * 0.80:
                print("Takeoff complete.")
                return

        time.sleep(0.2)

    print(
        "Takeoff wait finished; "
        "continuing with controller."
    )


# ==============================================================
# LOCAL POSITION
# ==============================================================

def get_local_position(master):

    msg = master.recv_match(
        type="LOCAL_POSITION_NED",
        blocking=False
    )

    if msg is None:
        return None

    return np.array([
        float(msg.x),
        float(msg.y),
        float(msg.z)
    ])


# ==============================================================
# ATTITUDE / YAW
# ==============================================================

def get_yaw(master):

    msg = master.recv_match(
        type="ATTITUDE",
        blocking=False
    )

    if msg is None:
        return None

    return float(msg.yaw)


# ==============================================================
# VELOCITY COMMAND
# ==============================================================

def send_velocity_body(
    master,
    vx,
    vy,
    vz=0.0
):
    """
    Send velocity command in BODY_OFFSET_NED.

    Body frame:

        +X = forward
        +Y = right
        +Z = down
    """

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,

        master.target_system,
        master.target_component,

        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,

        0x0DC7,

        0,
        0,
        0,

        vx,
        vy,
        vz,

        0,
        0,
        0,

        0,
        0
    )


def stop_vehicle(master):

    send_velocity_body(
        master,
        0.0,
        0.0,
        0.0
    )


# ==============================================================
# WORLD VELOCITY -> BODY VELOCITY
# ==============================================================

def world_to_body_velocity(
    world_vx,
    world_vy,
    yaw
):
    """
    Convert horizontal velocity from world/local
    coordinates into the drone body frame.
    """

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    body_vx = (
        world_vx * cos_yaw
        + world_vy * sin_yaw
    )

    body_vy = (
        -world_vx * sin_yaw
        + world_vy * cos_yaw
    )

    return body_vx, body_vy


# ==============================================================
# SEND WORLD VELOCITY
# ==============================================================

def send_world_velocity(
    master,
    world_vx,
    world_vy,
    yaw
):

    body_vx, body_vy = world_to_body_velocity(
        world_vx,
        world_vy,
        yaw
    )

    send_velocity_body(
        master,
        body_vx,
        body_vy,
        0.0
    )


# ==============================================================
# LIDAR ANGLE
# ==============================================================

def get_distance_at_lidar_angle(
    scan,
    target_angle
):
    """
    Get minimum valid LiDAR distance around
    the requested angle.
    """

    if scan is None:
        return MAX_LIDAR_RANGE

    ranges = list(scan.ranges)

    if not ranges:
        return MAX_LIDAR_RANGE

    angle_min = float(
        getattr(
            scan,
            "angle_min",
            -math.pi
        )
    )

    angle_step = float(
        getattr(
            scan,
            "angle_step",
            2.0 * math.pi / len(ranges)
        )
    )

    if abs(angle_step) < 1e-8:
        return MAX_LIDAR_RANGE

    target_angle = wrap_angle(target_angle)

    center_index = int(
        round(
            (target_angle - angle_min)
            / angle_step
        )
    )

    window_count = max(
        1,
        int(
            SENSOR_WINDOW
            / abs(angle_step)
        )
    )

    minimum = MAX_LIDAR_RANGE

    for offset in range(
        -window_count,
        window_count + 1
    ):

        index = (
            center_index + offset
        ) % len(ranges)

        try:
            value = float(
                ranges[index]
            )
        except Exception:
            continue

        if math.isnan(value):
            continue

        if math.isinf(value):
            continue

        if value < 0.05:
            continue

        value = min(
            value,
            MAX_LIDAR_RANGE
        )

        minimum = min(
            minimum,
            value
        )

    return minimum


# ==============================================================
# BODY ANGLE -> LIDAR ANGLE
# ==============================================================

def get_distance_at_body_angle(
    scan,
    body_angle
):
    """
    Convert body-relative angle into
    the LiDAR sensor frame.

    Based on the current Gazebo model:
        LiDAR angle = body angle - 90 degrees
    """

    lidar_angle = (
        body_angle
        - LIDAR_YAW_OFFSET
    )

    return get_distance_at_lidar_angle(
        scan,
        lidar_angle
    )


# ==============================================================
# READ SIX VIRTUAL SENSORS
# ==============================================================

def read_sensors(
    scan,
    goal_body_angle
):

    distances = {}

    for name, offset in SENSOR_ANGLES.items():

        sensor_body_angle = (
            goal_body_angle
            + offset
        )

        distance = get_distance_at_body_angle(
            scan,
            sensor_body_angle
        )

        distances[name] = min(
            distance,
            SENSOR_RANGE
        )

    return distances


# ==============================================================
# SIDE SCORE
# ==============================================================

def calculate_side_scores(distances):

    left_score = (
        0.65 * distances["LEFT"]
        +
        0.35 * distances["FAR_LEFT"]
    )

    right_score = (
        0.65 * distances["RIGHT"]
        +
        0.35 * distances["FAR_RIGHT"]
    )

    return left_score, right_score


# ==============================================================
# CHOOSE AVOIDANCE SIDE
# ==============================================================

def choose_side(
    distances,
    previous_side=None,
    force_opposite=False
):

    left_score, right_score = (
        calculate_side_scores(
            distances
        )
    )

    # If previous side exists and we
    # are forced to switch, choose opposite.
    if force_opposite:

        if previous_side == "LEFT":
            return "RIGHT", right_score

        if previous_side == "RIGHT":
            return "LEFT", left_score

    # Avoid a blocked side.
    if (
        left_score < SIDE_BLOCKED
        and right_score >= SIDE_BLOCKED
    ):
        return "RIGHT", right_score

    if (
        right_score < SIDE_BLOCKED
        and left_score >= SIDE_BLOCKED
    ):
        return "LEFT", left_score

    # Choose clearer side.
    if left_score > right_score:
        return "LEFT", left_score

    return "RIGHT", right_score


# ==============================================================
# GOAL DIRECTION
# ==============================================================

def get_goal_direction(
    current_position
):

    dx = (
        GOAL_X
        - current_position[0]
    )

    dy = (
        GOAL_Y
        - current_position[1]
    )

    distance = math.sqrt(
        dx * dx
        +
        dy * dy
    )

    if distance < 1e-6:

        return (
            np.array([0.0, 0.0]),
            distance
        )

    direction = np.array([
        dx / distance,
        dy / distance
    ])

    return direction, distance


# ==============================================================
# GOAL BODY ANGLE
# ==============================================================

def get_goal_body_angle(
    current_position,
    yaw
):

    goal_dx = (
        GOAL_X
        - current_position[0]
    )

    goal_dy = (
        GOAL_Y
        - current_position[1]
    )

    world_goal_angle = math.atan2(
        goal_dy,
        goal_dx
    )

    body_goal_angle = wrap_angle(
        world_goal_angle - yaw
    )

    return body_goal_angle


# ==============================================================
# GOAL REACHED
# ==============================================================

def goal_reached(
    current_position
):

    horizontal_distance = distance_2d(
        current_position,
        np.array([
            GOAL_X,
            GOAL_Y,
            current_position[2]
        ])
    )

    altitude_error = abs(
        float(current_position[2])
        - GOAL_Z_NED
    )

    return (
        horizontal_distance
        <= GOAL_TOLERANCE
        and
        altitude_error
        <= GOAL_TOLERANCE
    )


# ==============================================================
# PATH LINE ERROR
# ==============================================================

def get_path_lateral_error(
    start_position,
    current_position
):
    """
    Distance from current position to the
    straight start -> goal line.

    This is the equivalent of the original
    PyBullet controller's return-to-path check.
    """

    start_xy = np.array([
        start_position[0],
        start_position[1]
    ])

    current_xy = np.array([
        current_position[0],
        current_position[1]
    ])

    goal_xy = np.array([
        GOAL_X,
        GOAL_Y
    ])

    path_vector = (
        goal_xy
        - start_xy
    )

    path_length = np.linalg.norm(
        path_vector
    )

    if path_length < 1e-6:
        return 0.0

    path_unit = (
        path_vector
        / path_length
    )

    relative = (
        current_xy
        - start_xy
    )

    # 2D cross-product magnitude.
    lateral_error = abs(
        relative[0] * path_unit[1]
        -
        relative[1] * path_unit[0]
    )

    return float(lateral_error)


# ==============================================================
# STUCK DETECTION
# ==============================================================

def is_stuck(
    position_history
):

    if len(position_history) < (
        STUCK_WINDOW_STEPS
    ):
        return False

    old_position = position_history[0]
    new_position = position_history[-1]

    progress = distance_2d(
        old_position,
        new_position
    )

    return (
        progress
        < MIN_PROGRESS_DISTANCE
    )


# ==============================================================
# MAIN
# ==============================================================

def main():

    global latest_scan

    # ----------------------------------------------------------
    # Connect
    # ----------------------------------------------------------

    master = connect_vehicle()

    # ----------------------------------------------------------
    # Start LiDAR subscriber
    # ----------------------------------------------------------

    node = Node()

    subscribed = node.subscribe(
        LaserScan,
        LIDAR_TOPIC,
        lidar_callback
    )

    if not subscribed:
        raise RuntimeError(
            "Failed to subscribe to "
            + LIDAR_TOPIC
        )

    print(
        f"Subscribed to {LIDAR_TOPIC}"
    )

    # ----------------------------------------------------------
    # Wait for LiDAR
    # ----------------------------------------------------------

    print("Waiting for LiDAR data...")

    lidar_wait_start = time.time()

    while latest_scan is None:

        if (
            time.time()
            - lidar_wait_start
            > 10.0
        ):
            print(
                "WARNING: No LiDAR data "
                "received after 10 seconds."
            )
            break

        time.sleep(0.1)

    if latest_scan is not None:
        print("LiDAR data received.")

    # ----------------------------------------------------------
    # Guided + arm + takeoff
    # ----------------------------------------------------------

    set_guided_mode(master)

    arm_vehicle(master)

    takeoff(master)

    # ----------------------------------------------------------
    # Get initial position
    # ----------------------------------------------------------

    print(
        "Waiting for valid "
        "LOCAL_POSITION_NED..."
    )

    current_position = None

    position_wait_start = time.time()

    while current_position is None:

        new_position = (
            get_local_position(master)
        )

        if new_position is not None:
            current_position = (
                new_position
            )
            break

        if (
            time.time()
            - position_wait_start
            > 10.0
        ):
            print(
                "WARNING: Could not obtain "
                "initial position."
            )
            break

        time.sleep(0.1)

    if current_position is None:

        # Safe fallback.
        current_position = np.array([
            0.0,
            0.0,
            -TAKEOFF_ALTITUDE
        ])

        print(
            "Using fallback initial "
            "position."
        )

    start_position = (
        current_position.copy()
    )

    print()
    print("==========================================")
    print("INITIAL POSITION")
    print("==========================================")

    print(
        f"X = {current_position[0]:.3f}"
    )

    print(
        f"Y = {current_position[1]:.3f}"
    )

    print(
        f"Z = {current_position[2]:.3f}"
    )

    print()
    print(
        f"GOAL = "
        f"({GOAL_X:.2f}, "
        f"{GOAL_Y:.2f}, "
        f"{GOAL_Z_NED:.2f})"
    )

    print(
        "=========================================="
    )

    # ----------------------------------------------------------
    # Controller variables
    # ----------------------------------------------------------

    state = FORWARD

    avoidance_side = None

    avoid_steps = 0

    clear_steps = 0

    reverse_steps = 0

    position_history = deque(
        maxlen=STUCK_WINDOW_STEPS
    )

    position_history.append(
        current_position.copy()
    )

    path_length = 0.0

    previous_position = (
        current_position.copy()
    )

    start_time = time.time()

    goal_time = None

    control_steps = 0

    # ----------------------------------------------------------
    # Main control loop
    # ----------------------------------------------------------

    print()
    print("CONTROLLER STARTED")
    print()
    print(
        "Flow:"
    )
    print(
        "FORWARD -> AVOID -> RETURN -> FORWARD"
    )
    print(
        "                         ↓"
    )
    print(
        "                   GOAL_REACHED"
    )
    print()

    while True:

        loop_start = time.time()

        control_steps += 1

        elapsed_time = (
            time.time()
            - start_time
        )

        # ======================================================
        # UPDATE POSITION
        # ======================================================

        new_position = (
            get_local_position(master)
        )

        if new_position is not None:

            current_position = (
                new_position
            )

        # ------------------------------------------------------
        # IMPORTANT:
        # Never replace a valid position with None.
        # ------------------------------------------------------

        if current_position is None:

            print(
                "Waiting for valid position..."
            )

            stop_vehicle(master)

            time.sleep(
                CONTROL_PERIOD
            )

            continue

        # ======================================================
        # PATH LENGTH
        # ======================================================

        movement = distance_2d(
            previous_position,
            current_position
        )

        path_length += movement

        previous_position = (
            current_position.copy()
        )

        # ======================================================
        # POSITION HISTORY
        # ======================================================

        position_history.append(
            current_position.copy()
        )

        # ======================================================
        # YAW
        # ======================================================

        yaw = get_yaw(master)

        if yaw is None:

            # Keep last known yaw.
            # Initial yaw is 90 degrees.
            yaw = math.radians(90.0)

        # ======================================================
        # GOAL DISTANCE
        # ======================================================

        goal_distance = distance_2d(
            current_position,
            np.array([
                GOAL_X,
                GOAL_Y,
                current_position[2]
            ])
        )

        altitude_error = abs(
            float(current_position[2])
            - GOAL_Z_NED
        )

        # ======================================================
        # GOAL CHECK
        # ======================================================

        if (
            state != GOAL_REACHED
            and
            goal_reached(
                current_position
            )
        ):

            state = GOAL_REACHED

            goal_time = (
                time.time()
                - start_time
            )

            print()
            print(
                "=========================================="
            )
            print(
                "GOAL REACHED!"
            )
            print(
                f"Goal distance: "
                f"{goal_distance:.3f} m"
            )
            print(
                "=========================================="
            )

        # ======================================================
        # GOAL REACHED
        # ======================================================

        if state == GOAL_REACHED:

            stop_vehicle(master)

            print(
                f"[{control_steps:04d}] "
                f"STATE=GOAL_REACHED | "
                f"POS=("
                f"{current_position[0]:.2f}, "
                f"{current_position[1]:.2f}, "
                f"{current_position[2]:.2f}) | "
                f"GOAL_DIST="
                f"{goal_distance:.2f}"
            )

            # Hold position for a short time.
            if (
                goal_time is not None
                and
                time.time()
                - start_time
                >
                goal_time + 3.0
            ):
                break

            elapsed_loop = (
                time.time()
                - loop_start
            )

            sleep_time = (
                CONTROL_PERIOD
                - elapsed_loop
            )

            if sleep_time > 0:
                time.sleep(
                    sleep_time
                )

            continue

        # ======================================================
        # GOAL DIRECTION
        # ======================================================

        goal_direction, _ = (
            get_goal_direction(
                current_position
            )
        )

        goal_body_angle = (
            get_goal_body_angle(
                current_position,
                yaw
            )
        )

        # ======================================================
        # READ LIDAR
        # ======================================================

        distances = read_sensors(
            latest_scan,
            goal_body_angle
        )

        front_distance = (
            distances["FRONT"]
        )

        rear_distance = (
            distances["REAR"]
        )

        # ======================================================
        # SIDE SCORES
        # ======================================================

        left_score, right_score = (
            calculate_side_scores(
                distances
            )
        )

        # ======================================================
        # STUCK CHECK
        # ======================================================

        stuck = is_stuck(
            position_history
        )

        # ======================================================
        # DEBUG OUTPUT
        # ======================================================

        print(
            f"[{control_steps:04d}] "
            f"FL={distances['FAR_LEFT']:.2f} | "
            f"L={distances['LEFT']:.2f} | "
            f"F={front_distance:.2f} | "
            f"R={distances['RIGHT']:.2f} | "
            f"FR={distances['FAR_RIGHT']:.2f} | "
            f"REAR={rear_distance:.2f} | "
            f"GOAL={goal_distance:.2f} | "
            f"STATE={state}"
        )

        # ======================================================
        # EMERGENCY DISTANCE
        # ======================================================

        if (
            state != EMERGENCY_REVERSE
            and
            front_distance
            <= EMERGENCY_DISTANCE
            and
            state != GOAL_REACHED
        ):

            print()
            print(
                "!!! EMERGENCY OBSTACLE !!!"
            )
            print(
                "Switching to "
                "EMERGENCY_REVERSE"
            )

            state = (
                EMERGENCY_REVERSE
            )

            reverse_steps = 0

        # ======================================================
        # STATE: FORWARD
        # ======================================================

        if state == FORWARD:

            # --------------------------------------------------
            # Obstacle detected
            # --------------------------------------------------

            if (
                front_distance
                <= DETECTION_DISTANCE
            ):

                print()
                print(
                    "OBSTACLE DETECTED!"
                )

                avoidance_side, score = (
                    choose_side(
                        distances,
                        previous_side=
                        avoidance_side
                    )
                )

                print(
                    f"Choosing "
                    f"{avoidance_side} "
                    f"(score={score:.2f})"
                )

                state = AVOID

                avoid_steps = 0

                clear_steps = 0

            # --------------------------------------------------
            # Stuck
            # --------------------------------------------------

            elif stuck:

                print()
                print(
                    "STUCK DETECTED!"
                )

                avoidance_side, score = (
                    choose_side(
                        distances,
                        previous_side=
                        avoidance_side,
                        force_opposite=True
                    )
                )

                print(
                    f"Escape direction: "
                    f"{avoidance_side}"
                )

                state = (
                    EMERGENCY_REVERSE
                )

                reverse_steps = 0

            # --------------------------------------------------
            # Normal forward movement
            # --------------------------------------------------

            else:

                world_vx = (
                    goal_direction[0]
                    * FORWARD_SPEED
                )

                world_vy = (
                    goal_direction[1]
                    * FORWARD_SPEED
                )

                send_world_velocity(
                    master,
                    world_vx,
                    world_vy,
                    yaw
                )

        # ======================================================
        # STATE: AVOID
        # ======================================================

        elif state == AVOID:

            avoid_steps += 1

            # --------------------------------------------------
            # Check whether chosen side became blocked
            # --------------------------------------------------

            if avoidance_side == "LEFT":

                if (
                    distances["LEFT"]
                    < SIDE_BLOCKED
                ):

                    if (
                        distances["RIGHT"]
                        >
                        distances["LEFT"]
                    ):

                        print(
                            "Left side blocked."
                        )

                        print(
                            "Switching RIGHT."
                        )

                        avoidance_side = (
                            "RIGHT"
                        )

            elif avoidance_side == "RIGHT":

                if (
                    distances["RIGHT"]
                    < SIDE_BLOCKED
                ):

                    if (
                        distances["LEFT"]
                        >
                        distances["RIGHT"]
                    ):

                        print(
                            "Right side blocked."
                        )

                        print(
                            "Switching LEFT."
                        )

                        avoidance_side = (
                            "LEFT"
                        )

            # --------------------------------------------------
            # Forward component during avoidance
            # --------------------------------------------------

            forward_component = (
                goal_direction
                * AVOID_FORWARD_SPEED
            )

            # --------------------------------------------------
            # Lateral component
            # --------------------------------------------------

            if avoidance_side == "LEFT":

                # Left relative to goal direction.
                lateral_direction = np.array([
                    -goal_direction[1],
                    goal_direction[0]
                ])

            else:

                # Right relative to goal direction.
                lateral_direction = np.array([
                    goal_direction[1],
                    -goal_direction[0]
                ])

            lateral_component = (
                lateral_direction
                * SIDE_SPEED
            )

            world_velocity = (
                forward_component
                +
                lateral_component
            )

            send_world_velocity(
                master,
                world_velocity[0],
                world_velocity[1],
                yaw
            )

            # --------------------------------------------------
            # Check obstacle clearance
            # --------------------------------------------------

            if (
                front_distance
                >= CLEAR_DISTANCE
            ):

                clear_steps += 1

            else:

                clear_steps = 0

            # --------------------------------------------------
            # After minimum avoidance time and confirmed clear
            # --------------------------------------------------

            if (
                avoid_steps
                >= MIN_AVOID_STEPS
                and
                clear_steps
                >= CLEAR_CONFIRMATION_STEPS
            ):

                print()
                print(
                    "Obstacle cleared!"
                )

                print(
                    "Returning to goal path."
                )

                state = RETURN

                clear_steps = 0

        # ======================================================
        # STATE: RETURN
        # ======================================================

        elif state == RETURN:

            # --------------------------------------------------
            # Move toward goal
            # --------------------------------------------------

            world_vx = (
                goal_direction[0]
                * RETURN_SPEED
            )

            world_vy = (
                goal_direction[1]
                * RETURN_SPEED
            )

            send_world_velocity(
                master,
                world_vx,
                world_vy,
                yaw
            )

            # --------------------------------------------------
            # Check distance from original path
            # --------------------------------------------------

            lateral_error = (
                get_path_lateral_error(
                    start_position,
                    current_position
                )
            )

            if lateral_error <= 0.10:

                print(
                    "Returned to original "
                    "goal path."
                )

                state = FORWARD

                avoid_steps = 0

                clear_steps = 0

        # ======================================================
        # STATE: EMERGENCY REVERSE
        # ======================================================

        elif state == EMERGENCY_REVERSE:

            reverse_steps += 1

            # --------------------------------------------------
            # Reverse away from goal
            # --------------------------------------------------

            world_vx = (
                -goal_direction[0]
                * REVERSE_SPEED
            )

            world_vy = (
                -goal_direction[1]
                * REVERSE_SPEED
            )

            send_world_velocity(
                master,
                world_vx,
                world_vy,
                yaw
            )

            # --------------------------------------------------
            # Finish emergency escape
            # --------------------------------------------------

            if (
                reverse_steps
                >= ESCAPE_STEPS
            ):

                print()
                print(
                    "Emergency escape "
                    "complete."
                )

                avoidance_side, score = (
                    choose_side(
                        distances,
                        previous_side=
                        avoidance_side,
                        force_opposite=True
                    )
                )

                print(
                    f"Escape avoidance side: "
                    f"{avoidance_side}"
                )

                state = AVOID

                avoid_steps = 0

                clear_steps = 0

                reverse_steps = 0

        # ======================================================
        # TIME LIMIT
        # ======================================================

        if elapsed_time >= MAX_CONTROL_TIME:

            print()
            print(
                "TIME LIMIT REACHED."
            )

            break

        # ======================================================
        # LOOP TIMING
        # ======================================================

        elapsed_loop = (
            time.time()
            - loop_start
        )

        sleep_time = (
            CONTROL_PERIOD
            - elapsed_loop
        )

        if sleep_time > 0:

            time.sleep(
                sleep_time
            )

    # ==========================================================
    # FINAL STOP
    # ==========================================================

    stop_vehicle(master)

    # Give the vehicle several zero-velocity commands.
    for _ in range(10):

        stop_vehicle(master)

        time.sleep(0.1)

    # ==========================================================
    # FINAL RESULTS
    # ==========================================================

    total_time = (
        time.time()
        - start_time
    )

    print()
    print(
        "=========================================="
    )

    print(
        "FINAL RESULTS"
    )

    print(
        "=========================================="
    )

    if state == GOAL_REACHED:

        print(
            "RESULT: GOAL REACHED"
        )

    else:

        print(
            "RESULT: TIME LIMIT REACHED"
        )

    print(
        f"CONTROL STEPS: "
        f"{control_steps}"
    )

    if goal_time is not None:

        print(
            f"COMPLETION TIME: "
            f"{goal_time:.2f} s"
        )

    else:

        print(
            f"COMPLETION TIME: "
            f"{total_time:.2f} s"
        )

    print(
        f"PATH LENGTH: "
        f"{path_length:.3f} m"
    )

    print(
        f"GOAL DISTANCE: "
        f"{goal_distance:.3f} m"
    )

    if current_position is not None:

        print(
            f"FINAL POSITION: "
            f"("
            f"{current_position[0]:.3f}, "
            f"{current_position[1]:.3f}, "
            f"{current_position[2]:.3f}"
            f")"
        )

    else:

        print(
            "FINAL POSITION: unavailable"
        )

    print(
        f"FINAL STATE: "
        f"{state}"
    )

    print(
        "=========================================="
    )


# ==============================================================
# ENTRY POINT
# ==============================================================

if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print()
        print(
            "Controller interrupted by user."
        )

    except Exception as e:

        print()
        print(
            "Controller error:"
        )

        print(e)

        raise
