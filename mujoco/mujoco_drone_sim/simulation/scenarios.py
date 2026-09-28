"""
Scenario definitions and metadata for MuJoCo Drone Obstacle Avoidance Simulation.
Covers 8 benchmark scenarios including the original 6 from the base paper.
"""

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


def get_scenario_info(scenario_id):
    """Retrieve metadata dictionary for a given scenario ID."""
    return SCENARIOS.get(scenario_id, SCENARIOS[1])
