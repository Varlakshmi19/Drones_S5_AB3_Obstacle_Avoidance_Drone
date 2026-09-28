# MuJoCo Drone Obstacle Avoidance: Benchmark Performance Evaluation

Evaluation across 10 benchmark scenarios comparing obstacle avoidance strategies:
- **Reactive**: Finite State Machine with 6-sector virtual LiDAR rangefinding, 360° gap escape & vertical climb-over state machine, and closing-velocity dynamic prediction.
- **APF**: Artificial Potential Fields (gradient-based repulsive obstacle & attractive goal fields).
- **DWA**: Vectorized Dynamic Window Approach with forward trajectory arc sampling.
- **3D Avoidance**: 3D Volumetric avoidance with dynamic altitude climbing and overhang descent.

## Detailed Results Table

| Scenario | Scenario Name | Algorithm | Result | Flight Time (s) | Path Length (m) | Collisions | Min Clear (m) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 1 | Frontal Obstacle | REACTIVE | **SUCCESS** | 9.56 | 6.242 | 0 | 0.442 |
| 2 | Offset Left | REACTIVE | **SUCCESS** | 9.19 | 5.95 | 0 | 0.221 |
| 3 | Offset Right | REACTIVE | **SUCCESS** | 9.19 | 5.95 | 0 | 0.221 |
| 4 | Narrow Corridor | REACTIVE | **SUCCESS** | 9.15 | 6.04 | 3 | 0.22 |
| 5 | Dead-End U-Trap | REACTIVE | **SUCCESS** | 15.29 | 9.649 | 18 | 0.22 |
| 6 | Forest Maze | REACTIVE | **SUCCESS** | 9.65 | 6.59 | 0 | 0.389 |
| 7 | Dynamic Slider | REACTIVE | **SUCCESS** | 9.46 | 6.102 | 0 | 0.686 |
| 8 | 3D Cluttered | REACTIVE | **SUCCESS** | 9.96 | 6.613 | 0 | 0.378 |
| 9 | Dynamic Intruder | REACTIVE | **SUCCESS** | 9.62 | 6.087 | 0 | 0.719 |
| 10 | Dead-End Ring | REACTIVE | **SUCCESS** | 22.1 | 12.733 | 6 | 0.108 |
| 1 | Frontal Obstacle | SINDY_DWA | **SUCCESS** | 14.52 | 7.279 | 8 | 0.22 |
| 2 | Offset Left | SINDY_DWA | **SUCCESS** | 10.52 | 6.018 | 3 | 0.223 |
| 3 | Offset Right | SINDY_DWA | **SUCCESS** | 10.56 | 6.039 | 3 | 0.222 |
| 4 | Narrow Corridor | SINDY_DWA | **SUCCESS** | 9.52 | 5.759 | 0 | 0.23 |
| 5 | Dead-End U-Trap | SINDY_DWA | RUNNING | 60.0 | 3.416 | 2477 | 0.22 |
| 6 | Forest Maze | SINDY_DWA | **SUCCESS** | 15.58 | 6.387 | 7 | 0.22 |
| 7 | Dynamic Slider | SINDY_DWA | RUNNING | 60.0 | 34.375 | 2241 | 0.1 |
| 8 | 3D Cluttered | SINDY_DWA | **SUCCESS** | 14.58 | 6.613 | 4 | 0.22 |
| 9 | Dynamic Intruder | SINDY_DWA | **SUCCESS** | 9.81 | 5.762 | 0 | 0.385 |
| 10 | Dead-End Ring | SINDY_DWA | **SUCCESS** | 20.31 | 9.42 | 0 | 0.155 |
| 1 | Frontal Obstacle | APF | **SUCCESS** | 11.58 | 7.467 | 0 | 0.273 |
| 2 | Offset Left | APF | **SUCCESS** | 9.79 | 6.555 | 0 | 0.262 |
| 3 | Offset Right | APF | **SUCCESS** | 9.79 | 6.555 | 0 | 0.262 |
| 4 | Narrow Corridor | APF | RUNNING | 60.0 | 30.449 | 0 | 0.304 |
| 5 | Dead-End U-Trap | APF | RUNNING | 60.0 | 31.755 | 0 | 0.412 |
| 6 | Forest Maze | APF | **SUCCESS** | 18.77 | 11.34 | 0 | 0.265 |
| 7 | Dynamic Slider | APF | **SUCCESS** | 8.6 | 5.877 | 0 | 0.605 |
| 8 | 3D Cluttered | APF | RUNNING | 60.0 | 30.52 | 0 | 0.398 |
| 1 | Frontal Obstacle | DWA | **SUCCESS** | 16.19 | 7.653 | 10 | 0.221 |
| 2 | Offset Left | DWA | **SUCCESS** | 10.52 | 6.018 | 3 | 0.223 |
| 3 | Offset Right | DWA | **SUCCESS** | 10.56 | 6.039 | 3 | 0.222 |
| 4 | Narrow Corridor | DWA | **SUCCESS** | 9.52 | 5.759 | 0 | 0.23 |
| 5 | Dead-End U-Trap | DWA | RUNNING | 60.0 | 17.916 | 10 | 0.22 |
| 6 | Forest Maze | DWA | **SUCCESS** | 15.75 | 6.414 | 8 | 0.22 |
| 7 | Dynamic Slider | DWA | RUNNING | 60.0 | 34.339 | 2274 | 0.1 |
| 8 | 3D Cluttered | DWA | **SUCCESS** | 14.96 | 7.006 | 3 | 0.22 |
| 1 | Frontal Obstacle | 3D | RUNNING | 60.0 | 3.619 | 2603 | 0.105 |
| 2 | Offset Left | 3D | **SUCCESS** | 9.71 | 5.876 | 0 | 0.224 |
| 3 | Offset Right | 3D | **SUCCESS** | 9.71 | 5.876 | 0 | 0.224 |
| 4 | Narrow Corridor | 3D | **SUCCESS** | 9.9 | 5.861 | 0 | 0.22 |
| 5 | Dead-End U-Trap | 3D | RUNNING | 60.0 | 3.235 | 2544 | 0.22 |
| 6 | Forest Maze | 3D | **SUCCESS** | 22.06 | 14.077 | 33 | 0.22 |
| 7 | Dynamic Slider | 3D | RUNNING | 60.0 | 33.758 | 2521 | 0.101 |
| 8 | 3D Cluttered | 3D | RUNNING | 60.0 | 5.867 | 2678 | 0.22 |
| 10 | Ring Gap (seed 42) | REACTIVE | **SUCCESS** | 40.65 | 24.123 | 6 | 0.22 |
| 10 | Ring Gap (seed 100) | REACTIVE | **SUCCESS** | 46.71 | 27.019 | 6 | 0.22 |
| 10 | Ring Gap (seed 2023) | REACTIVE | **SUCCESS** | 37.17 | 22.483 | 6 | 0.22 |
| 10 | Ring Gap (seed 777) | REACTIVE | **SUCCESS** | 16.31 | 9.817 | 3 | 0.221 |
| 10 | Ring Gap (seed 999) | REACTIVE | **SUCCESS** | 34.27 | 20.311 | 6 | 0.22 |

## Performance Metrics by Algorithm

| Algorithm | Success Rate | Mean Flight Time (s) | Mean Path Length (m) | Total Collision Steps |
| :--- | :--- | :--- | :--- | :--- |
| **REACTIVE** | 100.0% (15/15) | 19.22 | 11.71 | 54 |
| **SINDY_DWA** | 80.0% (8/10) | 13.18 | 6.66 | 4743 |
| **APF** | 62.5% (5/8) | 11.71 | 7.56 | 0 |
| **DWA** | 75.0% (6/8) | 12.92 | 6.48 | 2311 |
| **3D** | 50.0% (4/8) | 12.84 | 7.92 | 10379 |
