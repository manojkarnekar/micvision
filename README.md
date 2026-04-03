# micvision — ROS 2 Port

A ROS 2 conversion of the [micvision](https://github.com/tyuownu/micvision) localization package originally written for ROS 1. This package provides **global initial pose estimation** for a robot by matching a live LiDAR scan against a known occupancy grid map — without requiring any prior pose guess.

---

## Overview

When a robot boots up or gets lost, it needs to figure out where it is on the map. This package solves that by exhaustively scoring candidate positions and orientations across the entire map using the current laser scan, then publishing the best-match pose as a `PoseWithCovarianceStamped` message (the standard input for AMCL and Nav2).

The pipeline runs in three stages:

1. **Map acquisition & inflation** — fetch the static occupancy grid, inflate obstacles by a configurable radius to create a cost layer
2. **Laser scan processing** — convert the raw scan into a point cloud, then pre-rotate it across 360° to generate one sample per candidate angle
3. **Exhaustive pose search** — slide every rotated scan sample across every candidate cell in the map and score the match; publish the best result

---

## File Structure

```
micvision/
├── main.cpp               # Node entry point
├── map_calculation.cpp    # Map fetch, obstacle inflation, cost lookup
├── initial_pose.cpp       # Bresenham ray tracing, pose search, result publish
└── laser_calculation.cpp  # Scan callback, point cloud rotation, line-of-sight validation
```

### `main.cpp`

Entry point. Initialises the ROS 2 node, calls `getMap()` to fetch the static map from the map server, then `inflateMap()` to build the cost layer before spinning.

### `map_calculation.cpp`

| Function | Purpose |
|---|---|
| `getMap()` | Calls `/map_server/map` service and stores the occupancy grid |
| `computeCaches()` | Pre-computes distance and cost lookup tables up to `cell_inflation_radius_` |
| `inflateMap()` | BFS-style inflation pass over all obstacle cells; builds `inflated_map_data_` — a vector of `(is_free, normalised_cost)` pairs |
| `enqueueObstacle()` | Pushes a cell into the inflation priority queue if within radius |
| `distanceLookup()` / `costLookup()` | O(1) table reads for Euclidean distance and inflated cost |

### `laser_calculation.cpp`

| Function | Purpose |
|---|---|
| `scanCallback()` | Converts incoming `LaserScan` ranges into a 3-D Cartesian point cloud (XY plane) |
| `handleLaserScan()` | Rotates the point cloud from −180° to +180° in `laserscan_anglar_step_` increments, producing one `LaserScanSample` per angle |
| `validPosition()` | Uses Bresenham ray tracing to check whether the line between the robot candidate cell and a scan endpoint passes through free space (line-of-sight check) |
| `scoreASample()` | Scores a single `(sample, u, v)` candidate using raw map data |

### `initial_pose.cpp`

| Function | Purpose |
|---|---|
| `bresenham()` | Returns all grid cells along a line between two pixels; bounds-checked against map edges |
| `transformPointCloud()` | Applies a quaternion rotation to the point cloud and converts to pixel indices |
| `init_pose_callback()` | Main search loop — iterates over all `(u, v, angle)` candidates, applies quick pre-filter, scores matches, and publishes the best pose |

---

## Algorithm Detail

### Map Inflation

Obstacles are expanded by `inflation_radius_` (metres) using a priority-queue BFS. Each cell's cost is computed as:

```
cost = (1 - distance / cell_inflation_radius_) * cost_obstacle_
```

This creates a smooth cost gradient around obstacles, which improves the scoring signal during pose search.

### Scan Sampling

The raw point cloud is pre-rotated at every integer multiple of `laserscan_anglar_step_` degrees across the full 360°. Each rotated version is stored as a `LaserScanSample` containing:
- Pixel coordinates of each point
- Flat map indices (for fast array access during scoring)
- Bounding box (`min_x`, `max_x`, `min_y`, `max_y`) for quick out-of-bounds rejection

### Pose Search

The search iterates over the map at stride `range_step_` and skips cells that are occupied. For each free cell and each pre-rotated scan sample:

1. **Bounds check** — skip if the sample bounding box falls outside the map
2. **Quick score pre-filter** (if `quick_score_` is enabled) — subsample `quick_score_num_` points; skip if fewer than half hit occupied cells
3. **Full score** — accumulate normalised cost over all scan points; update best if this exceeds the running maximum
4. **Line-of-sight validation** — Bresenham check on the subsampled points to reject poses where scan endpoints are occluded by walls

The best `(u, v, angle)` is converted to world coordinates and published as a `PoseWithCovarianceStamped` on the standard `/initialpose` topic.

---

## Key Parameters

| Parameter | Description |
|---|---|
| `inflation_radius_` | Obstacle inflation radius in metres |
| `cost_obstacle_` | Maximum cost value assigned to obstacle cells |
| `range_step_` | Stride (in cells) for the exhaustive map search — larger = faster but coarser |
| `laserscan_anglar_step_` | Angular resolution (degrees) for scan rotation samples |
| `laserscan_circle_step_` | Downsampling stride over laser scan rays |
| `quick_score_num_` | Number of points used in the quick pre-filter pass |
| `quick_score_` | Enable/disable the quick pre-filter |
| `min_valid_range_` / `max_valid_range_` | Valid range bounds for laser returns |

---

## ROS 2 Interface

### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Live laser scan |
| `/odom` | `nav_msgs/Odometry` | Used to detect high angular velocity (skips scoring during fast rotation) |

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Best-estimate global pose |

### Services Called

| Service | Type | Description |
|---|---|---|
| `/map_server/map` | `nav_msgs/srv/GetMap` | Fetches the static occupancy grid |

---

## ROS 1 → ROS 2 Migration Notes

The core algorithms are unchanged from the original. Key API changes made during the port:

- `ros::NodeHandle` → `rclcpp::Node` with `rclcpp::init` / `rclcpp::spin`
- `ros::Publisher` / `ros::Subscriber` → `rclcpp::Publisher` / `rclcpp::Subscription`
- `ros::ServiceClient` → `rclcpp::Client` with `async_send_request` + `spin_until_future_complete`
- `ROS_INFO` / `ROS_ERROR` → `RCLCPP_INFO` / `RCLCPP_ERROR`
- `ros::Time::now()` → `this->get_clock()->now()`
- Message headers use `rclcpp::Time` instead of `ros::Time`
- Service result access via `.get()->map` instead of dereferencing a shared pointer directly

---

## Dependencies

- ROS 2 (Humble or later recommended)
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `tf2` / `tf2_ros`
- Eigen3

---

## Building

```bash
cd ~/ros2_ws/src
# place the micvision package here
cd ~/ros2_ws
colcon build --packages-select micvision
source install/setup.bash
```

## Running

Ensure your map server is running and publishing on `/map_server/map`, then:

```bash
ros2 run micvision localization
```

The node will fetch the map, inflate it, and wait for a laser scan. Once a scan arrives and `init_pose_callback` is triggered, it will publish the estimated initial pose and shut down.

---

## Credits

Original ROS 1 implementation: [tyuownu/micvision](https://github.com/tyuownu/micvision)  
ROS 2 port: Manoj