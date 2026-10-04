# Dynamic Window Frontier Exploration (`roomwatch_explore`)

The `roomwatch_explore` package is a faithful ROS 2 Humble port of the **dynamic search window frontier exploration** package from `fetch_ws/src/dynamic-window-frontier-exploration/explore/`. It provides autonomous frontier-based exploration for mobile manipulators (Fetch-style robots) using `rclcpp`, `nav2_costmap_2d`, and Nav2's `NavigateToPose` action interface.

---

## 1. Architecture Overview

The package consists of three primary C++ components and Python launch/utility interfaces:

1. **`Costmap2DClient`** (`include/roomwatch_explore/costmap_client.hpp`, `src/costmap_client.cpp`):
   - Subscribes to the global map/costmap topic (`nav_msgs/msg/OccupancyGrid`, default `/map`) using `transient_local` reliability per `docs/CONTRACT.md`.
   - Subscribes to incremental map updates (`map_msgs/msg/OccupancyGridUpdate`, default `/map_updates`).
   - Translates raw occupancy grid cell costs (-1 to 100) into `nav2_costmap_2d` cost values (`FREE_SPACE = 0`, `INSCRIBED = 253`, `LETHAL_OBSTACLE = 254`, `NO_INFORMATION = 255`).
   - Maintains a thread-safe `nav2_costmap_2d::Costmap2D` instance protected by recursive mutex locks.
   - Tracks the robot's pose in the global frame using `tf2_ros::Buffer`.

2. **`FrontierSearch`** (`include/roomwatch_explore/frontier_search.hpp`, `src/frontier_search.cpp`):
   - Performs a Breadth-First Search (BFS) starting from the robot's current position across all reachable free cells.
   - Identifies candidate frontier cells (`NO_INFORMATION` cells adjacent in 4-neighborhood to `FREE_SPACE`).
   - Clusters contiguous frontier cells using 8-connected BFS into distinct `Frontier` objects.
   - Filters out frontiers smaller than `min_frontier_size` meters.
   - Enforces `min_frontier_spacing` to prevent redundant goal clustering.
   - Computes frontier costs and sorts them in ascending order (best candidate first).

3. **`Explore` Node** (`include/roomwatch_explore/explore.hpp`, `src/explore.cpp`, `src/explore_node.cpp`):
   - Orchestrates the periodic exploration loop driven by `planner_frequency`.
   - Implements the dynamic search window logic (local filtering vs. global expansion).
   - Manages goal dispatching via `rclcpp_action::Client<nav2_msgs::action::NavigateToPose>`.
   - Monitors navigation progress against `progress_timeout` and manages goal blacklisting.
   - Detects termination conditions (`frontiers.size() < min_global_frontiers` or all reachable frontiers blacklisted).
   - Automatically saves `map.pgm` and `map.yaml` into `data/<run>/` per `docs/CONTRACT.md` via `nav2_map_server::saveMapToFile` with direct PGM/YAML fallback.
   - Publishes termination events on `explore/exploration_termination` and `exploration_termination`.

---

## 2. Dynamic Search Window Algorithm

The dynamic search window mechanism balances localized, systematic room exploration against global map traversal, reducing unnecessary long-distance travel and preventing the robot from thrashing between distant regions.

```
                      +-----------------------------+
                      |   search_.searchFrom(pos)   |
                      |  Extract all map frontiers  |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | frontiers < min_global?     | ---- Yes ----> [ Terminate & Save Map ]
                      +--------------+--------------+
                                     | No
                                     v
                      +-----------------------------+
                      | Filter frontiers within     |
                      | local_frontier_filter_radius|
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | local_count <= min_local?   |
                      +--------------+--------------+
                             /               \
                    Yes     /                 \  No
                           v                   v
      +----------------------------+   +----------------------------+
      | Expand window to global:   |   | Retain local search window |
      | radius = global_filter_rad |   | radius = local_filter_rad  |
      | candidates = all frontiers |   | candidates = local set     |
      +--------------+-------------+   +--------------+-------------+
                     \                               /
                      \                             /
                       v                           v
                      +-----------------------------+
                      | Select best frontier not on |
                      |      frontier_blacklist     |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | Dispatch NavigateToPose     |
                      | Track distance & timeout    |
                      +-----------------------------+
```

### 2.1 Frontier Extraction and Scoring
1. **BFS Discovery**: Search expands outwards from the clear cell closest to the robot. For each cell where `map[nbr] <= map[idx]`, the cell is traversed. When a `NO_INFORMATION` cell with at least one 4-connected `FREE_SPACE` neighbor is met, it is flagged as the seed of a new frontier.
2. **Cluster Grouping**: All 8-connected frontier cells are grouped. The frontier's size, world-coordinate centroid, closest point (`middle`), and minimum Euclidean distance to the robot are calculated.
3. **Spacing and Size Filtering**: Clusters whose physical length (`size * resolution`) is below `min_frontier_size` are discarded. If a new frontier centroid is within `min_frontier_spacing` of an existing frontier centroid, it is merged/skipped.
4. **Cost Formulation**:
   $$\text{cost} = (\text{potential\_scale} \cdot \text{min\_distance} \cdot \text{res}) - (\text{gain\_scale} \cdot \text{size} \cdot \text{res})$$
   - Frontiers closer to the robot receive a lower cost (favored).
   - Larger frontiers provide higher information gain, decreasing cost (favored).
   - Frontiers are sorted ascending by cost.

### 2.2 Window Growth Logic
- **Local Window Filtering**:
  Candidates within Euclidean distance $r \le \text{local\_frontier\_filter\_radius}$ from the robot pose are gathered into `filtered_frontiers`.
- **Dynamic Expansion**:
  - If $\text{filtered\_frontiers.size()} \le \text{min\_local\_frontiers}$:
    The robot detects that local exploration opportunities are depleted. The search window expands to `global_frontier_filter_radius` (covering the full map), and all extracted frontiers are made available.
  - Else:
    The search window remains local (`local_frontier_filter_radius`), keeping the robot focused on finishing the current room/hallway before venturing elsewhere.

### 2.3 Blacklist & Fallback
- If all frontiers in the local window are blacklisted, the algorithm dynamically falls back to evaluate global frontiers before concluding that exploration has stalled.
- If all frontiers across the global map are blacklisted or exhausted, exploration cleanly terminates.

### 2.4 Progress Tracking & Preemption
- As the robot pursues a target centroid:
  - If the target centroid changes or the minimum distance to the target decreases, progress is registered (`last_progress = now()`).
  - If elapsed time exceeds `progress_timeout` without distance reduction, the robot is marked as stuck, the target is appended to `frontier_blacklist`, and replanning triggers immediately.
  - If a navigation goal receives `ABORTED` from Nav2, it is immediately blacklisted.

### 2.5 Termination & Map Saving
Upon termination (either $\text{total\_frontiers} < \text{min\_global\_frontiers}$ or exhaustion of reachable frontiers):
1. An exploration termination signal (`std_msgs/msg/Bool` with `data = true`) is published on `explore/exploration_termination` and `exploration_termination`.
2. Active navigation goals are canceled and the planning timer is stopped.
3. If `save_map` is true, the current occupancy grid map is written to `<run_dir>/map.pgm` and `<run_dir>/map.yaml` (defaulting to `data/<YYYY-MM-DD_HH-mm-ss>/map.*` per `docs/CONTRACT.md`).

---

## 3. Configuration Parameters

All parameters from the original ROS 1 package are maintained with identical behavior and defaults:

| Parameter | Type | Default | Launch Default | Description |
|---|---|---|---|---|
| `robot_base_frame` | string | `"base_link"` | `"base_link"` | Base TF frame of the robot. |
| `costmap_topic` | string | `"map"` | `"map"` | Topic name for `OccupancyGrid`. |
| `costmap_updates_topic` | string | `"map_updates"` | `"map_updates"` | Topic name for partial map updates. |
| `visualize` | bool | `true` | `true` | Publish `MarkerArray` on `/frontiers`. |
| `planner_frequency` | double | `1.0` | `0.15` | Exploration planning frequency (Hz). |
| `progress_timeout` | double | `30.0` | `20.0` | Progress timeout before blacklisting (s). |
| `potential_scale` | double | `1e-3` | `5.0` | Cost multiplier for distance to frontier. |
| `orientation_scale` | double | `0.0` | `0.0` | Cost multiplier for orientation (reserved). |
| `gain_scale` | double | `1.0` | `1.0` | Cost multiplier for frontier size. |
| `min_frontier_size` | double | `0.5` | `2.0` | Minimum frontier length in meters. |
| `local_frontier_filter_radius` | double | `5.0` | `3.0` | Radius of local search window (m). |
| `min_local_frontiers` | double | `5.0` | `2.0` | Minimum local frontiers before expanding search window. |
| `min_global_frontiers` | double | `5.0` | `1.0` | Frontier count below which exploration terminates. |
| `global_frontier_filter_radius` | double | `5.0` | `30.0` | Radius of global search window (m). |
| `min_frontier_spacing` | double | `2.0` | `1.5` | Minimum distance between frontier centroids (m). |
| `transform_tolerance` | double | `0.3` | `0.3` | TF transform lookup tolerance (s). |
| `save_map` | bool | `true` | `true` | Trigger saving map to disk on completion. |
| `run_dir` | string | `""` | `""` | Target run directory (`data/<run>`); auto-generated timestamp if empty. |
| `data_dir` | string | `"data"` | `"data"` | Base data directory when `run_dir` is not explicitly set. |

---

## 4. Verification and Testing

### 4.1 Unit Tests
Run unit tests verifying costmap tools and frontier search:
```bash
source /opt/ros/humble/setup.bash
source ws/install/setup.bash
colcon test --packages-select roomwatch_explore && colcon test-result --verbose
```
Covers:
- `EuclideanDistance`: Euclidean point calculations.
- `Neighborhood4And8`: 4-connected and 8-connected grid boundary and neighbor indexing.
- `NearestCell`: BFS discovery of values.
- `AllUnknownMapHasNoFrontiers`: Asserts 0 frontiers on completely unexplored map.
- `FreeSpacePatchDetectsFrontiers`: Asserts correct frontier clustering on free-unknown boundaries.
- `MinFrontierSizeFilter`: Asserts rejection of clusters smaller than threshold.
- `FrontierCostSorting`: Asserts correct ranking by cost formula.

### 4.2 Integration Test with Mock Robot
Test against the contract mock robot (`tests/mock_robot/mock_robot.py`) on `ROS_DOMAIN_ID=15`:
```bash
python3 tests/explore/test_mock_robot_explore.py
```
Verifies map ingestion, TF lookup, action server connection, termination on fully mapped rooms, and output of `map.pgm` and `map.yaml` in `data/<run>/`.

### 4.3 Dynamic Growing Map Test
Test against a dynamic map simulating laser clearing and obstacle frontiers:
```bash
python3 tests/explore/test_growing_map_explore.py
```
Verifies multi-goal navigation, search window expansion, progress timeouts, frontier clearing, and final map serialization.
