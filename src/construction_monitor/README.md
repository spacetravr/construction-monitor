# Construction Monitor - ROS2 TurtleBot3

A ROS2 Humble project for monitoring construction site progress using TurtleBot3 robots with SLAM mapping capabilities.

## Overview

This project uses two TurtleBot3 Burger robots to scan a construction site. Each robot explores a designated zone (left/right) and creates a map using SLAM Toolbox. The individual maps are merged in real-time to create a complete view of the construction site.

**Future Goal:** Compare the scanned map with a reference blueprint to calculate construction progress percentage.

## Prerequisites

1. ROS2 Humble installed
2. TurtleBot3 packages:
   ```bash
   sudo apt install ros-humble-turtlebot3* ros-humble-slam-toolbox
   ```
3. Set TurtleBot3 model:
   ```bash
   echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
   source ~/.bashrc
   ```

## Project Structure

```
construction_monitor/
├── construction_monitor/          # Python nodes
│   ├── auto_explorer.py           # Single robot autonomous explorer
│   ├── auto_explorer_ns.py        # Namespaced explorer
│   ├── auto_explorer_zone.py      # Zone-based explorer (left/right)
│   ├── map_merger.py              # Merges maps from two robots
│   └── tf_relay.py                # TF relay utility (optional)
│
├── launch/
│   ├── construction_slam.launch.py      # Single robot SLAM
│   ├── two_robots.launch.py             # Two robots without SLAM
│   └── two_robots_slam.launch.py        # Two robots with SLAM (Step 9)
│
├── config/
│   ├── slam_params.yaml                 # SLAM Toolbox parameters
│   └── two_robots_slam.rviz             # RViz config for multi-robot
│
├── worlds/
│   ├── construction_site.world          # Complete construction site
│   └── construction_incomplete.world    # Incomplete site (for progress)
│
└── models/
    ├── robot1/model.sdf                 # Robot 1 with namespaced TF
    └── robot2/model.sdf                 # Robot 2 with namespaced TF
```

---

## Implementation Steps

### Step 1: Basic TurtleBot3 Setup
- Installed ROS2 Humble and TurtleBot3 packages
- Verified Gazebo simulation works with TurtleBot3 Burger
- Tested basic robot control with teleop_keyboard

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
```

---

### Step 2: Create Construction Site World
- Created custom Gazebo world file: `construction_site.world`
- World contains walls representing a construction site layout
- Size: approximately 10m x 10m area

**File:** `worlds/construction_site.world`

---

### Step 3: Single Robot SLAM Mapping
- Integrated SLAM Toolbox for map generation
- Robot creates occupancy grid map while exploring
- Map shows walls (100), free space (0), and unknown (-1)

```bash
ros2 launch construction_monitor construction_slam.launch.py
```

View map in RViz:
```bash
rviz2  # Add Map display, topic: /map
```

---

### Step 4: Autonomous Exploration
- Created `auto_explorer.py` node
- Robot autonomously navigates using laser scan data
- Avoids obstacles and explores unknown areas
- Uses simple reactive navigation (no path planning)

**Algorithm:**
1. Read laser scan data
2. If obstacle ahead: rotate to find clear path
3. If clear: move forward
4. Prefer unexplored directions

```bash
ros2 run construction_monitor auto_explorer
```

---

### Step 5: Save Map as Image
- Maps can be saved using map_saver from nav2_map_server
- Generates `.pgm` image and `.yaml` metadata file

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

---

### Step 6: Two Robot Simulation (No SLAM)
- Spawned two TurtleBot3 robots in Gazebo
- Each robot has its own namespace (`robot1`, `robot2`)
- Topics: `/robot1/cmd_vel`, `/robot2/cmd_vel`, etc.

```bash
ros2 launch construction_monitor two_robots.launch.py
```

---

### Step 7: Namespaced Explorer
- Created `auto_explorer_ns.py` for namespaced robots
- Takes `robot_namespace` parameter
- Subscribes/publishes to namespaced topics

```bash
ros2 run construction_monitor auto_explorer_ns --ros-args -p robot_namespace:=robot1
ros2 run construction_monitor auto_explorer_ns --ros-args -p robot_namespace:=robot2
```

---

### Step 8: Zone-Based Exploration
- Created `auto_explorer_zone.py`
- Robots assigned to specific zones: left or right
- Prevents overlap and improves efficiency
- Zone boundaries defined by x-coordinate

**Parameters:**
- `robot_namespace`: robot1 or robot2
- `zone`: left or right

```bash
ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot1 -p zone:=left
ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot2 -p zone:=right
```

---

### Step 9: Multi-Robot SLAM with Map Merging (CURRENT)

Two robots each running SLAM, with maps merged in real-time.

#### Architecture

**TF Tree Structure:**
```
world
  ├── robot1/map -> robot1/odom -> robot1/base_footprint -> robot1/base_scan
  └── robot2/map -> robot2/odom -> robot2/base_footprint -> robot2/base_scan
```

**Topics:**
| Topic | Description |
|-------|-------------|
| `/robot1/scan` | Laser scan from robot 1 |
| `/robot2/scan` | Laser scan from robot 2 |
| `/robot1/map` | SLAM map from robot 1 |
| `/robot2/map` | SLAM map from robot 2 |
| `/map` | MERGED map from both robots |
| `/robot1/cmd_vel` | Velocity commands for robot 1 |
| `/robot2/cmd_vel` | Velocity commands for robot 2 |

#### How to Run

**Terminal 1 - Launch simulation:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch construction_monitor two_robots_slam.launch.py
```

**Terminal 2 - Start Robot 1 explorer (left zone):**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot1 -p zone:=left
```

**Terminal 3 - Start Robot 2 explorer (right zone):**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot2 -p zone:=right
```

**Terminal 4 - Open RViz:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/install/construction_monitor/share/construction_monitor/config/two_robots_slam.rviz
```

#### What You Should See

- **Gazebo:** Two TurtleBot3 robots in a construction site
- **RViz:**
  - Gray/white areas = explored free space
  - Black lines = walls detected
  - Red dots = Robot 1 laser scan
  - Blue dots = Robot 2 laser scan
  - TF frames showing robot positions
  - Robot models (if TF is working)

#### Key Files
- `launch/two_robots_slam.launch.py` - Main launch file
- `construction_monitor/map_merger.py` - Combines robot maps
- `models/robot1/model.sdf` - Robot 1 with namespaced TF
- `models/robot2/model.sdf` - Robot 2 with namespaced TF
- `config/two_robots_slam.rviz` - RViz configuration

#### Map Merger Logic

The `map_merger` node combines maps using this priority:
1. **Wall (100)** - wins over everything
2. **Free space (0)** - wins over unknown
3. **Unknown (-1)** - default

---

### Step 10: Real-Time Exploration Percentage (PENDING)

**Goal:** Display how much of the construction site has been explored.

**Planned features:**
- Calculate explored percentage from map data
- Display in terminal with live updates
- Optionally overlay on RViz

**Formula:**
```
explored% = (free_cells + wall_cells) / total_cells * 100
```

---

### Step 11: Construction Progress Comparison (PENDING)

**Goal:** Compare scanned map with reference blueprint to show construction progress.

**Planned features:**
- Load reference blueprint image (complete construction site)
- Align scanned map with blueprint
- Compare wall positions
- Calculate construction completion percentage
- Highlight missing/incomplete structures
- Visual diff display

---

## Technical Details

### Multi-Robot TF Solution

The main challenge in multi-robot SLAM is TF frame conflicts. This project solves it using **NAMESPACED FRAME PREFIXES**:

1. **Custom SDF models** define namespaced TF frames:
   - `robot1/odom`, `robot1/base_footprint`, `robot1/base_scan`
   - `robot2/odom`, `robot2/base_footprint`, `robot2/base_scan`

2. **SLAM Toolbox** runs in each robot's namespace with its own map frame:
   - Robot 1: `map_frame = 'robot1/map'`
   - Robot 2: `map_frame = 'robot2/map'`

3. **Static TF publishers** connect both map frames to 'world':
   - `world -> robot1/map` (identity transform)
   - `world -> robot2/map` (identity transform)

4. **Map merger** combines both maps and publishes to `/map` with `frame_id='world'`

### QoS Settings

SLAM Toolbox publishes maps with:
- **Durability:** TRANSIENT_LOCAL
- **Reliability:** RELIABLE

> All map subscribers must match these QoS settings or they won't receive data.

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "No map received" in RViz | Check QoS settings. Maps use TRANSIENT_LOCAL durability. |
| "Frame [xxx] does not exist" in RViz | Change Fixed Frame to 'world' in Global Options |
| Robots not moving | Make sure `auto_explorer_zone` is running for each robot |
| Only one robot's map showing | Both robots need to move for SLAM to create maps |
| Map looks strange or misaligned | The two robot maps may have different origins. This is expected when robots start in different positions. |

### Useful Commands

```bash
# Check if maps are being published
ros2 topic echo /robot1/map --once
ros2 topic echo /robot2/map --once
ros2 topic echo /map --once

# Check TF tree
ros2 run tf2_tools view_frames

# Check topic info and QoS
ros2 topic info /robot1/map -v

# List all topics
ros2 topic list

# Manual robot control
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel

# Check running nodes
ros2 node list
```

---

## Build & Install

```bash
cd ~/ros2_ws
colcon build --packages-select construction_monitor
source install/setup.bash
```

---

## Quick Start

```bash
# 1. Build the package
cd ~/ros2_ws
colcon build --packages-select construction_monitor
source install/setup.bash

# 2. Launch two-robot SLAM
ros2 launch construction_monitor two_robots_slam.launch.py

# 3. Start explorers (in separate terminals)
ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot1 -p zone:=left
ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot2 -p zone:=right

# 4. View in RViz
rviz2 -d ~/ros2_ws/install/construction_monitor/share/construction_monitor/config/two_robots_slam.rviz

# 5. Watch the robots explore and the merged map grow!
```

---

## License

Apache-2.0
