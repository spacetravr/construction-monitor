# Construction Monitor - ROS2 Multi-Robot SLAM

A ROS2 Humble project for monitoring construction site progress using two TurtleBot3 robots with autonomous exploration, SLAM mapping, and construction progress calculation.

## Overview

This project deploys two TurtleBot3 Burger robots to autonomously scan a construction site. Each robot explores a designated zone (left/right) using SLAM Toolbox. The individual maps are merged in real-time, and construction progress is calculated by comparing against a reference blueprint.

## Features

- **Multi-Robot SLAM**: Two robots with namespaced TF frames for conflict-free mapping
- **Zone-Based Exploration**: Left robot (x < 0) and right robot (x > 0) with boundary detection
- **Real-Time Map Merging**: Combines maps from both robots into unified `/map`
- **Construction Progress**: Compares scanned map with blueprint to calculate completion %
- **Collision Avoidance**: TF-based robot-to-robot collision detection and avoidance
- **Auto-Save**: Automatically saves map when exploration threshold is reached

## Prerequisites

1. ROS2 Humble installed
2. TurtleBot3 packages:
   ```bash
   sudo apt install ros-humble-turtlebot3* ros-humble-slam-toolbox ros-humble-nav2-map-server
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
│   ├── auto_explorer_zone.py      # Zone-based autonomous explorer
│   ├── map_merger.py              # Merges maps from two robots
│   └── construction_progress.py   # Compares map with blueprint
│
├── launch/
│   └── two_robots_slam.launch.py  # Main launch file
│
├── config/
│   └── two_robots_slam.rviz       # RViz visualization config
│
├── worlds/
│   ├── my_house_small.world       # Complete construction (15m x 15m)
│   └── my_house_small_70.world    # 70% complete construction
│
└── models/
    ├── robot1/model.sdf           # Robot 1 (red) with namespaced TF
    └── robot2/model.sdf           # Robot 2 (blue) with namespaced TF
```

## Usage

### Step 1: Launch Simulation with SLAM

```bash
# For complete world (to create blueprint)
CONSTRUCTION_WORLD=my_house_small ros2 launch construction_monitor two_robots_slam.launch.py

# For 70% incomplete world (to measure progress)
CONSTRUCTION_WORLD=my_house_small_70 ros2 launch construction_monitor two_robots_slam.launch.py
```

### Step 2: Start Autonomous Explorers (in separate terminals)

```bash
# Robot 1 - explores left zone (x < 0)
ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot1 -p zone:=left

# Robot 2 - explores right zone (x > 0)
ros2 run construction_monitor auto_explorer_zone --ros-args -p robot_namespace:=robot2 -p zone:=right
```

### Step 3: Visualize in RViz

```bash
rviz2 -d ~/ros2_ws/install/construction_monitor/share/construction_monitor/config/two_robots_slam.rviz
```

### Step 4: Calculate Construction Progress

```bash
# Compare live scan with saved blueprint
ros2 run construction_monitor construction_progress --ros-args -p blueprint_map:=~/construction_maps/blueprint_complete.yaml
```

## Workflow

1. **Create Blueprint**: Run robots in complete world (`my_house_small`), map is auto-saved to `~/construction_maps/`
2. **Scan Incomplete Site**: Run robots in incomplete world (`my_house_small_70`)
3. **Compare**: Run `construction_progress` node to see completion percentage

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robot1/map` | OccupancyGrid | Map from robot1's SLAM |
| `/robot2/map` | OccupancyGrid | Map from robot2's SLAM |
| `/map` | OccupancyGrid | Merged map from both robots |
| `/construction_progress` | Float32 | Construction completion percentage |
| `/exploration_complete` | Bool | Signal when exploration threshold reached |

## TF Tree

```
world
├── robot1/map -> robot1/odom -> robot1/base_footprint -> robot1/base_scan
└── robot2/map -> robot2/odom -> robot2/base_footprint -> robot2/base_scan
```

## Parameters

### auto_explorer_zone
- `robot_namespace`: Robot namespace (robot1 or robot2)
- `zone`: Exploration zone (left or right)
- `obstacle_distance`: Distance to trigger obstacle avoidance (default: 0.4m)

### map_merger
- `world_size`: World size in meters for % calculation (default: 10.0)
- `save_threshold`: Auto-save at this exploration % (default: 95.0)
- `save_path`: Directory to save maps (default: ~/construction_maps)

### construction_progress
- `blueprint_map`: Path to blueprint .yaml file
- `update_interval`: How often to calculate progress (default: 5.0s)

## License

Apache-2.0
