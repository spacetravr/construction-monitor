# TurtleBot3 Construction Site Progress Monitoring

## Project Overview
This project uses two TurtleBot3 robots to map an incomplete construction site, compare it with a complete reference map, and calculate construction progress percentage.

## Project Goals
- Two TurtleBot3 robots navigate different areas of a construction site
- Robots use SLAM to create maps of the incomplete site
- Compare incomplete map with complete reference map
- Calculate construction progress percentage

---

## Progress Log

### Session: 2025-12-09

#### Step 1: Install Required Packages ✅
**Commands executed:**
```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot3 \
                    ros-humble-turtlebot3-gazebo \
                    ros-humble-turtlebot3-cartographer \
                    ros-humble-turtlebot3-navigation2 \
                    ros-humble-turtlebot3-teleop \
                    ros-humble-slam-toolbox
```

**Verification:**
```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep turtlebot3
ros2 pkg list | grep slam_toolbox
```

**Result:** All packages installed successfully.

**Notes:**
- Decided to use SLAM Toolbox instead of Cartographer for better multi-robot support
- Both packages installed for flexibility

---

#### Step 2: Test Single Robot in Empty Gazebo World ✅
**Commands executed:**
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Issues encountered:**
- Error: "Service /spawn_entity unavailable"
- **Solution:** Added `source /usr/share/gazebo/setup.bash` to load Gazebo ROS plugins

**Result:** Robot successfully spawned in Gazebo empty world.

**Environment setup added to ~/.bashrc:**
```bash
export TURTLEBOT3_MODEL=burger
```

---

#### Step 3: Test Teleop Control ✅
**Commands executed:**
```bash
# Terminal 1: Gazebo running
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2: Teleop control
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Result:** Robot successfully controlled via keyboard (w, a, s, d, x keys).

---

#### Step 4: Create Simple Construction Site World in Gazebo ✅
**Commands executed:**
```bash
# Create ROS2 package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python construction_monitor

# Create directories
cd construction_monitor
mkdir -p worlds launch maps config scripts

# Build the package
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select construction_monitor
```

**Files created:**
- [construction_complete.world](src/construction_monitor/worlds/construction_complete.world) - Complete construction site with all walls
- [construction_incomplete.world](src/construction_monitor/worlds/construction_incomplete.world) - Incomplete site (missing some walls)
- [construction_world.launch.py](src/construction_monitor/launch/construction_world.launch.py) - Launch file for the world

**World design:**
- Outer walls (10x10 meters) - represents site boundary
- Room 1: Complete version has 2 walls, Incomplete has only 1 wall
- Room 2: Complete version has 2 walls, Incomplete has 0 walls
- This gives us a clear difference to detect with mapping

**To test the world:**
```bash
export TURTLEBOT3_MODEL=burger
source ~/ros2_ws/install/setup.bash
source /usr/share/gazebo/setup.bash
ros2 launch construction_monitor construction_world.launch.py
```

**Issues encountered:**
- Error: Launch file had incorrect path to gzserver.launch.py
- **Solution:** Changed from `turtlebot3_gazebo_dir` to `gazebo_ros_dir` for gzserver and gzclient launch files

**Result:** Custom construction site world created and tested successfully! Robot spawns in incomplete construction site.

---

#### Step 5: Test SLAM Mapping with One Robot ✅
**What we did:**
- Created autonomous explorer node to automatically map the construction site
- Robot explores using obstacle avoidance (no manual control needed)
- SLAM Toolbox creates map as robot explores
- RViz2 visualizes the map being created in real-time

**Files created:**
- [auto_explorer.py](src/construction_monitor/construction_monitor/auto_explorer.py) - Autonomous exploration node with obstacle avoidance

**How auto_explorer.py was created:**

1. **File Location:**
   - Path: `/home/ezis/ros2_ws/src/construction_monitor/construction_monitor/auto_explorer.py`
   - This is the standard Python module location for ROS2 packages

2. **Code Structure (Final Optimized Version with Frontier Detection):**
   ```python
   class AutoExplorer(Node):
       def __init__(self):
           # Publisher: sends movement commands to robot
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

           # Subscriber: receives laser scan data for obstacle detection
           self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

           # Subscriber: receives map data for frontier detection
           self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

           # Robot states: FORWARD, TURN_LEFT, TURN_RIGHT (NO BACKUP STATE)
           self.state = 'FORWARD'

           # Parameters (OPTIMIZED FOR NARROW PATH NAVIGATION)
           self.obstacle_distance = 1.0   # React at 1.0m (allows narrow paths)
           self.linear_speed = 0.12       # Slower speed for safety in tight spaces
           self.angular_speed = 1.0       # Fast turning

           # Frontier exploration (SMART - turns toward unexplored areas)
           self.forward_counter = 0
           self.max_forward_time = 150    # Turn every ~15 seconds (explore more before turning)
           self.frontier_direction = None # LEFT, RIGHT, or None (from map analysis)
   ```

3. **Obstacle Avoidance Logic (OPTIMIZED FOR NARROW PATHS):**
   - **Step 1:** Analyze laser scan data
     - **Front sector:** Indices 315-360 and 0-45 (90° centered at 0° = FRONT)
       - Index 0 = FRONT (along +X axis, confirmed by official TurtleBot3 docs)
       - Covers -45° to +45° from forward direction
     - **Left sector:** Indices 45-135 (90° on left side)
     - **Right sector:** Indices 225-315 (90° on right side)

   - **Step 2:** Filter invalid readings (infinity, NaN)

   - **Step 3:** Calculate distances
     - `min_front_distance` = minimum distance directly in front
     - `avg_left_dist` = average distance on left side
     - `avg_right_dist` = average distance on right side

   - **Step 4:** Make intelligent decisions
     - **EMERGENCY STOP:** If `min_front_distance < 0.3m` → Stop immediately + 180° turn
     - If `min_front_distance < 1.0m` → Obstacle detected! (smaller distance allows narrow paths)
       - **Corner detection:** If left AND right < 0.6m → 180° turn to escape
       - **Normal obstacle:** Turn 45° toward side with more space (CONSISTENT)
     - If moving straight for 15+ seconds → Turn 20° toward unexplored area (frontier detection)
     - Else → Move FORWARD at 0.12 m/s (slower for safety)

4. **Frontier Detection (SMART EXPLORATION):**
   - Subscribes to `/map` topic from SLAM Toolbox
   - Analyzes map to find unexplored areas (cells with value -1)
   - Counts unknown cells on LEFT vs RIGHT side of map
   - Robot turns toward side with MORE unexplored area
   - Logs exploration progress: `Map: X% explored`

5. **Movement Flow (OPTIMIZED FOR NARROW PATH NAVIGATION):**
   ```
   START → FORWARD state → Move forward at 0.12 m/s (slower for safety)
          ↓
   Obstacle detected at 1.0m (LATE DETECTION - allows narrow paths)
          ↓
   EMERGENCY STOP if < 0.3m → Immediate 180° turn
          ↓
   Check if CORNER (both sides < 0.6m)?
          ↓ Yes                        ↓ No
   180° turn (3.14s)         Compare left vs right space
          ↓                            ↓
   Turn complete              45° turn (0.7-0.8s) - CONSISTENT
          ↓                            ↓
   Back to FORWARD ←──────────┘
          ↓
   After 15s straight → 20° turn toward unexplored area (FRONTIER DETECTION)
          ↓
   REPEAT until map complete
   ```

   **Key improvements:**
   - ✅ **Narrow path navigation** (1.0m detection allows entering tight spaces)
   - ✅ **Emergency stop** (0.3m safety threshold prevents wall collisions)
   - ✅ **Slower speed** (0.12 m/s for safer navigation in tight spaces)
   - ✅ **Consistent turns** (45° obstacle turns, 20° exploration turns)
   - ✅ **Smart exploration** (turns toward unexplored areas, not random)
   - ✅ **Corner escape** (180° turn when trapped on both sides)
   - ✅ **Progress tracking** (logs X% explored)

6. **Registration in setup.py:**
   ```python
   entry_points={
       'console_scripts': [
           'auto_explorer = construction_monitor.auto_explorer:main',
       ],
   },
   ```
   This creates an executable command `ros2 run construction_monitor auto_explorer`

6. **Build process:**
   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select construction_monitor
   ```
   This creates the executable at:
   `~/ros2_ws/install/construction_monitor/lib/construction_monitor/auto_explorer`

**Commands to run (directory doesn't matter - can run from anywhere):**

**Terminal 1: Launch Construction Site World**
```bash
export TURTLEBOT3_MODEL=burger
source ~/ros2_ws/install/setup.bash
source /usr/share/gazebo/setup.bash
ros2 launch construction_monitor construction_world.launch.py
```

**Terminal 2: Launch SLAM Toolbox (with custom config for better accuracy)**
```bash
export TURTLEBOT3_MODEL=burger
source ~/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py params_file:=$(ros2 pkg prefix construction_monitor)/share/construction_monitor/config/slam_params.yaml
```

**Terminal 3: Launch RViz2 with auto-config (to visualize the map)**
```bash
export TURTLEBOT3_MODEL=burger
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix construction_monitor)/share/construction_monitor/config/slam_config.rviz
```
*Note: RViz2 now loads with Map, LaserScan, RobotModel, and TF displays pre-configured!*

**Alternative (manual config):**
```bash
ros2 run rviz2 rviz2
# Then manually: Add Map display, change Fixed Frame to "map"
```

**Terminal 4: Launch Auto Explorer (REPLACES teleop)**
```bash
export TURTLEBOT3_MODEL=burger
source ~/ros2_ws/install/setup.bash
ros2 run construction_monitor auto_explorer
```

**What you should see:**
- **Gazebo:** Robot moving at 0.12 m/s, making 45° turns near walls, 20° exploration turns
- **Terminal 4:** Log messages like:
  - "Auto Explorer Node Started! Obstacle avoidance distance: 1.0m"
  - "Min front distance: 2.50m (threshold: 1.0m)"
  - "Obstacle at 0.95m - Turn LEFT (L:2.30m > R:1.10m)"
  - "Turn complete - Moving FORWARD"
  - "Frontier turn LEFT - more unexplored area on left"
  - "CORNER! (L:0.45m R:0.52m F:0.88m) - 180° turn"
  - "EMERGENCY STOP! Wall at 0.28m"
- **RViz2:** Map of construction site being built with good coverage of narrow paths

**Files created:**
- [slam_config.rviz](src/construction_monitor/config/slam_config.rviz) - Pre-configured RViz2 settings
- [slam_params.yaml](src/construction_monitor/config/slam_params.yaml) - Custom SLAM parameters for better accuracy

**Custom SLAM Parameters (slam_params.yaml):**
| Setting | Default | Custom | Effect |
|---------|---------|--------|--------|
| `map_update_interval` | 5.0s | **1.0s** | Map updates 5x faster |
| `resolution` | 0.05m | **0.03m** | Sharper walls, more detail |
| `loop_search_maximum_distance` | 3.0m | **8.0m** | Better loop closure detection |
| `minimum_travel_distance` | 0.5m | **0.3m** | More frequent scan matching |
| `loop_match_minimum_chain_size` | 10 | **6** | Easier loop closure trigger |
| `min_laser_range` | 0.0m | **0.12m** | Matches TurtleBot3 LDS-01 spec |
| `max_laser_range` | 20.0m | **3.5m** | Matches TurtleBot3 LDS-01 spec |

**Result:** Robot autonomously explores the construction site with improved SLAM accuracy and better loop closure!

---

## Setup Instructions

### 1. Prerequisites
- ROS2 Humble installed
- Ubuntu 22.04 (Jammy)
- Gazebo 11

### 2. Install Required Packages

```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot3 \
                    ros-humble-turtlebot3-gazebo \
                    ros-humble-turtlebot3-navigation2 \
                    ros-humble-turtlebot3-teleop \
                    ros-humble-slam-toolbox
```

### 3. Environment Setup

Add these lines to your `~/.bashrc`:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
```

Then reload your bashrc:

```bash
source ~/.bashrc
```

---

## Testing Steps

### Step 1: Verify Installation

Check if TurtleBot3 packages are installed:

```bash
ros2 pkg list | grep turtlebot3
ros2 pkg list | grep slam_toolbox
```

### Step 2: Test Single Robot in Empty World

Launch TurtleBot3 in an empty Gazebo world:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Expected Result:** Gazebo window opens with one TurtleBot3 robot in an empty gray world.

To stop: Press `Ctrl+C` in the terminal.

### Step 3: Test Teleop Control

In a **new terminal**, control the robot with keyboard:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Controls:**
- `w` - Move forward
- `x` - Move backward
- `a` - Turn left
- `d` - Turn right
- `s` - Stop
- `q` / `z` - Increase/decrease speed

### Step 4: Test SLAM Mapping (Coming Soon)

Launch robot with SLAM:

```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2: Launch SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Control robot
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 4: View map in RViz
ros2 launch turtlebot3_bringup rviz2.launch.py
```

---

## Project Structure (To Be Created)

```
ros2_ws/
├── src/
│   └── construction_monitor/
│       ├── worlds/
│       │   ├── construction_complete.world
│       │   └── construction_incomplete.world
│       ├── launch/
│       │   ├── spawn_two_robots.launch.py
│       │   └── mapping.launch.py
│       ├── maps/
│       │   ├── complete_map.yaml
│       │   └── complete_map.pgm
│       ├── scripts/
│       │   ├── map_comparison.py
│       │   └── progress_calculator.py
│       └── config/
│           └── navigation_params.yaml
└── README.md
```

---

## Development Roadmap

- [x] Step 1: Install TurtleBot3 packages
- [x] Step 2: Test single robot in empty world
- [ ] Step 3: Test teleop control
- [ ] Step 4: Create construction site world in Gazebo
- [ ] Step 5: Test SLAM mapping with one robot
- [ ] Step 6: Save complete reference map
- [ ] Step 7: Spawn two robots in Gazebo
- [ ] Step 8: Configure robots to navigate different areas
- [ ] Step 9: Implement map comparison algorithm
- [ ] Step 10: Calculate progress percentage

---

## Useful Commands

### Check ROS2 Topics
```bash
ros2 topic list
```

### View Robot Position
```bash
ros2 topic echo /odom
```

### Save a Map
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Kill All ROS2 Nodes
```bash
killall -9 gzserver gzclient
```

---

## Troubleshooting

### Gazebo doesn't start or robot doesn't spawn

**Solution:** Make sure to source both ROS2 and Gazebo setup files:
```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
```

### "Service /spawn_entity unavailable" error

**Solution:** The Gazebo ROS plugins aren't loaded. Run:
```bash
source /usr/share/gazebo/setup.bash
```

### Robot model not found

**Solution:** Set the TurtleBot3 model:
```bash
export TURTLEBOT3_MODEL=burger
```

---

## Resources

- [TurtleBot3 Official Docs](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo Tutorials](https://gazebosim.org/tutorials)

---

## Notes

- Using TurtleBot3 Burger model (simplest and fastest)
- Using SLAM Toolbox (better for multi-robot mapping than Cartographer)
- Testing step-by-step to ensure no errors

---

**Last Updated:** 2025-12-12
