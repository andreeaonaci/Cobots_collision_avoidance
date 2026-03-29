# UR5e Dual Cobot Collision Avoidance
### ROS 2 Humble · Gazebo Classic · Two UR5e Robots · Speed-scaling avoidance

---

## What this project does

Two UR5e cobots are mounted on opposite ends of a shared worktable, arms
extended toward each other on a direct collision course.

**Avoidance flow:**

```
START
  ↓  Both robots move at full speed toward each other
  ↓  EE distance enters SLOW ZONE  (< 0.50 m)
  ↓      → Robot 2 speed ramps DOWN  (linear, 100% → 5%)
  ↓  EE distance enters DANGER ZONE (< 0.20 m)
  ↓      → Robot 2 STOPS completely
  ↓      → Robot 1 continues at full speed, passes through
  ↓  EE distance exceeds RESUME ZONE (> 0.60 m)
  ↓      → Robot 2 resumes, speed ramps back UP
  ↓  Both robots reach final waypoints
END
```

---

## Project structure

```
ur5e_collision_avoidance/
└── src/
    └── ur5e_collision_avoidance/
        ├── package.xml
        ├── CMakeLists.txt
        ├── setup.py
        ├── __init__.py
        │
        ├── urdf/
        │   └── dual_ur5e.urdf.xacro      ← Two UR5e robots, world-anchored
        │
        ├── worlds/
        │   └── ur5e_workspace.world       ← Gazebo world: table, pedestals, markers
        │
        ├── config/
        │   ├── dual_ur5e_controllers.yaml ← ros2_control joint trajectory controllers
        │   └── collision_avoidance_params.yaml  ← all tunable thresholds
        │
        ├── launch/
        │   └── simulation.launch.py       ← MASTER launch file (run this)
        │
        ├── rviz/
        │   └── dual_ur5e.rviz             ← RViz2 config
        │
        └── scripts/
            ├── collision_monitor.py       ← TF-based distance monitor + state machine
            ├── scenario_orchestrator.py   ← Drives robots through collision trajectory
            ├── robot1_controller.py       ← Robot 1 speed relay (keeps going)
            └── robot2_controller.py       ← Robot 2 speed relay (yields/stops)
```

---

## Prerequisites

### 1. ROS 2 Humble (Ubuntu 22.04)

```bash
# Full desktop install
sudo apt install ros-humble-desktop
```

### 2. Gazebo Classic (Gazebo 11)

```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-gazebo-ros2-control
```

### 3. UR Robot packages

```bash
sudo apt install ros-humble-ur \
                 ros-humble-ur-description \
                 ros-humble-ur-moveit-config
```

### 4. ros2_control + controllers

```bash
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-joint-trajectory-controller \
                 ros-humble-joint-state-broadcaster
```

### 5. Other tools

```bash
sudo apt install ros-humble-xacro \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2 \
                 python3-colcon-common-extensions
```

---

## Build

```bash
# Create workspace (if needed)
mkdir -p ~/ros2_ws/src
cp -r ur5e_collision_avoidance/src/ur5e_collision_avoidance ~/ros2_ws/src/

cd ~/ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build
colcon build --packages-select ur5e_collision_avoidance --symlink-install

# Source workspace
source install/setup.bash
```

---

## Run

### Full simulation with Gazebo GUI

```bash
ros2 launch ur5e_collision_avoidance simulation.launch.py
```

### With RViz2 as well

```bash
ros2 launch ur5e_collision_avoidance simulation.launch.py use_rviz:=true
```

### Headless (CI / server)

```bash
ros2 launch ur5e_collision_avoidance simulation.launch.py gui:=false
```

---

## Monitor what's happening

### Watch the state machine live

```bash
ros2 topic echo /collision_avoidance/status
```

Output example:
```json
{"state": "STOPPED", "distance_m": 0.1823, "robot1_speed": 1.0, "robot2_speed": 0.0, "stamp": 123456789}
```

### Watch speed factors

```bash
ros2 topic echo /collision_avoidance/robot1_speed
ros2 topic echo /collision_avoidance/robot2_speed
```

### Event log (written on every state change)

```bash
cat /tmp/collision_avoidance_events.json
```

Sample:
```json
[
  {
    "time_sec": 8.42,
    "transition": "FREE → SLOWING",
    "distance_m": 0.4987,
    "robot1_ee": [-0.12, 0.0, 1.02],
    "robot2_ee": [0.38, 0.0, 1.02]
  },
  {
    "time_sec": 11.07,
    "transition": "SLOWING → STOPPED",
    "distance_m": 0.1994,
    "robot1_ee": [-0.01, 0.0, 1.02],
    "robot2_ee": [0.19, 0.0, 1.02]
  },
  {
    "time_sec": 15.33,
    "transition": "STOPPED → RESUMING",
    "distance_m": 0.6104,
    "robot1_ee": [0.41, 0.0, 1.02],
    "robot2_ee": [0.19, 0.0, 1.02]
  },
  {
    "time_sec": 15.83,
    "transition": "RESUMING → FREE",
    "distance_m": 0.6350,
    "robot1_ee": [0.45, 0.0, 1.02],
    "robot2_ee": [0.19, 0.0, 1.02]
  }
]
```

### Published topics overview

| Topic | Type | Description |
|---|---|---|
| `/collision_avoidance/status` | `std_msgs/String` | JSON state machine status |
| `/collision_avoidance/robot1_speed` | `std_msgs/Float64` | Robot 1 speed factor (0–1) |
| `/collision_avoidance/robot2_speed` | `std_msgs/Float64` | Robot 2 speed factor (0–1) |
| `/collision_avoidance/markers` | `visualization_msgs/MarkerArray` | RViz2 proximity line + EE spheres |
| `/robot1_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Robot 1 commands |
| `/robot2_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Robot 2 commands |

---

## Tune the avoidance behaviour

Edit `config/collision_avoidance_params.yaml`:

| Parameter | Default | Meaning |
|---|---|---|
| `slow_zone_m` | 0.50 m | Distance where robot starts slowing |
| `danger_zone_m` | 0.20 m | Distance where robot stops completely |
| `resume_zone_m` | 0.60 m | Distance at which stopped robot resumes |
| `min_speed_factor` | 0.05 | Minimum speed (5%) at edge of danger zone |
| `yield_robot` | "robot2" | Which robot stops — "robot1" or "robot2" |
| `resume_delay_sec` | 0.5 s | Pause before resuming after clearance |

---

## State machine diagram

```
         ┌──────────────────────────────────────────────────────────┐
         │                                                          │
    dist < slow_zone                                     dist > slow_zone
         │                                                          │
         ▼                                                          │
  ┌──────────┐     dist >= slow_zone     ┌──────┐                  │
  │ SLOWING  │ ─────────────────────────►│ FREE │◄─────────────────┘
  └──────────┘                           └──────┘
         │
   dist < danger_zone
         │
         ▼
  ┌──────────┐     dist > resume_zone    ┌──────────┐   delay done   ┌──────┐
  │ STOPPED  │ ─────────────────────────►│ RESUMING │───────────────►│ FREE │
  └──────────┘                           └──────────┘                └──────┘
```

**In SLOWING**: Robot 2 speed = linear ramp (1.0 → 0.05) as distance shrinks.
**In STOPPED**: Robot 2 speed = 0.0. Robot 1 continues at 1.0.
**In RESUMING**: Robot 2 speed ramps back 0.05 → 1.0 over `resume_delay_sec`.

---

## Extending the project

- **MoveIt 2**: Replace the incremental joint streaming in `scenario_orchestrator.py`
  with full MoveIt 2 `move_group` planning for real collision-aware path planning.
- **Real UR5e**: Change `sim_gazebo:=true` to `use_fake_hardware:=false` in the xacro
  and provide a real IP via `robot_ip` — the rest of the architecture is identical.
- **Speed slider**: On a real UR5e, send `/robot{n}/set_speed_slider_fraction`
  (ur_robot_driver topic) instead of the Float64 topics.
- **N robots**: `CollisionMonitor` can be extended to track pairwise distances
  across any number of robots and publish per-pair yield decisions.