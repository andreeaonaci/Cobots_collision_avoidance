# UR5e Dual Collision Avoidance Demo (Current)

ROS 2 Humble + Gazebo Classic demo with two UR5e robots, live web control, and TF-based collision monitoring.

## Current behavior

- Two UR5e robots are spawned in one world (`robot1_` and `robot2_`).
- Motion is driven by `scenario_orchestrator.py` via joint trajectory streaming.
- Collision monitor reads TF distance between:
  - `robot1_inner_sensor`
  - `robot2_inner_sensor`
- State machine: `FREE -> SLOWING -> STOPPED -> RESUMING -> FREE`.
- In `STOPPED`, both robots are hard-stopped for safety.
- Web dashboard is served at `http://localhost:8000` and controls start/pause/reset plus live tuning.

## Important paths

- Launch: `launch/simulation.launch.py`
- URDF: `urdf/dual_ur5e.urdf.xacro`
- Avoidance config: `config/collision_avoidance_params.yaml`
- Runtime scripts (authoritative): `ur5e_collision_avoidance/scripts/`
- Cleanup helper: `cleanup.sh`

## Quick run (recommended)

From workspace root:

```bash
cd ~/ros2_ws
bash src/ur5e_collision_avoidance/cleanup.sh
```

`cleanup.sh` does the full cycle:

- kills stale Gazebo/ROS/web processes
- normalizes script line endings and permissions
- rebuilds with `--symlink-install`
- sources ROS + workspace setup
- launches `simulation.launch.py`

After launch:

- open `http://localhost:8000`
- press `Start` in the dashboard

## Manual run (without cleanup script)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5e_collision_avoidance --symlink-install
source install/setup.bash
ros2 launch ur5e_collision_avoidance simulation.launch.py
```

## Useful runtime checks

```bash
ros2 topic echo /collision_avoidance/status
ros2 topic echo /collision_avoidance/robot1_speed
ros2 topic echo /collision_avoidance/robot2_speed
ros2 topic echo /demo/scenario_state
cat /tmp/collision_avoidance_events.json
```

## Current default tuning

From `config/collision_avoidance_params.yaml`:

- `danger_zone_m: 0.15`
- `slow_zone_m: 0.40`
- `resume_zone_m: 0.50`
- `min_speed_factor: 0.02`
- `yield_robot: "robot2"` (used in slowing/resuming policy)
- `resume_delay_sec: 0.8`

## Notes

- Launch starts `rosbridge_websocket` on `9090` and web server on `8000`.
- Scenario auto-start is disabled by launch override (`auto_start: false`), so use the web `Start` command.
- If browser view looks stale after updates, hard refresh with `Ctrl+Shift+R`.