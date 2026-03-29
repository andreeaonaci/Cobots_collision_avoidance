#!/usr/bin/env bash
set -e

cd ~/ros2_ws/src/ur5e_collision_avoidance

echo "=== Stopping stale demo processes ==="
pkill -f "simulation.launch.py|scenario_orchestrator.py|collision_monitor.py|robot1_controller.py|robot2_controller.py|web_server.py|rosbridge_websocket|gzserver|gzclient|rviz2" 2>/dev/null || true
sleep 1

# ── The REAL scripts are here (what install/ symlinks point to)
SCRIPTS_DIR=~/ros2_ws/src/ur5e_collision_avoidance/ur5e_collision_avoidance/scripts

echo "=== Fixing line endings in all script locations ==="
for dir in \
  "$SCRIPTS_DIR" \
  ~/ros2_ws/src/ur5e_collision_avoidance/src/scripts \
  ~/ros2_ws/src/ur5e_collision_avoidance/src/ur5e_collision_avoidance/launch; do
  if [ -d "$dir" ]; then
    find "$dir" -name "*.py" | while read f; do
      python3 -c "
data = open('$f','rb').read()
fixed = data.replace(b'\r\n',b'\n').replace(b'\r',b'\n')
open('$f','wb').write(fixed)
print('Fixed:', '$f')
"
    done
  fi
done

echo "=== Making scripts executable ==="
chmod +x $SCRIPTS_DIR/*.py

echo "=== Verifying shebang (should end with \$ not ^M\$) ==="
for f in $SCRIPTS_DIR/*.py; do
  echo -n "$f: "
  head -1 "$f" | cat -A
done

echo "=== Rebuilding ==="
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5e_collision_avoidance --symlink-install
source install/setup.bash

echo "=== Verifying install symlinks point to correct files ==="
ls -la ~/ros2_ws/install/ur5e_collision_avoidance/lib/ur5e_collision_avoidance/

export DISPLAY=:0   
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir

ros2 launch ur5e_collision_avoidance simulation.launch.py use_rviz:=true