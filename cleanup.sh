cd ~/ros2_ws/src/ur5e_collision_avoidance

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
colcon build --packages-select ur5e_collision_avoidance --symlink-install
source install/setup.bash

echo "=== Verifying install symlinks point to correct files ==="
ls -la ~/ros2_ws/src/ur5e_collision_avoidance/install/ur5e_collision_avoidance/lib/ur5e_collision_avoidance/

export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir

ros2 launch ur5e_collision_avoidance simulation.launch.py use_rviz:=true
```

---

Also — looking at your tree, I can see the package was built from **inside** `src/ur5e_collision_avoidance` rather than from `~/ros2_ws`. Your `build/`, `install/`, and `log/` folders are **inside the package** instead of at workspace root. That means colcon is confused. The correct workspace layout should be:
```
~/ros2_ws/
├── src/
│   └── ur5e_collision_avoidance/   ← package lives here
├── build/                          ← colcon puts this here
├── install/                        ← colcon puts this here
└── log/                            ← colcon puts this here