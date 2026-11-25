# Source separately in every vehicle terminal, after starting the original preview.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || return 1
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
source install_f1/local_setup.bash
export ROS_DOMAIN_ID=42 ROS2CLI_NO_DAEMON=1
export ROS_LOG_DIR="$PWD/validation_f1/ros_logs"
export GZ_PARTITION="$(python3 - <<'PY'
from pathlib import Path
parts=set()
for p in Path('/proc').glob('[0-9]*'):
    try:
        cmd=(p/'cmdline').read_bytes()
        if b'gz sim' not in cmd or b'monaco_race.sdf' not in cmd: continue
        for e in (p/'environ').read_bytes().split(b'\0'):
            if e.startswith(b'GZ_PARTITION=urrc_v19_preview_'): parts.add(e.split(b'=',1)[1].decode())
    except OSError: pass
if len(parts)!=1: raise SystemExit(f'Exactly one Monaco preview required, found {parts}')
print(parts.pop())
PY
)"
[ -n "$GZ_PARTITION" ] || return 1
printf 'GZ_PARTITION=%s ROS_DOMAIN_ID=%s\n' "$GZ_PARTITION" "$ROS_DOMAIN_ID"
[ "$(ros2 pkg prefix hanla_race)" = "$PWD/install_f1/hanla_race" ] || return 1
printf 'package prefix: %s\n' "$(ros2 pkg prefix hanla_race)"
python3 -c 'from pathlib import Path; import hanla_race.relaxed_navigation_node as n; print("navigation:",Path(n.__file__).resolve()); print("model:",Path("install_f1/hanla_race/share/hanla_race/models/hanla_car/model.sdf").resolve())' 
