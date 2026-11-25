#!/usr/bin/env bash
set -eo pipefail
TEAM_01_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Build against Jazzy, without recording the original workspace as an underlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
cd "$TEAM_01_ROOT"
# CMake and symlink-install outputs contain absolute paths. Preserve stale
# outputs instead of feeding an old workspace cache to the relocated build.
python3 - "$TEAM_01_ROOT" <<'RELOCATION_CHECK'
from pathlib import Path
import sys, time, shutil
root = Path(sys.argv[1])
stale = False
for cache in (root / 'build').glob('*/CMakeCache.txt'):
    for line in cache.read_text(errors='replace').splitlines():
        if line.startswith('CMAKE_HOME_DIRECTORY:INTERNAL='):
            source = Path(line.split('=', 1)[1])
            if source != root / 'src' / cache.parent.name:
                stale = True
if stale:
    backup = root / ('.relocation_backup_' + str(time.time_ns()))
    backup.mkdir()
    (backup / 'COLCON_IGNORE').touch()
    for name in ('build', 'install'):
        if (root / name).exists():
            shutil.move(str(root / name), str(backup / name))
    print('이전 위치의 빌드 결과를 보관하고 새 위치에서 빌드합니다: ' + str(backup), flush=True)
RELOCATION_CHECK
chmod +x "$TEAM_01_ROOT"/src/team_01_race/scripts/*
# Explicit src discovery allows the parent workspace to ignore this copy.
colcon build --base-paths src --symlink-install "$@"
