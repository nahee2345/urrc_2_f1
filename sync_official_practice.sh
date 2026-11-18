#!/usr/bin/env bash
# Align official organizer files with urrc_f1_ws/main.
# Preserve src/team/team_08, team_01, team_02 and existing legacy tests.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if [[ ! -d .git ]]; then
  echo "Must run inside urrc_2_f1 Git checkout" >&2; exit 1
fi
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "Working tree has tracked modifications; refusing to overwrite them" >&2
  exit 1
fi
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
git clone --quiet --depth 1 --branch main https://github.com/nahee2345/urrc_f1_ws.git "$TMP/official"
python3 - "$ROOT" "$TMP/official" <<'PY'
from pathlib import Path
import shutil
import subprocess
import sys
root, official = (Path(x).resolve() for x in sys.argv[1:])
tracked=subprocess.check_output(['git','-C',str(official),'ls-files','-z']).decode().split('\0')
files={p for p in tracked if p}
# Synchronize tracked files only; preserve untracked race/install,
# build outputs, previews, logs and the user's working data.
managed=(Path('race'),Path('src/urrc_track_gazebo'))
local_tracked=subprocess.check_output(['git','-C',str(root),'ls-files','-z']).decode().split('\\0')
for rel in local_tracked:
    if not rel or rel in files:continue
    p=Path(rel)
    if any(p==folder or folder in p.parents for folder in managed):
        target=root/p
        if target.is_file() or target.is_symlink():target.unlink()
# Synchronize the complete upstream tracked official file set.
for rel in sorted(files):
    p=Path(rel)
    if p.parts[0]=='src' and len(p.parts)>1 and p.parts[1]=='team':
        continue
    dest=root/p
    dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(official/p,dest)
print(f"Copied {len(files)} official tracked files from urrc_f1_ws/main")
PY
python3 "$ROOT/.github/scripts/create_practice_teams.py"
echo "Official files synchronized; team_08 preserved; team_01 and team_02 generated."
echo "Review with: git status --short"
echo "To publish after inspection: git add -A && git commit -m 'Sync official race environment and Monaco practice teams' && git push origin main"
