#!/usr/bin/env bash
# Restore ONLY Monza SDFs altered by the old in-place enhancement function.
# Does not alter original geometry, source history or any other track.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUB="$ROOT/upstream/urrc_f1_ws"
[[ -d "$SUB/src/urrc_track_gazebo" ]] || { echo 'Missing Monza submodule' >&2; exit 2; }
tracked=(
  src/urrc_track_gazebo/models/urrc_monza/model.sdf
  src/urrc_track_gazebo/worlds/monza.sdf
  src/urrc_track_gazebo/worlds/monza_race.sdf
)
# git restore only if the original tracked map was modified in the worktree.
if ! git -C "$SUB" diff --quiet -- "${tracked[@]}";then
  git -C "$SUB" restore --source=HEAD --worktree -- "${tracked[@]}"
  echo '[TEAM01] Monza tracked map source restored to submodule HEAD'
else
  echo '[TEAM01] Monza original tracked map already unchanged'
fi
generated=(
  src/urrc_track_gazebo/assets/asphalt.png
  src/urrc_track_gazebo/assets/offroad.png
  src/urrc_track_gazebo/models/urrc_monza/meshes/asphalt.png
  src/urrc_track_gazebo/models/urrc_monza/meshes/offroad.png
  src/urrc_track_gazebo/models/urrc_monza/meshes/road_textured.obj
  src/urrc_track_gazebo/models/urrc_monza/meshes/asphalt_ground.obj
  src/urrc_track_gazebo/models/urrc_monza/meshes/road_asphalt.mtl
  src/urrc_track_gazebo/worlds/monza_signal_active.sdf
)
for path in "${generated[@]}";do
  if ! git -C "$SUB" ls-files --error-unmatch -- "$path" >/dev/null 2>&1;then
    rm -f -- "$SUB/$path"
  fi
done
git -C "$SUB" diff --exit-code -- "${tracked[@]}"
echo '[TEAM01] Monza tracked source unchanged. All new assets use runtime copy.'
