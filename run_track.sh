#!/usr/bin/env bash
set -Eeuo pipefail
TRACK="${1:-monza}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *)
  echo "Supported tracks: monza spa monaco cota hungaroring baku" >&2
  exit 2 ;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$ROOT/src/urrc_track_gazebo"
WORLD="$PKG/worlds/${TRACK}_race.sdf"
GUI="$PKG/worlds/${TRACK}_grid.gui.config"
if [[ ! -f "$WORLD" || ! -f "$GUI" || ! -f "$PKG/models/urrc_$TRACK/model.sdf" || ! -f "$PKG/worlds/${TRACK}_signal_active.sdf" ]]; then
  echo "Track import missing. Run python3 import_6_tracks.py --official ~/urrc_f1_ws" >&2
  exit 2
fi
export GZ_SIM_RESOURCE_PATH="$PKG/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
echo "[URRC] Start official-copy track: $TRACK"
echo "[URRC] Start lights separately after the world/cameras/vehicles are ready:"
echo "  bash run_lights.sh $TRACK"
exec gz sim -r -v 3 --gui-config "$GUI" "$WORLD"
