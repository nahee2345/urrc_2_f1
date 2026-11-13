#!/usr/bin/env bash
set -Eeuo pipefail
TRACK="${1:-monza}"
case "$TRACK" in monza|spa|monaco|cota|hungaroring|baku) ;; *)
  echo "Supported tracks: monza spa monaco cota hungaroring baku" >&2
  exit 2 ;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$ROOT/src/urrc_track_gazebo/scripts/traffic_lights_v10.py"
SIGNAL="$ROOT/src/urrc_track_gazebo/worlds/${TRACK}_signal_active.sdf"
if [[ ! -f "$SCRIPT" || ! -f "$SIGNAL" ]]; then
  echo "Track/signal files missing. Run python3 import_6_tracks.py first." >&2
  exit 2
fi
# Original upstream logic: wait for Gazebo visual service and both spectator
# camera image topics, then illuminate five red lenses and remove all at once.
exec /usr/bin/python3 "$SCRIPT" --track "$TRACK"
