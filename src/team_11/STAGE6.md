# Stage 6 — track / starting grid context and initial lane policy

## Implemented
- Six official tracks, grid slots 1..11.
- Odd grids -> right path 1, even grids -> left path 3.
- Position-independent lateral convention (driver-right=-1, left=+1).
- ROS2 diagnostic publisher at /race_context_11 and /initial_lane_11.
- Unset/ambiguous grid and track are rejected rather than guessing.
- No use of map positions, race timing topics, world odometry or opponents to make driving decisions.

## Important scope
Official upstream runner (race_manager.py) does **not** pass track name or
starting-grid number in required launch arguments. Reliable automatic discovery
of a numbered grid is therefore NOT established. A LiDAR scan cannot reliably
read numbered painted grid slots, and the official rules prohibit using world
pose and stored coordinates for navigation. At this stage context must come
from the local test launcher, via explicit track/grid values. Do not claim that
the formal race manager passes it automatically. Before formal submission this
requires organizer-approved injection of non-driving metadata OR a compliant
sensor-only start-lane policy that never needs numbered grid identifiers.

## Update, test and run
```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p 'test_*.py' -v
bash src/team_11/tests/run_stage6.sh monza 1
```
The single launcher runs the stage-5 integrated test and launches context
publisher with the same explicit test track/grid. Context is diagnostic only:
vehicle will still perform a straight one-second test, not an autonomous lap.
