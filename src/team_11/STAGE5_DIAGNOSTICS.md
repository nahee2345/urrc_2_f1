# Stage 5 — front/rear and black-background diagnostics

## Front and rear
- Vehicle front is local +X and has **white front lights**.
- Vehicle rear is local -X and has **red rear lights**.
- The four wheel link rotations were changed from +90° about X to -90° about X to correct a suspected reversed rolling direction.
- This is a **hypothesis to verify by Gazebo motion**, not an already measured successful fix. Driver command sign conversion is unchanged.

## Official background
The official upstream `urrc_f1_ws` intentionally sets `<background>0 0 0 1</background>` and renders the base ground black; its `BLACK_BACKGROUND_CHANGE.md` confirms this design. TEAM 11 must not unilaterally modify those official map files.

Official launch `run_track.sh` invokes the official map runner, which conditionally prepares `road_textured.obj`, `asphalt_ground.obj`, textures and spectator cameras. Previous TEAM 11 stage5_world.sh bypassed this setup. The launcher now refuses to run when those official assets are absent, avoiding silent incomplete rendering.

If the launcher asks for official asset preparation, and you authorize generating assets in `urrc_f1_ws`, run the upstream official launcher separately:
```bash
cd ~/urrc_f1_ws
bash run_track.sh monza --view grid
```
This command **can update generated assets and world files in urrc_f1_ws** through its own official implementation. It does not add any custom TEAM 11 scenery or alter other TEAM 11 code. Exit the official Gazebo before starting the TEAM 11 script again.

## Retest
```bash
cd ~/urrc_2_f1
git pull --ff-only origin main
PYTHONPATH=src/team_11 python3 -m unittest discover -s src/team_11 -p 'test_*.py' -v
gz sdf -p src/team_11/models/team11_yellow_car/model.sdf >/tmp/team11_expanded.sdf
```
Close Gazebo and reopen the stage-5 official world, respawn vehicle (old spawn uses old SDF). Use short manual straight command and visually verify white lights lead; red lights trail. Do not reuse an already spawned vehicle to validate a model-file change.
