# Deferred P0-P2 QA cases — NOT EXECUTED

| ID | Fault / stimulus | Expected response |
|---|---|---|
| START_NO_CAMERA | RGB signal frames missing | no movement, NO_FRAMES |
| START_RED | 4 to 0 red lamp sequence | motion only after authorization |
| START_NO_SIGNAL | explicitly requested fallback | 7s after spawn, only without observed red |
| COMMAND_SKEW | speed and steering >120ms apart | adapter sends zero twist |
| COMMAND_STALL | drive command but zero odometry | drive_chain reports mismatch |
| RGBD_TIME | road RGB and Depth timestamps >50ms apart | do not fuse as current road |
| NO_ROAD | road boundaries unobserved | invalid corridor, no invented free cells |
| STRAIGHT_OSCILLATION | alternating unverified steering | rejection/physical steering limit |
| TRUE_CORNER | consistent progressive same-side demand | gradual turn allowed |
| SLAM_JUMP | implausible VO displacement | loop closure authorization revoked |
| REVERSE_MAP | map target tangent opposes vehicle heading | map target rejected |
| MULTICAR_PASS | passing side obstructed | HOLD and conservative speed |
| NO_SPEED_APPROVAL | 8/10m/s not authorized | 5m/s upper stage gate |
| MONZA_LAP | official unchanged track | independent lap evidence |
| FIVE_TRACKS | each named non-Monza track | separate successful run evidence |
| TEN_CARS | ten contemporaneous vehicles | no collision, offtrack or unsafe merges |

Record run SHA, actual frames and clock, ROSbag, individual turn/drive timing, measured odometry, first failure, safety impact, and official lap result. Only separate QA team may assign PASS/FAIL after user requests verification.
