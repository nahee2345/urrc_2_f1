from pathlib import Path
import json
import re
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]


def test_entry_uses_latest_contract_and_team_topics():
    entry = json.loads((PACKAGE / "race_entry.json").read_text())
    assert entry == {
        "entry_id": "team_10",
        "display_name": "Hanla Racing",
        "ros_package": "team10_race",
        "algorithm_launch_file": "race.launch.py",
        "vehicle_model_file": "src/team_10/models/team10_car/model.sdf",
        "vehicle_model_format": "sdf",
        "spawn_z_offset_m": 0.12,
        "speed_topic": "/team_10/speed_10",
        "steering_topic": "/team_10/steering_10",
        "lidar_topic": "/team_10/scan_10",
        "launch_arguments": {},
    }


def test_participant_launch_declares_contract_without_central_roles():
    launch = (PACKAGE / "launch" / "race.launch.py").read_text()
    for argument in (
        "race_start_topic", "speed_topic", "steering_topic", "lidar_topic", "use_sim_time"
    ):
        assert f'DeclareLaunchArgument("{argument}"' in launch
    lowered = launch.lower()
    for forbidden in ("ros_gz_sim\", executable=\"create", "gz sim", "spawn_entity"):
        assert forbidden not in lowered
    assert "parameter_bridge" in launch
    assert "navigation" in launch
    assert "race_adapter" in launch


def test_no_absolute_home_paths_or_forbidden_navigation_inputs():
    source = "\n".join(
        path.read_text()
        for path in PACKAGE.rglob("*")
        if path.is_file() and "test" not in path.parts and "tools" not in path.parts and path.suffix in {".py", ".yaml", ".json", ".sdf", ".xml"}
    ).lower()
    assert "/" + "home" + "/" not in source
    for forbidden in ("nav_msgs/msg/odometry", "sensor_msgs/msg/navsatfix", "sensor_msgs/msg/imu"):
        assert forbidden not in source
    for track_name in ("monza", "spa", "suzuka", "silverstone", "monaco"):
        assert re.search(rf"\b{track_name}\b", source) is None


def test_team_isolated_lidar_and_ackermann_contract():
    model = ET.parse(PACKAGE / "models" / "team10_car" / "model.sdf").getroot().find("model")
    assert model.get("name") == "team10_car"
    sensor = model.find("link[@name='lidar_link']/sensor")
    assert sensor.findtext("topic") == "/team_10/scan_gz_10"
    assert sensor.get("type") == "gpu_lidar"
    assert sensor.find("../collision") is None
    opponent_sensor = model.find(
        "link[@name='lidar_link']/sensor[@name='opponent_lidar']"
    )
    assert opponent_sensor.findtext("topic") == "/team_10/opponent_scan_gz_10"
    assert opponent_sensor.get("type") == "gpu_lidar"
    launch = (PACKAGE / "launch" / "race.launch.py").read_text()
    assert 'DeclareLaunchArgument("opponent_lidar_topic"' in launch
    assert '"/team_10/opponent_scan_gz_10@sensor_msgs/msg/LaserScan' in launch
    plugin = model.find("plugin[@name='gz::sim::systems::AckermannSteering']")
    assert plugin is not None
    assert float(plugin.findtext("steering_limit")) <= 0.6108652382
    assert float(plugin.findtext("wheel_base")) == 0.340
