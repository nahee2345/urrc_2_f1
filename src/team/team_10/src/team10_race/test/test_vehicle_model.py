from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest

MODEL = Path(__file__).resolve().parents[1] / "models" / "team10_car" / "model.sdf"
LIMITS = (0.55, 0.25, 0.18)


def _pose(element):
    values = [float(value) for value in (element.findtext("pose") or "0 0 0 0 0 0").split()]
    return values[:3], values[3:]


def _rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matmul(first, second):
    return tuple(tuple(sum(first[i][k] * second[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _matvec(matrix, vector):
    return tuple(sum(matrix[i][k] * vector[k] for k in range(3)) for i in range(3))


def _half_extents(geometry, rotation):
    box = geometry.find("box")
    if box is not None:
        half = tuple(float(value) / 2.0 for value in box.findtext("size").split())
        return tuple(sum(abs(rotation[i][j]) * half[j] for j in range(3)) for i in range(3))
    sphere = geometry.find("sphere")
    if sphere is not None:
        radius = float(sphere.findtext("radius"))
        return radius, radius, radius
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.findtext("radius"))
        half_length = float(cylinder.findtext("length")) / 2.0
        axis = tuple(rotation[i][2] for i in range(3))
        return tuple(math.sqrt(radius * radius * (1.0 - value * value) + half_length * half_length * value * value) for value in axis)
    raise AssertionError("unsupported geometry")


def model_envelope():
    model = ET.parse(MODEL).getroot().find("model")
    low = [math.inf] * 3
    high = [-math.inf] * 3
    for link in model.findall("link"):
        link_position, link_rpy = _pose(link)
        link_rotation = _rotation(link_rpy)
        for shape in [*link.findall("visual"), *link.findall("collision")]:
            local_position, local_rpy = _pose(shape)
            rotation = _matmul(link_rotation, _rotation(local_rpy))
            offset = _matvec(link_rotation, local_position)
            center = tuple(link_position[i] + offset[i] for i in range(3))
            half = _half_extents(shape.find("geometry"), rotation)
            for axis in range(3):
                low[axis] = min(low[axis], center[axis] - half[axis])
                high[axis] = max(high[axis], center[axis] + half[axis])
    return tuple(high[i] - low[i] for i in range(3)), tuple(low), tuple(high)


def test_vehicle_envelope_and_spawn_offset():
    dimensions, low, high = model_envelope()
    assert all(actual <= limit + 1e-9 for actual, limit in zip(dimensions, LIMITS))
    assert dimensions == pytest.approx((0.536, 0.232, 0.155), abs=1e-6)
    assert low == pytest.approx((-0.260, -0.116, -0.120), abs=1e-6)
    assert high == pytest.approx((0.276, 0.116, 0.035), abs=1e-6)
    assert -low[2] == pytest.approx(0.120, abs=1e-9)


def test_wheel_contact_is_lowest_collision_and_lidar_is_collision_free():
    model = ET.parse(MODEL).getroot().find("model")
    for name in ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"):
        link = model.find(f"link[@name='{name}']")
        assert float(link.findtext("pose").split()[2]) - 0.038 == pytest.approx(-0.120)
        assert link.find("collision/geometry/cylinder") is not None
    assert model.find("link[@name='lidar_link']/collision") is None


def test_primary_is_unchanged_and_secondary_is_horizontal_opponent_only_scan():
    model = ET.parse(MODEL).getroot().find("model")
    link = model.find("link[@name='lidar_link']")
    assert [float(value) for value in link.findtext("pose").split()] == pytest.approx(
        [0.080, 0.0, -0.045, 0.0, 0.0, 0.0]
    )
    primary = link.find("sensor[@name='front_lidar']")
    secondary = link.find("sensor[@name='opponent_lidar']")
    assert primary.findtext("topic") == "/team_10/scan_gz_10"
    assert [float(v) for v in primary.findtext("pose").split()] == pytest.approx([0, 0, .015, 0, 0, 0])
    assert secondary.findtext("topic") == "/team_10/opponent_scan_gz_10"
    assert [float(value) for value in secondary.findtext("pose").split()] == pytest.approx(
        [0.120, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    assert primary.findtext("update_rate") == "30"
    assert secondary.findtext("update_rate") == "30"
    for sensor in (primary, secondary):
        assert sensor.findtext("lidar/scan/horizontal/samples") == "720"
        assert sensor.findtext("lidar/range/max") == "25.0"


def test_rear_sensor_is_horizontal_and_inside_original_envelope():
    model = ET.parse(MODEL).getroot().find('model')
    link = model.find("link[@name='lidar_link']")
    sensor = link.find("sensor[@name='rear_opponent_lidar']")
    values = list(map(float,sensor.findtext('pose').split()))
    assert values == pytest.approx([-.325,0,.015,0,0,math.pi])
    assert .080 + values[0] == pytest.approx(-.245)
    assert -.045 + values[2] == pytest.approx(-.030)
    assert sensor.findtext('topic') == '/team_10/rear_opponent_scan_gz_10'
    assert sensor.findtext('update_rate') == '30'
    assert sensor.findtext('lidar/scan/horizontal/samples') == '361'
    horizontal = sensor.find('lidar/scan/horizontal')
    assert float(horizontal.findtext('max_angle')) - float(horizontal.findtext('min_angle')) == pytest.approx(math.pi)
    assert model_envelope()[0] == pytest.approx((.536,.232,.155))
