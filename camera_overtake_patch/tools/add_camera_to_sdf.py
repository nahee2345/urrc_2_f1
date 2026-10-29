#!/usr/bin/env python3
from pathlib import Path
import sys

CAMERA_XML = r'''

    <!-- Front camera for LiDAR + camera fusion -->
    <link name="camera_link">
      <pose relative_to="chassis">0.16 0 0.075 0 -0.08 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
        </inertia>
      </inertial>
      <sensor name="front_camera" type="camera">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <topic>/camera/image_raw</topic>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>80.0</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="camera_fixed_joint" type="fixed">
      <parent>chassis</parent>
      <child>camera_link</child>
    </joint>
'''


def main():
    if len(sys.argv) != 2:
        print('usage: add_camera_to_sdf.py /path/to/model_f1_lidar_audi_clean.sdf')
        return 2
    path = Path(sys.argv[1]).expanduser().resolve()
    text = path.read_text(encoding='utf-8')
    if 'name="front_camera"' in text or "name='front_camera'" in text:
        print(f'already patched: {path}')
        return 0
    if 'name="chassis"' not in text and "name='chassis'" not in text:
        raise SystemExit('chassis link was not found; camera parent must be checked manually')
    pos = text.rfind('</model>')
    if pos < 0:
        raise SystemExit('</model> not found')
    backup = path.with_suffix(path.suffix + '.before_camera')
    if not backup.exists():
        backup.write_text(text, encoding='utf-8')
    patched = text[:pos] + CAMERA_XML + '\n  ' + text[pos:]
    path.write_text(patched, encoding='utf-8')
    print(f'patched camera into: {path}')
    print(f'backup: {backup}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
