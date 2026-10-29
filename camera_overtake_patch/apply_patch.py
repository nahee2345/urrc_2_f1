#!/usr/bin/env python3
from pathlib import Path
import re
import shutil
import subprocess
import sys

PATCH_ROOT = Path(__file__).resolve().parent


def add_entry_points(setup_path: Path):
    text = setup_path.read_text(encoding='utf-8')
    backup = setup_path.with_suffix('.py.before_camera_overtake')
    if not backup.exists():
        shutil.copy2(setup_path, backup)

    additions = [
        "camera_perception = urrc_lidar_racer.camera_perception:main",
        "overtake_fusion = urrc_lidar_racer.overtake_fusion:main",
    ]
    for entry in additions:
        if entry.split(' = ')[0] in text:
            continue
        m = re.search(r"(['\"]console_scripts['\"]\s*:\s*\[)", text)
        if not m:
            raise RuntimeError('console_scripts block not found in setup.py')
        text = text[:m.end()] + f"\n            '{entry}'," + text[m.end():]
    setup_path.write_text(text, encoding='utf-8')


def add_package_deps(package_path: Path):
    text = package_path.read_text(encoding='utf-8')
    backup = package_path.with_suffix('.xml.before_camera_overtake')
    if not backup.exists():
        shutil.copy2(package_path, backup)
    deps = ['sensor_msgs', 'geometry_msgs', 'std_msgs', 'cv_bridge', 'ros_gz_bridge']
    insert = ''
    for dep in deps:
        if f'<exec_depend>{dep}</exec_depend>' not in text and f'<depend>{dep}</depend>' not in text:
            insert += f'  <exec_depend>{dep}</exec_depend>\n'
    if insert:
        text = text.replace('</package>', insert + '</package>')
        package_path.write_text(text, encoding='utf-8')


def main():
    ws = Path(sys.argv[1] if len(sys.argv) > 1 else '.').expanduser().resolve()
    pkg = ws / 'src' / 'urrc_lidar_racer'
    if not pkg.exists():
        raise SystemExit(f'package not found: {pkg}')

    src_pkg = PATCH_ROOT / 'src' / 'urrc_lidar_racer'
    for rel in [
        'urrc_lidar_racer/camera_perception.py',
        'urrc_lidar_racer/overtake_fusion.py',
        'config/camera_overtake.yaml',
        'launch/race_camera_overtake.launch.py',
    ]:
        dst = pkg / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_pkg / rel, dst)
        print('copied', dst)

    setup = pkg / 'setup.py'
    package_xml = pkg / 'package.xml'
    add_entry_points(setup)
    add_package_deps(package_xml)

    candidates = list(ws.glob('models/urrc_car/model_f1_lidar_audi_clean.sdf'))
    candidates += list(ws.glob('**/model_f1_lidar_audi_clean.sdf'))
    candidates = list(dict.fromkeys(p.resolve() for p in candidates))
    if not candidates:
        print('WARNING: model_f1_lidar_audi_clean.sdf not found. Run tools/add_camera_to_sdf.py manually.')
    else:
        tool = PATCH_ROOT / 'tools' / 'add_camera_to_sdf.py'
        subprocess.run([sys.executable, str(tool), str(candidates[0])], check=True)

    print('\nPatch applied.')
    print('Build: colcon build --symlink-install')
    print('Launch: ros2 launch urrc_lidar_racer race_camera_overtake.launch.py')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
