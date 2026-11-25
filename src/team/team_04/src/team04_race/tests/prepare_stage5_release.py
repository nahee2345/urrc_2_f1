#!/usr/bin/env python3
"""Original official pack/intake in a new team-local stage5 fixture."""
from pathlib import Path
import argparse,os,sys,shutil,subprocess,json,re
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--revision');a=p.parse_args();root=a.root.resolve()
if a.revision and not re.fullmatch('[a-z0-9]+',a.revision):raise ValueError('revision must be alphanumeric')
out=root/'src/team_04/artifacts/stage5';release=out/('release_'+a.revision if a.revision else 'final_release');fixture=out/('relocation_'+a.revision+'_ws' if a.revision else 'relocation_ws');archive=release/'team04_stage5.zip'
if archive.exists() or fixture.exists():raise FileExistsError('Preserve existing release; use a new explicit release directory after inspection')
release.mkdir(exist_ok=True)
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
subprocess.run([sys.executable,'-s',str(root/'src/team_04/team04_race/scripts/package_submission.py'),'--output',str(archive)],check=True,env=env)
(fixture/'race/inbox').mkdir(parents=True)
shutil.copy2(archive,fixture/'race/inbox/team04_stage5.zip')
shutil.copy2(root/'race/event_config.json',fixture/'race/event_config.json')
shutil.copy2(out/'baseline.json',release/'baseline.json')
for track in ['monza','hungaroring']:
 for rel in [f'worlds/{track}_race.sdf',f'tracks/processed/{track}.json']:
  target=fixture/'src/urrc_track_gazebo'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/'src/urrc_track_gazebo'/rel,target)
sys.path.insert(0,str(root/'race/scripts'));import race_manager
entries=race_manager.intake(fixture)
assert len(entries)==1 and entries[0]['entry_id']=='team04'
print(json.dumps(dict(archive=str(archive),fixture=str(fixture),official_intake=True),indent=2))
