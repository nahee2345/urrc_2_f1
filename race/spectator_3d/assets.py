"""Export *each team's* SDF visual model; never replace it with another team's car.

Read-only exporter. Resolves local mesh/texture files into a web asset sandbox.
Unsupported formats / missing geometry fail explicitly instead of faking visuals.
"""
from __future__ import annotations
from pathlib import Path
import json
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
WEB = Path(__file__).resolve().parent / 'web'
PKG = ROOT / 'src/urrc_track_gazebo'
AS = WEB / 'assets'
AS.mkdir(parents=True, exist_ok=True)
TRACKS = ('monza', 'spa', 'monaco', 'cota', 'hungaroring', 'baku')
TEAM_PATTERN = re.compile(r'^team_(?:0[1-9]|1[0-2])$')
SUPPORTED_MESH = {'.obj', '.stl', '.dae'}


def vec(s, n, default=0.):
    if not s:
        return [default] * n
    v = [float(x) for x in s.split()]
    if len(v) != n:
        raise ValueError(f'Expected {n} numbers: {s}')
    return v


def resolve_resource(uri, sdf_path, team_root=None):
    if not uri:
        raise ValueError('Empty SDF mesh/texture URI')
    if uri.startswith('model://'):
        relative = uri[len('model://'):]
        candidates = [PKG / 'models' / relative]
        if team_root:
            # Gazebo model URI may start at its own model's directory name.
            candidates.append(sdf_path.parent / relative)
            parts0 = Path(relative).parts
            if parts0 and parts0[0] == sdf_path.parent.name:
                candidates.append(sdf_path.parent.joinpath(*parts0[1:]))
            candidates.extend([d / relative for d in team_root.rglob('models') if d.is_dir()])
            candidates += [team_root / relative]
            parts = Path(relative).parts
            for base in team_root.rglob('model.config'):
                if base.parent.name == parts[0]:
                    candidates.append(base.parent.joinpath(*parts[1:]))
    elif uri.startswith('file://'):
        relative = uri[len('file://'):]
        candidates = [Path(relative)] if relative.startswith('/') else [sdf_path.parent / relative]
    elif '://' in uri:
        raise ValueError(f'Unsupported resource URI: {uri}')
    else:
        candidates = [sdf_path.parent / uri]
    for candidate in candidates:
        path = candidate.resolve()
        if path.is_file() and (path.is_relative_to(ROOT.resolve())):
            return path
    raise FileNotFoundError(f'SDF asset unresolved: {uri} in {sdf_path}')


def resource(uri, sdf_path, scope, team_root=None):
    path = resolve_resource(uri, sdf_path, team_root)
    # Preserve relative folder structure for Collada adjacent references.
    if team_root and path.is_relative_to(team_root.resolve()):
        rel = path.relative_to(team_root.resolve())
    elif path.is_relative_to((PKG/'models').resolve()):
        rel = path.relative_to((PKG/'models').resolve())
    else:
        rel = Path(path.name)
    dest = AS / scope / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, dest)
    return dest.relative_to(WEB).as_posix()


def model(m, sdf_path, scope, team_root=None):
    links = []
    for link in m.findall('link'):
        visuals = []
        for vis in link.findall('visual'):
            geometry = vis.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise ValueError(f'{scope}: geometry malformed in {sdf_path}')
            kind = geometry[0].tag
            color = vec(vis.findtext('material/diffuse'), 4, .7)
            row = {'name':vis.get('name',''), 'pose':vec(vis.findtext('pose'),6),
                   'color':color[:3], 'opacity':color[3], 'kind':kind}
            if kind == 'mesh':
                uri = geometry.findtext('mesh/uri')
                path = resolve_resource(uri, sdf_path, team_root)
                if path.suffix.lower() not in SUPPORTED_MESH:
                    raise ValueError(f'{scope}: unsupported mesh {path.suffix}: {path}')
                row['url'] = resource(uri, sdf_path, scope, team_root)
                row['scale'] = vec(geometry.findtext('mesh/scale'),3,1)
                tex = vis.findtext('material/pbr/metal/albedo_map') or vis.findtext('material/pbr/specular/albedo_map')
                row['texture'] = resource(tex,sdf_path,scope,team_root) if tex else None
            elif kind == 'box':
                row['size'] = vec(geometry.findtext('box/size'),3)
            elif kind == 'cylinder':
                row.update(radius=float(geometry.findtext('cylinder/radius')),
                           length=float(geometry.findtext('cylinder/length')))
            elif kind == 'sphere':
                row['radius'] = float(geometry.findtext('sphere/radius'))
            else:
                raise ValueError(f'{scope}: unsupported SDF geometry {kind} in {sdf_path}')
            visuals.append(row)
        links.append({'name':link.get('name'), 'pose':vec(link.findtext('pose'),6), 'visuals':visuals})
    if not any(link['visuals'] for link in links):
        raise ValueError(f'{scope}: model has no direct visual links: {sdf_path}')
    return {'name':m.get('name',''), 'pose':vec(m.findtext('pose'),6), 'links':links}


def quat(rpy):
    r,p,y=rpy
    cr,sr=math.cos(r/2),math.sin(r/2)
    cp,sp=math.cos(p/2),math.sin(p/2)
    cy,sy=math.cos(y/2),math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)


def qmul(a,b):
    x,y,z,w=a; X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)


def qrot(q,p):
    x,y,z,w=q
    v=(p[0],p[1],p[2],0.)
    qinv=(-x,-y,-z,w)
    return qmul(qmul(q,v),qinv)[:3]


def qeuler(q):
    x,y,z,w=q
    sinp=max(-1.,min(1.,2*(w*y-z*x)))
    return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(sinp), math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]


def origin(elt):
    if elt is None:return [0.]*6
    return vec(elt.get('xyz'),3)+vec(elt.get('rpy'),3)


def compose(a,b):
    qa=quat(a[3:]); qb=quat(b[3:])
    rotated=qrot(qa,b[:3])
    return [a[i]+rotated[i] for i in range(3)]+qeuler(qmul(qa,qb))


def urdf_model(robot, source, scope, team_root):
    """Build static URDF appearance from joint transforms and each link's visuals.

    Revolute/continuous joints use zero position, as in default Gazebo spawn;
    dynamic steering/wheel animation is outside this static exporter.
    """
    links={l.get('name'):l for l in robot.findall('link')}
    parents={}
    for joint in robot.findall('joint'):
        child=joint.find('child'); parent=joint.find('parent')
        if child is None or parent is None:continue
        c=child.get('link'); pa=parent.get('link')
        if c in parents:raise ValueError('URDF duplicate joint parent: '+str(c))
        parents[c]=(pa,origin(joint.find('origin')))
    poses={}; visiting=set()
    def pose_of(name):
        if name in poses:return poses[name]
        if name in visiting:raise ValueError('URDF joint cycle: '+name)
        visiting.add(name)
        if name not in parents:
            transform=[0.]*6
        else:
            parent,local=parents[name]
            if parent not in links:raise ValueError('URDF parent link missing: '+str(parent))
            transform=compose(pose_of(parent),local)
        visiting.remove(name);poses[name]=transform
        return transform
    result=[]
    for name,link in links.items():
        visuals=[]
        for i,vis in enumerate(link.findall('visual')):
            geometry=vis.find('geometry')
            if geometry is None or len(geometry)!=1:
                raise ValueError(f'{scope}: URDF visual geometry malformed: {name}')
            g=geometry[0]; kind=g.tag
            mat=vis.find('material');rgba=None
            if mat is not None:
                color=mat.find('color')
                if color is not None:rgba=color.get('rgba')
            # Named global URDF material declarations are also supported.
            if rgba is None and mat is not None and mat.get('name'):
                global_mat=robot.find(f"material[@name='{mat.get('name')}']/color")
                if global_mat is not None:rgba=global_mat.get('rgba')
            color=vec(rgba,4,.7)
            row={'name':vis.get('name',f'{name}_visual_{i}'),
                 'pose':origin(vis.find('origin')),
                 'color':color[:3],'opacity':color[3],'kind':kind}
            if kind=='box':row['size']=vec(g.get('size'),3)
            elif kind=='cylinder':row.update(radius=float(g.get('radius')),length=float(g.get('length')))
            elif kind=='sphere':row['radius']=float(g.get('radius'))
            elif kind=='mesh':
                filename=g.get('filename')
                if not filename:raise ValueError('URDF mesh filename missing')
                # package:// requires its ROS package's source folder. The
                # exporter never reads files outside the competition tree.
                if filename.startswith('package://'):
                    rel=filename[len('package://'):]
                    parts=Path(rel).parts
                    options=[directory.joinpath(*parts[1:])
                             for directory in team_root.rglob(parts[0]) if directory.is_dir()] if parts else []
                    options += [team_root/rel,source.parent/rel]
                    matches=[x for x in options if x.is_file() and x.resolve().is_relative_to(ROOT.resolve())]
                    if not matches:raise FileNotFoundError('URDF package resource unresolved: '+filename)
                    filename='file://'+str(matches[0].resolve())
                path=resolve_resource(filename,source,team_root)
                if path.suffix.lower() not in SUPPORTED_MESH:
                    raise ValueError('Unsupported URDF mesh: '+str(path))
                row['url']=resource(filename,source,scope,team_root)
                row['scale']=vec(g.get('scale'),3,1.)
                row['texture']=None
            else:raise ValueError('Unsupported URDF geometry: '+kind)
            visuals.append(row)
        result.append({'name':name,'pose':pose_of(name),'visuals':visuals})
    if not any(x['visuals'] for x in result):
        raise ValueError('URDF has no visual geometry: '+str(source))
    return {'name':robot.get('name',scope),'pose':[0.]*6,'links':result}

def car_models():
    candidates = {}
    entrants = ROOT/'race/entrants'
    teams = ROOT/'src/team'
    for directory in sorted(teams.glob('team_*')):
        if TEAM_PATTERN.fullmatch(directory.name):
            candidates[directory.name] = directory
    if entrants.is_dir():
        for directory in sorted(entrants.glob('team_*')):
            if TEAM_PATTERN.fullmatch(directory.name) and directory.is_dir():
                candidates[directory.name] = directory
    models, errors = {}, {}
    for name, root in sorted(candidates.items()):
        # Prefer actual prepared participant model when present. Avoid stale
        # or unrelated backup models and never silently borrow other cars.
        entry = root / 'race_entry.json'
        specified = []
        if entry.is_file():
            try:
                config = json.loads(entry.read_text(encoding='utf-8'))
                rel = config.get('vehicle_model_file')
                if rel:
                    selected = (root / rel).resolve()
                    if selected.is_relative_to(root.resolve()) and selected.is_file():
                        specified.append(selected)
            except (OSError,ValueError):
                pass
        paths = specified + [p for p in sorted(list(root.rglob('model.sdf')) + list(root.rglob('*.urdf')), key=lambda p:(
            'backup' in p.parts or 'legacy' in p.parts, len(p.parts), str(p))) if p not in specified]
        if not paths:
            errors[name] = 'No model SDF/URDF under participant directory'
            continue
        failures = []
        for path in paths:
            try:
                document=ET.parse(path).getroot()
                if document.tag=='robot':
                    models[name]=urdf_model(document,path,name,root.resolve())
                else:
                    mdl=document.find('model')
                    if mdl is None:raise ValueError('Not an SDF/URDF model')
                    models[name]=model(mdl,path,name,root.resolve())
                break
            except (OSError, ValueError, AttributeError, ET.ParseError) as exc:
                failures.append(f'{path.relative_to(ROOT)}: {exc}')
        if name not in models:
            errors[name] = '; '.join(failures[:3])
    return models, errors


def main():
    track_name = os.environ['URRC_SPECTATOR_TRACK']
    if track_name not in TRACKS:
        raise ValueError('Unsupported track: '+track_name)
    world_path=PKG/'worlds'/f'{track_name}_race.sdf'
    track_path=PKG/'models'/f'urrc_{track_name}'/'model.sdf'
    world=ET.parse(world_path).getroot().find('world')
    track=ET.parse(track_path).getroot().find('model')
    manifest={'static':[model(track,track_path,'map')]+[
        model(m,world_path,'map') for m in world.findall('model')
        if m.get('name') in ('ground','race_start_gantry')]}
    cars, errors = car_models()
    manifest['cars'] = cars
    manifest['car_errors'] = errors
    active_path=PKG/'worlds'/f'{track_name}_signal_active.sdf'
    active=ET.parse(active_path).getroot().find('model')
    if active is None or active.get('name')!='urrc_start_signal_active':
        raise ValueError('Unexpected active signal model')
    manifest['signal_active']=model(active,active_path,'map')
    names=[v['name'] for l in manifest['signal_active']['links']
           for v in l['visuals'] if v['name'].startswith('active_red_lens_')]
    if sorted(names)!=[f'active_red_lens_{i}' for i in range(1,6)]:
        raise ValueError('Five lenses are required')
    manifest['signal_yaw']=json.loads((PKG/'tracks/processed'/f'{track_name}.json').read_text())['start_finish']['yaw']
    (WEB/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False),encoding='utf-8')
    print(f'[3D 모델] {track_name}: {len(cars)}팀 실제 SDF/URDF 모델 준비')
    for team, reason in errors.items():
        print(f'[3D 모델] {team} 변환 실패: {reason}')
    if not cars:
        raise RuntimeError('No participant visual models were exported')

if __name__=='__main__':
    main()
