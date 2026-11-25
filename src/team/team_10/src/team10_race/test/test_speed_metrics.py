"""Statistics must include initial acceleration and temporary safety stops."""
import importlib.util
import json
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('speed_metrics',Path(__file__).parents[1]/'tools/speed_metrics.py')
metrics=importlib.util.module_from_spec(spec);spec.loader.exec_module(metrics)


def test_temporary_gate_close_does_not_trim_slow_time_and_ties_are_joint(tmp_path):
    (tmp_path/'audit').mkdir()
    poses=[]
    for i in range(81):
        t=i*.05;x=t if t<1 else (1 if t<2 else t-1)
        poses.append(dict(sim=t,wall=100+t,xyz=[x,0,0],yaw=0))
    def save(name,rows):
        (tmp_path/name).write_text(''.join(json.dumps(r)+'\n' for r in rows))
    save('poses.jsonl',poses)
    save('chain.jsonl',[dict(values={'/team_10/camera_gate_10':True,'/team_10/start_status_10':json.dumps(dict(stamp=0))})])
    (tmp_path/'crossings.jsonl').write_text('')
    # Stop request precedes a temporary close; delivery is only terminal at4s.
    (tmp_path/'deliberate_stop.json').write_text(json.dumps(dict(wall=100.9,sim=.9)))
    plans=[];adapters=[]
    for i in range(5):
        plan=dict(current=1.,target=2.,final=1.,unconstrained=4.,race_cap=float('inf'),state='BRAKE',
            source='SCAN_MIDPOINT',candidates=dict(CURVATURE=2.,STEERING=2.,MAX=4.,STATE=4.),
            filtered_steering=.01,preview=5.)
        plans.append(dict(event='plan',sim=float(i),mono=100.+i,wall=100.+i,plan=plan,
            started=True,valid=True,state='DRIVE_SCAN_MIDPOINT',processing_wall_s=.01,
            scan_stamp=float(i),sensor_age_done_sim=.01))
        adapters.append(dict(event='output',sim=float(i),wall=100.+i,started=i not in (1,4),
            speed=1. if i not in (1,4) else 0.,requested=1.,steering=.01))
    save('audit/navigation.jsonl',plans);save('audit/adapter.jsonl',adapters)
    result=metrics.analyze(tmp_path)
    assert result['observed_seconds']==pytest.approx(4.)
    assert result['mean_actual_mps']==pytest.approx(.75)
    assert result['limiter_joint_sim_s']['CURVATURE+STEERING']==pytest.approx(4.)
    assert result['limiter_inclusive_sim_s']['CURVATURE']==pytest.approx(4.)
    assert result['limiter_inclusive_sim_s']['STEERING']==pytest.approx(4.)
    assert result['stop_episodes_0_3s']


def test_one_pose_speed_spike_does_not_count_as_smoothed_ten_mps(tmp_path):
    poses=[dict(sim=i*.01,wall=100+i*.01,xyz=[9.9*i*.01+(.004 if i>=200 else 0.),0,0],yaw=0)
           for i in range(401)]
    (tmp_path/'poses.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in poses))
    (tmp_path/'chain.jsonl').write_text(json.dumps(dict(values={
        '/team_10/camera_gate_10':True,'/team_10/start_status_10':json.dumps(dict(stamp=0.))}))+'\n')
    (tmp_path/'deliberate_stop.json').write_text(json.dumps(dict(sim=4.,wall=104.)))
    result=metrics.analyze(tmp_path)
    assert result['max_actual_sample_mps']>10.
    assert result['above_10_sim_s']==pytest.approx(.01)
    assert result['max_actual_0_2s_mps']<10.
    assert result['above_10_smoothed_sim_s']==0.
    assert result['above_10_smoothed_longest_sim_s']==0.
