#!/usr/bin/env python3
"""Operator-only v26 adaptation of the approved v24 race runner.

Validates exact source anchors and fails closed if the upstream runner changes.
Only official stage_v2 loads this; rehearsal and original participant files unchanged.
"""
from __future__ import annotations
import ast
import time


def once(src, old, new, label):
    if src.count(old)!=1:
        raise RuntimeError(f'[v26] 경주 실행기 호환성 실패 ({label}): 수정 중단')
    return src.replace(old,new,1)



def patch_missing_and_camera(source):
    """Patch the gate failure while preserving unrelated local diagnostics.

    Accept a known camera preflight in the group, or safely add the mandatory
    preflight directly before start_sequence if an older/custom runner omitted it.
    Never bypass the camera check or the official start gate.
    """
    tree=ast.parse(source)
    missing=[]
    for node in ast.walk(tree):
        if not isinstance(node,ast.For):continue
        if not (isinstance(node.target,ast.Name) and node.target.id=='e'):continue
        if ast.unparse(node.iter)!='list(active)':continue
        for condition in node.body:
            if not (isinstance(condition,ast.If) and 'missing' in ast.unparse(condition.test)):
                continue
            records=[n for n in condition.body if isinstance(n,ast.Expr)
                     and isinstance(n.value,ast.Call)
                     and isinstance(n.value.func,ast.Name)
                     and n.value.func.id=='record']
            body=ast.unparse(condition)
            if len(records)==1 and 'retire(e)' in body and 'active.remove(e)' in body:
                missing.append(records[0])
    cameras=[]
    starts=[]
    gates=[]
    for node in ast.walk(tree):
        if not isinstance(node,ast.Expr) or not isinstance(node.value,ast.Call):
            continue
        call=node.value
        if not isinstance(call.func,ast.Name):continue
        if call.func.id=='check_camera_streams_v21' and len(call.args)>=4:
            if isinstance(call.args[3],ast.Name) and call.args[3].id=='active':
                cameras.append(node)
        elif call.func.id=='start_sequence' and node.col_offset==12:
            starts.append(node)
        elif call.func.id=='check_world' and node.col_offset==12:
            gates.append(node)
    lines=source.splitlines(keepends=True)
    if len(missing)!=1 or missing[0].col_offset!=20:
        raise RuntimeError('[운영] 출발 구독 실패 판정을 안전하게 식별하지 못했습니다: '+str(len(missing)))
    record=missing[0]
    replacement='''                    print(f'[운영자] {e["entry_id"]}: 출발 구독/제어기 오류, 수동 DNF 대기',flush=True)
                    operator_session.status(e['entry_id'],'NEEDS_OPERATOR')
                    why=operator_session.wait_dnf(e['entry_id'],check_world)
                    record(e,'운영자 수동 DNF: '+why,1 if mode=='qualifying' else entries.index(e)+1)
'''
    camera_replacement='''            # A camera frame and simulator clock are mandatory before the start.
            from race_camera_ready_v21 import check_camera_streams_v21
            for e in list(active):
                try:
                    cam=check_camera_streams_v21(root,run_dir,env,[e],processes,timeout=60)
                    monitor.watch_camera(e['entry_id'],cam['topics'][e['entry_id']])
                except (RuntimeError,ValueError,OSError,KeyError,subprocess.CalledProcessError,
                        subprocess.TimeoutExpired) as exc:
                    check_world()
                    if processes[e['entry_id']].poll() is not None:
                        print(f'[운영자] {e["entry_id"]}: 카메라 확인 중 제어기 종료, 수동 DNF 대기',flush=True)
                        operator_session.status(e['entry_id'],'NEEDS_OPERATOR')
                        why=operator_session.wait_dnf(e['entry_id'],check_world)
                        record(e,'운영자 수동 DNF: '+why,
                               1 if mode=='qualifying' else entries.index(e)+1)
                    else:
                        record(e,'자동 DNF: 카메라 센서 연결 실패: '+str(exc),
                               1 if mode=='qualifying' else entries.index(e)+1)
                    retire(e);active.remove(e)
            if not active:continue
'''
    if len(cameras)==1:
        camera=cameras[0]
        if record.lineno>=camera.lineno or camera.col_offset!=12:
            raise RuntimeError('[운영] 기존 카메라 검사 위치가 출발 구독 검사보다 앞서거나 중첩 수준이 다릅니다')
        replacements=[(record, replacement), (camera,camera_replacement)]
    elif len(cameras)==0:
        # Some patched runners use another camera preflight or have no direct
        # check_camera_streams call. Never silently skip the safety check.
        # Insert a guaranteed real-frame check directly before actual lights.
        if len(starts)!=1 or starts[0].lineno<=record.end_lineno:
            raise RuntimeError('[운영] 출발 시퀀스 위치를 안전하게 확인할 수 없습니다')
        start=starts[0]
        between=''.join(lines[record.end_lineno:start.lineno-1])
        if 'if not active:continue' not in between or not any(
             record.end_lineno<node.lineno<start.lineno for node in gates):
            raise RuntimeError('[운영] 출발 직전 활성 차량·Gazebo 상태 검사가 없어 자동 패치를 중단합니다')
        # For insertion rather than replacement, the empty range is intentional.
        replacements=[(record,replacement)]
        for node,new in sorted(replacements,key=lambda x:x[0].lineno,reverse=True):
            lines[node.lineno-1:node.end_lineno]=[new]
        # Replacement elements contain embedded newlines, so list indexes
        # still correspond to the ORIGINAL source line numbers.
        lines.insert(start.lineno-1,camera_replacement)
        result=''.join(lines)
        ast.parse(result)
        if result.count('cam=check_camera_streams_v21(root,run_dir,env,[e],processes,timeout=60)')!=1:
            raise RuntimeError('[운영] 카메라 안전검사 삽입 검증 실패')
        return result
    else:
        raise RuntimeError('[운영] 카메라 검사 호출이 복수여서 자동 패치를 중단합니다')
    for node,new in sorted(replacements,key=lambda pair:pair[0].lineno,reverse=True):
        original=''.join(lines[node.lineno-1:node.end_lineno])
        if not original.endswith('\n'):
            raise RuntimeError('[운영] 소스 줄바꿈이 예상과 다릅니다')
        lines[node.lineno-1:node.end_lineno]=[new]
    result=''.join(lines)
    ast.parse(result)
    return result


def monitored(base):
    """Keep global clock/Gazebo faults fatal; isolate per-team pose failures for auto DNF."""
    class V26PoseMonitor(base):
        def __init__(self,*args,**kwargs):
            self.team_errors={}
            self.lidar_received={}
            self.lidar_last={}
            self.camera_last={}
            self.camera_subs={}
            super().__init__(*args,**kwargs)
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import qos_profile_sensor_data
            self._v26_lidar_handles=[]
            for eid,entry in self.entries.items():
                self.lidar_received[eid]=0
                topic=entry['lidar_topic']
                self._v26_lidar_handles.append(self.node.create_subscription(
                    LaserScan,topic,lambda msg,eid=eid:self._lidar_cb(eid,msg),
                    qos_profile_sensor_data))

        def _lidar_cb(self,eid,msg):
            if not msg.ranges:return
            with self.cv:
                self.lidar_last[eid]=time.monotonic()
                self.lidar_received[eid]=self.lidar_received.get(eid,0)+1
                self.cv.notify_all()

        def wait_lidar(self,eid,timeout=30):
            end=time.monotonic()+timeout
            with self.cv:
                while self.lidar_received.get(eid,0)<2:
                    self.check()
                    left=end-time.monotonic()
                    if left<=0:return False
                    self.cv.wait(min(.1,left))
                return True

        def watch_camera(self,eid,topic):
            if not isinstance(topic,str) or not topic.startswith('/'):
                raise ValueError('카메라 토픽 경로 오류: '+str(topic))
            from sensor_msgs.msg import Image,CompressedImage
            from rclpy.qos import qos_profile_sensor_data
            types={t:set(kinds) for t,kinds in self.node.get_topic_names_and_types()}
            choices=types.get(topic,set())
            kind=(CompressedImage if 'sensor_msgs/msg/CompressedImage' in choices
                  else Image if 'sensor_msgs/msg/Image' in choices else None)
            if kind is None:
                raise ValueError(f'{eid}: ROS 카메라 토픽 타입 확인 실패: {topic}')
            def cb(msg):
                if not getattr(msg,'data',None):return
                with self.cv:
                    self.camera_last[eid]=time.monotonic()
                    self.cv.notify_all()
            self.camera_subs[eid]=self.node.create_subscription(kind,topic,cb,qos_profile_sensor_data)
            self.camera_last[eid]=time.monotonic()

        def note_resume(self):
            super().note_resume()
            now=time.monotonic()
            with self.cv:
                for eid in self.entries:
                    if self.timers[eid].active:
                        if eid in self.lidar_last:self.lidar_last[eid]=now
                        if eid in self.camera_last:self.camera_last[eid]=now

        def pose_cb(self,eid,msg):
            super().pose_cb(eid,msg)
            with self.cv:
                problem=self.error
                if problem is not None and str(problem).startswith(eid+':'):
                    self.team_errors[eid]=str(problem)
                    self.error=None

        def wait_seen(self,eid,timeout=30):
            end=time.monotonic()+timeout
            with self.cv:
                while self.timers[eid].previous is None:
                    self.check()
                    if eid in self.team_errors:return False
                    left=end-time.monotonic()
                    if left<=0:return False
                    self.cv.wait(min(.1,left))
                return True

        def team_faults(self,ids,wall_timeout=5):
            from timing import TimingError
            with self.cv:
                self.check()
                now=time.monotonic()
                if now-self.last_clock_wall>wall_timeout:
                    raise TimingError('중앙 시뮬레이션 시계 연결 중단: 전체 경기 무효')
                found={}
                for eid in ids:
                    timer=self.timers[eid]
                    if not timer.active:continue
                    if eid in self.team_errors:
                        found[eid]='base_link 계측 데이터 오류: '+self.team_errors[eid]
                    elif now-self.last_pose_wall.get(eid,0)>wall_timeout:
                        found[eid]='base_link 계측 토픽 연결 중단(5초 이상 미수신)'
                    elif eid in self.lidar_last and now-self.lidar_last[eid]>wall_timeout:
                        found[eid]='LiDAR 센서 ROS 토픽 연결 중단(5초 이상 미수신)'
                    elif eid in self.camera_last and now-self.camera_last[eid]>wall_timeout:
                        found[eid]='카메라 ROS 토픽 연결 중단(5초 이상 미수신)'
                return found
    V26PoseMonitor.__name__='V26PoseMonitor'
    return V26PoseMonitor


def adapt_run(source: str) -> str:
    """Source-level extension of race_manager.run_mode; strict compatibility guards."""
    source=once(source,
        "    slots=grid(root,track);server=bridge=monitor=gate=log=env=None\n",
        "    slots=grid(root,track);server=bridge=monitor=gate=log=env=None\n    operator_session=None\n",
        '세션 생성')
    source=once(source,
        "        out.append(row)\n        print(f\"{e['display_name']}: {duration(row['time_seconds']) if row['status']=='finished' else 'DNF '+reason} / {row['laps']}랩\",flush=True)",
        "        out.append(row)\n        if operator_session is not None:\n            operator_session.status(e['entry_id'], 'FINISHED' if row['status']=='finished' else 'DNF')\n        print(f\"{e['display_name']}: {duration(row['time_seconds']) if row['status']=='finished' else 'DNF '+reason} / {row['laps']}랩\",flush=True)",
        '완주·DNF 상태 기록')
    start='    def prepare_entry(e, slot, position):\n'
    end='    try:\n        server,bridge,env,log=start_world(root,track,event,entries,run_dir)\n'
    if source.count(start)!=1 or source.count(end)!=1:
        raise RuntimeError('[v26] 차량 준비 경로가 예상과 다릅니다')
    i,j=source.index(start),source.index(end)
    source=source[:i]+'''    from operator_v26 import OperatorSession, AutoDNF

    def prepare_entry(e, slot, position):
        eid=e['entry_id']
        try:
            # Operator controls WHEN the existing official spawner creates a car.
            # Its PosePublisher plugin and checked grid coordinates remain intact.
            processes[eid]=launch_algo(e,env,run_dir)
            wait_gate_ready_before_spawn_v21(gate,eid,processes[eid],timeout=30)
            cmd=operator_session.wait_spawn(eid,check_world)
            if cmd['action']=='dnf':
                record(e,'운영자 수동 DNF: '+str(cmd.get('reason') or '운영자 결정'),position)
                retire(e)
                return False
            print(f'[v26] {eid}: 운영자 요청으로 차량 스폰',flush=True)
            spawn(root,track,e,slot,env,run_dir)
            spawned.add(eid)
            operator_session.status(eid,'SPAWNED')
            if not monitor.wait_seen(eid):
                if processes[eid].poll() is not None:
                    raise RuntimeError(f'{eid}: 계측 대기 중 제어기 종료')
                raise AutoDNF(f'{eid}: base_link 계측 연결 실패')
            if not monitor.wait_lidar(eid,timeout=30):
                if processes[eid].poll() is not None:
                    raise RuntimeError(f'{eid}: LiDAR 대기 중 제어기 종료')
                raise AutoDNF(f'{eid}: LiDAR 센서 연결 실패(30초 동안 유효한 LaserScan 2회 미수신)')
            return True
        except AutoDNF as exc:
            check_world()
            record(e,'자동 DNF: 센서·계측 연결 실패: '+str(exc),position)
            retire(e)
            return False
        except (ValueError,ET.ParseError,subprocess.CalledProcessError,subprocess.TimeoutExpired,OSError,RuntimeError,TimingError) as exc:
            # Controller exit, start subscription and model/spawn errors need
            # operator adjudication. They are NOT automatic DNF causes.
            check_world()
            print(f'[v26] {eid}: 자동 DNF 금지 / 준비 오류: {exc}',flush=True)
            operator_session.status(eid,'NEEDS_OPERATOR')
            reason=operator_session.wait_dnf(eid,check_world)
            record(e,'운영자 수동 DNF: '+reason,position)
            retire(e)
            return False

'''+source[j:]
    source=once(source,
        "        gate=StartGate(entries)\n        groups=[[e] for e in entries] if mode=='qualifying' else [entries]\n",
        "        gate=StartGate(entries)\n        operator_session=OperatorSession(root, globals().get('stage',mode), track, entries, run_dir)\n        groups=[[e] for e in entries] if mode=='qualifying' else [entries]\n",
        '운영자 세션 활성화')
    original_camera_block='''            for e in list(active):
                if e['entry_id'] in missing:
                    record(e,'출발 구독 미준비 또는 알고리즘 종료',1 if mode=='qualifying' else entries.index(e)+1)
                    retire(e);active.remove(e)
            if not active:continue
            check_world()
            # v21 START 4/6: real camera frames and ROS /clock before ANY red light.
            check_camera_streams_v21(root,run_dir,env,active,processes,timeout=60)
'''
    new_camera_block='''            for e in list(active):
                if e['entry_id'] in missing:
                    print(f'[v26] {e["entry_id"]}: 출발 구독/제어기 오류, 수동 DNF 대기',flush=True)
                    operator_session.status(e['entry_id'],'NEEDS_OPERATOR')
                    why=operator_session.wait_dnf(e['entry_id'],check_world)
                    record(e,'운영자 수동 DNF: '+why,1 if mode=='qualifying' else entries.index(e)+1)
                    retire(e);active.remove(e)
            if not active:continue
            check_world()
            # Auto DNF only for REAL per-car camera/sensor connectivity failures.
            for e in list(active):
                try:
                    cam=check_camera_streams_v21(root,run_dir,env,[e],processes,timeout=60)
                    monitor.watch_camera(e['entry_id'],cam['topics'][e['entry_id']])
                except (RuntimeError,ValueError,OSError,KeyError,subprocess.CalledProcessError,
                        subprocess.TimeoutExpired) as exc:
                    check_world()
                    if processes[e['entry_id']].poll() is not None:
                        # An exited algorithm is NEVER an automatic DNF condition.
                        print(f'[v26] {e["entry_id"]}: 카메라 확인 중 제어기 종료 / 운영자 DNF 대기',flush=True)
                        operator_session.status(e['entry_id'],'NEEDS_OPERATOR')
                        why=operator_session.wait_dnf(e['entry_id'],check_world)
                        record(e,'운영자 수동 DNF: '+why,
                               1 if mode=='qualifying' else entries.index(e)+1)
                    else:
                        record(e,'자동 DNF: 카메라 센서 연결 실패: '+str(exc),
                               1 if mode=='qualifying' else entries.index(e)+1)
                    retire(e);active.remove(e)
            if not active:continue
'''
    # Accept pre-existing local diagnostics in the official runner without erasing them.
    if original_camera_block in source:
        source=once(source, original_camera_block, new_camera_block, '출발 구독·카메라 연결')
    else:
        source=patch_missing_and_camera(source)
    source=once(source,
'''            deadline=time.monotonic()+timeout
            shown={e['entry_id']:0 for e in active}
''',
'''            deadline=time.monotonic()+timeout
            operator_warning=set()
            shown={e['entry_id']:0 for e in active}
            for e in active:operator_session.status(e['entry_id'],'RUNNING')
''',
        '무응답 알림')
    source=once(source,
'''                monitor.health([e['entry_id'] for e in active])
                completed=[]
''',
'''                sensor_faults=monitor.team_faults([e['entry_id'] for e in active])
                completed=[]
''',
        '개별 base_link 연결 감시')
    source=once(source,
'''                    reason=None
                    if s['finish_time'] is not None:reason=''
                    elif processes[eid].poll() is not None:reason=monitor.failure_reason(eid,'알고리즘 종료')
                    elif time.monotonic()>=deadline:reason=monitor.failure_reason(eid,'제한시간 초과')
                    if reason is not None:completed.append((e,reason))
''',
'''                    reason=None
                    if s['finish_time'] is not None:
                        reason=''
                    elif eid in sensor_faults and processes[eid].poll() is None:
                        reason='자동 DNF: 센서·계측 연결 실패: '+sensor_faults[eid]
                    else:
                        manual=operator_session.requested_dnf(eid)
                        if manual is not None:
                            reason='운영자 수동 DNF: '+manual
                        elif processes[eid].poll() is not None and (eid,'algo') not in operator_warning:
                            print(f'[v26] {eid}: 알고리즘 종료 / 자동 DNF 하지 않음. 수동 명령: bash race_v24.sh dnf {eid}',flush=True)
                            operator_warning.add((eid,'algo'))
                        if time.monotonic()>=deadline and (eid,'time') not in operator_warning:
                            print(f'[v26] {eid}: 제한시간 도달 / 자동 DNF 하지 않음. 운영자가 판단하십시오.',flush=True)
                            operator_warning.add((eid,'time'))
                    if reason is not None:completed.append((e,reason))
''',
        '수동·자동 DNF 판정')
    source=once(source,
'''        if monitor:
            for entry in list(active):
                if any(row['entry_id']==entry['entry_id'] for row in out):
                    continue
                try:
                    record(entry,monitor.failure_reason(entry['entry_id'],error_text),
                           1 if mode=='qualifying' else entries.index(entry)+1)
                except Exception as record_error:
                    print(f'[v13] DNF 기록 실패: {entry["entry_id"]}: {record_error}',flush=True)
        payload.update(valid=False,state='invalid',error=error_text,results=rank_rows(out))
''',
'''        # Host/Gazebo/clock/operator-stop failure invalidates the whole race;
        # it does NOT create individual automatic DNF for unadjudicated cars.
        unadjudicated=[e['entry_id'] for e in active
                       if not any(row['entry_id']==e['entry_id'] for row in out)]
        if unadjudicated:
            print('[v26] 경기 무효 / 운영자 판정 필요: '+', '.join(unadjudicated),flush=True)
        payload.update(valid=False,state='invalid',error=error_text,
                       unadjudicated=unadjudicated,results=rank_rows(out))
''',
        '전체 무효 기록 DNF 오인 방지')
    source=once(source,
'''    finally:
        _v13_cleanup_errors=[]
''',
'''    finally:
        if operator_session is not None:operator_session.close()
        _v13_cleanup_errors=[]
''',
        '세션 종료')
    ast.parse(source)
    return source
