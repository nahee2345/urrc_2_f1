#!/usr/bin/env python3
"""Produce compact QA Markdown and independent odometry-vs-reference SVG."""
import argparse,csv,json,os
from pathlib import Path
def get(path):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):return {}
def main():
    a=argparse.ArgumentParser()
    a.add_argument('--state',required=True);a.add_argument('--reference',required=True)
    args=a.parse_args();s=Path(args.state);d=get(s/'drive_metrics.json');lap=get(s/'lap.json')
    def value(k):
        z=d.get(k)
        return '미검증' if z is None else str(z)
    lines=['# TEAM 01 실제 Gazebo 주행 검증','',
       '- 커밋: '+os.environ.get('GITHUB_SHA','local'),
       '- 독립 완주 판정: **'+str(lap.get('status','미검증'))+'**',
       '- 실측 누적 거리: '+str(lap.get('distance_actual_odometry_m','미검증'))+'m',
       '- 계획경로 추종오차 평가 가능: '+value('planned_path_tracking_evaluable'),
       '- 과거 계획경로 추종오차 평균/P95: '+value('planned_path_error_mean_m')+'/'+value('planned_path_error_p95_m')+' m',
       '- 도로 중심선 편차 평균/P95: '+value('centerline_abs_offset_mean_m')+'/'+value('centerline_abs_offset_p95_m')+' m',
       '- 도로 경계 최소 여유: '+value('road_clearance_min_m')+' m',
       '- 실측 최대속도/P95: '+value('actual_speed_max_mps')+'/'+value('actual_speed_p95_mps')+' m/s',
       '- 2° 이상 좌우 조향 반전 횟수: '+value('steering_reversals_over_2deg'),
       '- 누적 조향 변화량/실주행 거리: '+value('steering_variation_deg_per_m')+' °/m',
       '- 안전한 직선 평균 실측속도: '+value('safe_straight_speed_mean_mps')+' m/s',
       '- 안전한 직선 절대조향 평균/P95: '+value('safe_straight_abs_steer_mean_deg')+'/'+value('safe_straight_abs_steer_p95_deg')+' °',
       '- 안전한 직선 실측/명령 속도비: '+value('safe_straight_actual_over_cmd_ratio'),
       '- 카메라 준비/소등/권한/어댑터 전진: '+ '/'.join(str(d.get(k,'미검증')) for k in ('camera_ready','camera_lights_out','permission','adapter_nonzero')),
       '- ROS bag: '+('기록 확인' if (s/'rosbag_core'/'metadata.yaml').exists() else '자료 없음'), '',
       '## 안전 제동·정지 사유']
    lines.extend('- '+k+': '+str(v)+'회' for k,v in sorted((d.get('fault_reasons') or {}).items(),key=lambda x:-x[1])[:15])
    lines+=['','도로 중심선 편차는 의도된 차선 유지와 구분합니다. 5m 미만 이동 시 경로 정확도는 미검증입니다.',
            '직선 구간에서는 강제 중앙복귀를 성능 향상으로 간주하지 않으며, 안전여유·조향·속도를 함께 비교합니다.']
    size=get(s/'source_line_limits.json')
    lines+=['','## 파일당 400줄 제한','- 판정: '+str(size.get('status','미검증'))]
    for item in size.get('violations',[]):
        lines.append('- '+item['path']+': '+str(item['lines'])+'줄 (초과 '+str(item['excess'])+'줄)')
    (s/'qa_run_report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    try:
        from team01_race.lap_validation import load_reference
        ref,_=load_reference(args.reference)
        with (s/'trajectory.csv').open(encoding='utf-8',newline='') as f:
            actual=[(float(z['x_m']),float(z['y_m'])) for z in csv.DictReader(f)]
        allp=[tuple(z) for z in ref]+actual
        lo=[min(p[i] for p in allp) for i in (0,1)]
        hi=[max(p[i] for p in allp) for i in (0,1)]
        scale=min(730/max(1,hi[0]-lo[0]),520/max(1,hi[1]-lo[1]))
        def coords(p):return f"{35+(p[0]-lo[0])*scale:.1f},{570-(p[1]-lo[1])*scale:.1f}"
        r=' '.join(coords(p) for p in ref);a=' '.join(coords(p) for p in actual)
        svg=f'<svg xmlns="http://www.w3.org/2000/svg" width="820" height="615"><rect width="820" height="615" fill="white"/><polyline points="{r}" fill="none" stroke="#a3a3a3" stroke-width="2"/><polyline points="{a}" fill="none" stroke="#2563eb" stroke-width="2.5"/><text x="20" y="20" font-size="13">Grey: Monza centerline / Blue: actual Gazebo odometry</text></svg>'
        (s/'trajectory_overlay.svg').write_text(svg,encoding='utf-8')
    except (OSError,ImportError,KeyError,ValueError) as e:
        (s/'trajectory_plot_error.txt').write_text(str(e),encoding='utf-8')
if __name__=='__main__':main()
