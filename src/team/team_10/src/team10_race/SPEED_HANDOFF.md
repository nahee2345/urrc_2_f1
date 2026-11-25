# 속도 작업 인계 — 2026-11-23 최종 갱신

## 현재 선택본

- 소스 `src/team/team_10/`, 실행 prefix `install_f1/team10_race`. Python/config/model은 소스 연결이며 실제 import 경로 확인.
- **선택본 `latest_scan_final_lap`: 실제 카메라 출발→gate0→1…66→0 전체완주**,226.453495sim초. 출발포함 평균2.933369, 원시최고10.167571/약.2초평활10.104984m/s.
- 실제>=10 총1.321sim초/전체.57905%/최장.599초. 평활총1.056초/.46289%/최장.574초. **도달·누적시간은 늘었지만 장시간 연속 유지 목표는 미달.**
- 광학 구간76.95→305.092470sim초, pose유효228.131초. 게이트 랩78.638976→305.092470. 출발 가속·중간감속 모두 포함. 원래 preview 검증이며 공식 기록 아님.
- 접촉/다른물체충돌/도로이탈(중심·중점)/.3초 정지/역주행0. 단독 시험이며 다차량 무충돌 미검증.
- 최종 false 실제2.6433→정지.80866sim초/.80943m. 실제0/프로세스 정리 완료, 호스트 잔여 시험 없음.
- 최종22파일 SHA 모두 일치: `latest_scan_final_lap/current_identity_verified.json`. 전체복원 `selected_latest_scan_final/source_full/`, `sha256.json`. 빠른 이전 기준248.99본과commit254.78본 모두 보존.

| 전체 랩 | 게이트 랩sim초 | 출발 포함 평균m/s | 원시/약0.2초평활 최고m/s | 실제>=10 총sim초/비율/최장sim초 |
|---|---:|---:|---:|---:|
|final_lap 기준|248.995|2.679948|7.513757/7.453626|0.000/0.0000%/0.000|
|commit_final_lap 재개본|254.778|2.616564|9.006154/8.861264|0.000/0.0000%/0.000|
|latest_scan_final_lap 선택|226.453|2.933369|10.167571/10.104984|1.321/0.5791%/0.599|

표의 평균은 출발 포함이며 기준2.69는 게이트 랩 평균이다. 평활/원시 >=10은 정확히10.0 기준, 튐을 목표 달성으로 처리하지 않는다. 전체랩과 별도 구간은 섞지 않는다.

## 확인한 원인·최종 변경

- 유일한 주행 변경: `navigation_node.py` primary LaserScan KEEP_LAST depth5→1(BEST_EFFORT 유지). 다른 제어·경로·single executor는commit완주본 그대로.
- 같은 scan의 독립depth1 기록기는 생성→콜백중앙.01277/p95.02137sim초, 기존navigation은.13485/.17018초. 큐 적체 근거에 따라 최신 입력만 사용했다. `delay_resume/input_queue_evidence.json`.
- 전체랩 센서→계획완료 p95.22625→.10832sim초, 센서→adapter p95.26420→.15387/max.25953초. 처리wall p95.13132→.12912초로 유사. 최신scan으로 입력 나이가 줄었음.
- 고속211→470표본: 센서→완료중앙.17346→.05510sim초. 그동안 전방이동중앙1.1192→.4214m. 원래목표 거리2.8402→3.3479m, 완료시같은목표1.7424→2.8897m, adapter시1.6542→2.6930m. 회전·레이저 레버암 포함평면SE(2), 단순v×age와 구분. pitch/roll 미기록/궤적분포 차이 한계 있음.
- header와/clock은sim시간, wall/mono는host. 완료·출력을 독립clock으로 재정렬. middleware 자체수신 시각 없음/콜백진입 복원이며, 늦은로컬clock을 실제완료로 오인하지 않는다. adapter연결은sequence ID 없는 관측매칭(상한보장아님).
- 경로/목표는 여전히 센서시점 레이저좌표. 온라인 정답pose/게이트/맵 입력 없음. 관측영역/경로/fullbody 제동 검사는 보존.
- planner/adapter/model 상한10.1, gain.40/maxL4.4, accel3/decel3/reaction.40, preview22/reserve.55/brake margin.60, lat3.0, 감속완료 latch·lease·freshness·명령만료·출발/false/영상정지 그대로. 모델크기/질량/마찰/steering gain 변경 없음.

## 완료 검증·실패 기록

- 최종201검사/빌드PASS `delay_resume/queue_tests.txt`, `queue_build.log`.
- 고속712/실패369 저장입력 명령·상태가commit후보와 전부동일. `queue_replay_comparison.json`. 고정입력은 변경 궤적/큐 선택의 폐루프 재현이 아니다.
- `latest_scan8`: 평활8.000743, 접촉/이탈/정지/역주행0. false4.5978→정지1.25008sim초/2.51773m.
- `latest_scan10 --until-gate43`: gate40 이후통과, 출발포함150.0sim초/평균3.009966, 원시10.318710/평활10.140670. >=10 총1.477초/.98467%/최장.899초, 평활1.376초/최장.843초. 접촉/이탈/정지/역주행0. false8.6295→정지2.49969sim초/11.04172m. 실제정지/정리완료.
- 위주행15파일·설정 그대로 최종전체랩1회 완료. 6단계/기준전체랩은 반복하지 않았다.
- 철회한 모델정렬후보:204검사/빌드PASS, 고속재생32/712적용·최고명령9.5367→9.1268로 개선미확인. `delay_align8`은3.6943m/s 이하/보정비활성에서 NO_CENTERLINE 지속정지(접촉/이탈0). 전체랩으로 진행하지 않았고소스/증거 `selected_delay_candidate/`, `delay_align8/` 보존. 명령모델 yaw오차max.0773rad를 보장으로 취급하지 않음.
- 이전tail25/gain.50/accel4조합 재생미채택/설정미혼입: `commit_final_lap/next_candidate_ablation.json`. 옛gate40실패 `resume_final_lap`과 감속완료수정증거도보존.
- false평가에서 명령0과실제정지를 구분하도록 실제속도로 대상선정 수정. 임시false와최종false는분리. `delay_resume/false_evaluation_correction.json`.

## 남은 병목·다음 최소 작업

- 최종nav 적용 배타시간: CURVATURE103.4477(45.34%), VISIBILITY71.9050(31.52%), BRAKE_COMMIT19.3944, 공동BRAKE_COMMIT+VISIBILITY20.5893, STEERING7.7360sim초. 비율을 인과적 손실 기여도로 단정하지 않는다. 상세 공동/가감속 사유는 `speed_metrics.json`.
- adapter실질clamp/timeout0, steer포화0초/추종절대오차p95.02415rad. 10m/s 관측제동길이21.2167m/10.1은21.5917m라22m preview 여유가 작다.
- 실제 정상감속전체2.8504~3.4280/중앙3.0167m/s²(구배/짧은회귀 포함). 10구간의10.1086 시작감속표본2.9958. 모든조건최소제동 또는10→0별도긴급제동보장으로 확대해석하지 않는다.
- 다음은 저장고속입력의 CURVATURE tier와VISIBILITY 최초제한을분리해, 실제급커브/표본곡률노이즈/가시성 부족을 확인한다. 근거있는한가지후보만비교하고 급커브/fullbody/.40반응/제동여유를보존한다. 불확실한모델정렬·광범위gain/accel탐색반복금지.

## 해시·공간·증거

- navigation_node SHA256 `2f4924838c5ed68b7d88a4c1a60f9b882fd8f4ee13b36b87f565377028c1a443`
- config SHA256 `b25328fd127a4a647edee467690b55cf282f763ba502463f70e37a208d25fe03`
- model SHA256 `9407dea12d07615d25e0a7b1654ff64d16a658503293e1868e23db160482b915`
- 원본 `source_before/`, `final_lap/`, `selected_commit/`, 실패/성공 스냅샷 보존. 운영자155파일SHA전부동일 `delay_resume/operator_match.json`. 사용자변경/기존.gitkeep삭제상태 유지.
- 수동공간확보후4.9GB/inode854만→종료4.4GB/inode854만. 증거삭제없음. 기록예상전체256MB, 시작1GiB/1만inode, 기록중512MiB reserve/필수6파일5초갱신감시. 실패시false정지우선.
- 상세 REPORT `validation_f1/speed_20261123/REPORT.md`. 최종증거 `latest_scan_final_lap/`의 metrics/speed_metrics/speed_response/delay_geometry_analysis/integrity/runtime_identity/stopped/cleanup 및 원시scan/pose/clock/contact/audit. 비교 `delay_resume/performance_comparison.json`, 그림speed_evidence.png.

## 전체 실행

```bash
cd ~/urrc_2
source /opt/ros/jazzy/setup.bash
bash src/team/team_10/tools/build_f1.sh
python3 src/team/team_10/tools/run_speed_trial.py --output validation_f1/speed_20261123/new_lap --lap
# 별도 터미널에서 실제 Gazebo 도로·그리드·신호등 확인 후:
touch ~/urrc_2/validation_f1/speed_20261123/new_lap/VISUAL_READY
```
기존 폴더명은 재사용하지 않는다. 감독기가 센서 검사→준비 허가→최종 검사→원래 READY→실제 카메라 출발을 수행한다. 종료 시 false→실제0 확인→자기 프로세스 정리. 수동 개별 터미널 명령은 `src/team/team_10/RUN_GUIDE.md`.

