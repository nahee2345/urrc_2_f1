# URRC `urrc_22` — 자유 맵 관전 + 모든 차량 Follow

`urrc_22` 공식 대회 운영용 **읽기 전용 관전 패치**입니다. 팀 주행 코드, 맵 모델, 대회 계측·DNF 기능은 수정하지 않습니다.

## 설치

```bash
unzip -oq 'urrc_22_관전.zip' -d ~/Downloads
bash ~/Downloads/urrc_22_spectator_patch/install.sh ~/urrc_22
```

## 운영 PC에서 관전 서버 실행

```bash
cd ~/urrc_22
bash spectate.sh
```

터미널에 출력되는 `http://운영PC_사설IP:8765/?key=...` 주소를 같은 Wi-Fi 참가자에게 전달합니다. 참가자 컴퓨터는 브라우저만 있으면 됩니다. 별도 Gazebo 설치는 필요 없습니다.

## 관전 조작

- **접속 직후** 모든 참가자가 전체 맵을 위에서 자유롭게 확인합니다. 자기 차량으로 자동 Follow하지 않습니다.
- **왼쪽 마우스 드래그:** 맵 이동.
- **마우스 휠:** 커서 중심 확대·축소.
- **오른쪽 마우스 드래그:** 지도 회전(관전용 화면만 변경).
- **전체 맵:** 전체 코스가 화면 안에 들어오도록 맞추고 Follow 해제.
- **자유 관전:** Follow 해제, 화면은 차량 위치 중심에 놓입니다.
- **차량 선택:** `team_01`~`team_12`의 현재 스폰되어 주행 중인 모든 차량 중 하나를 선택. **자기 팀 제한 없음**.
- **맵의 차량 아이콘 클릭 / 순위표 행 클릭:** 해당 차량 Follow 시작.
- **Follow 시점:** 후방 또는 상공.
- **Follow 중 드래그:** 자유 관전으로 전환.
- **Follow 중인 차량이 DNF 또는 제거됨:** 남아 있는 차량 중 **현재 잠정 1위 차량**으로 자동 Follow 전환. 차량이 없으면 자유 관전으로 유지.
- **경기 전:** 6개 맵 미리 보기 가능. 경기 진행 중에는 해당 경기 맵만 표시.

## 범위 및 제한

- 현재 화면은 대회 맵의 **트랙 중심선 및 도로 폭에 기반한 경량 2D 자유 관전 / 간략 3D Follow**입니다. Gazebo 실제 벽·연석·지형·텍스처는 렌더링하지 않습니다.
- 관전 화면에서 스폰, DNF, 차량 제어, 맵 변경은 할 수 없습니다. 모든 서버 API는 기존 기존 **인증된 읽기 전용 GET**입니다.
- 경기 라이브 관전에는 기존 운영 기능에서 생성하는 경기 상태 파일 `race/results/operator_v26_active.json`과 `race/results/runs/*/timing.jsonl` 생성 및 `base_link` 계측이 필요합니다. 관련 운영 기능 미설치 또는 미실행 시에는 맵 미리 보기만 가능합니다.
- 참가자가 같은 Wi-Fi에서 ROS 2/Gazebo 통신 서비스에 직접 연결할 수 없게, 운영 PC를 방화벽 또는 별도 VLAN으로 격리해야 안전합니다. 관전 포트 TCP 8765만 허용하십시오. 링크 키를 외부에 공개하지 마십시오.
- 공식 랩 판정이나 순위를 변경하지 않습니다. 관전용 순위는 잠정 순위입니다.

## 자체 검증

```bash
cd ~/urrc_22
python3 -m unittest discover -s race/tests -p 'test_spectator_core.py' -v
python3 -m unittest discover -s race/tests -p 'test_spectator_ui.py' -v
```

실제 원격 PC 및 Gazebo 다중 차량 주행은 대회 Wi-Fi에서 추가 확인이 필요합니다.
