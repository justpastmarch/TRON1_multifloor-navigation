# 새 장소 2D 지도 GUI 및 단일 주행 A/B 비교 구현

2026-10-21. 상태: 코드 적용 및 격리 ROS/BAG/브라우저 호환성 검증 완료. 새 넓은 장소에서의 실주행 품질 검증은 미실시.

## 실행과 사용

바탕화면 `TRON 지도 관리` 아이콘을 연다. 기존 주행 프로그램과는 동시에 실행하지 않는다. 기존 프로그램을 자동 종료하거나 임무를 취소하지 않는다.

터미널 대체 명령:

```bash
cd ~/Desktop/TRON1_Control/TRON1_RViz_Navigation
bash run.sh --mapping
```

UI 주소는 http://127.0.0.1:8766/ 이다. 아이콘으로 실행하면 UI가 자동으로 열린다. 실행 터미널은 유지하고 프로그램 종료는 Ctrl+C다. 일반 주행용 `TRON 실행` 아이콘은 기존대로 유지했다. 주행 UI 상단에도 지도 관리 화면 링크를 추가했다.

1. 새 장소 이름을 입력하고 `연결·작성 시작`.
2. 물리 조종기로 한 번 주행. SLAM Toolbox 지도가 표시되며 /scan·wheel odom·TF만 비교용 BAG으로 한 번 기록한다.
3. 원하는 시점에 `지도 저장`. PGM·YAML과 SLAM Toolbox posegraph/data를 저장한다. 저장 후 작성은 계속된다.
4. `작성 종료·BAG 저장`으로 기록을 마감한다.
5. 장소 기록을 선택하고 `이 기록으로 두 지도 만들기`.
6. 같은 기록을 SLAM Toolbox와 Gmapping에 각각 재생한 결과를 나란히 확인·다운로드한다.

매핑 결과는 새 장소 라이브러리에만 저장한다. 기존 3F/5F/RF 지도·계단 진입점·촬영 포인트를 변경하지 않는다. 1TB 디스크가 /mnt/ethan에 마운트돼 있으면 /mnt/ethan/tron-bags/mapping, 아니면 ~/TRON1_maps/mapping에 저장한다. MAPPING_OUTPUT_ROOT로 지정할 수 있다.

## 실제 연결

- 기존 mission_manager 실행 파일의 mapping 모드. 임무 action server·AMCL·move_base·층 전이·Stair Supervisor를 시작하지 않는다.
- 기존 미니PC tron1-sensors.service 재사용. 기본 실행은 ROS master를 준비하고 센서 서비스 시작을 요청한다. 센서 연결 실패로 UI 자체를 숨기지 않는다.
- 기존 MID360 /scan 및 odom→base_Link TF 사용. 새 LiDAR driver·중복 scan 변환기를 추가하지 않았다.
- 공식 SLAM Toolbox async_slam_toolbox_node, 설치 버전 1.5.7 사용. 현재 구현은 실제 설치 버전의 /map 절대 토픽을 /mapping/map으로 remap하고 /mapping/mapper/serialize_map을 호출한다.
- SLAM Toolbox 설정을 명시적으로 로드하며 loop closing=true, 해상도 0.05m, 최대 30m, base_Link/odom/map frame을 사용한다.
- 기록은 /scan, /tron/wheel_odom_raw, /tf, /tf_static만 포함한다. 카메라·depth·raw LiDAR 중복 구독은 추가하지 않는다. 원본 3D 재처리용 BAG이 필요한 경우는 별도 기능이다.
- 프로그램에는 속도·로봇 모드·임무 명령 송신 경로가 없다. 지도 작성 중 이동은 물리 조종기로 한다.

## 한 번 주행으로 두 방식 비교하는 방식

주행 중에는 기존 SLAM Toolbox 미리보기와 한 개의 입력 BAG만 만든다. 종료 후 두 방식은 CPU 경쟁과 TF 충돌을 피하기 위해 순서대로 실행한다. 추가 실주행은 필요하지 않다.

입력 BAG에서 scan·odom·TF를 추출하되 이전 SLAM/AMCL의 map 관련 TF는 제거한다. 두 worker는 같은 임시 BAG을 읽고, 각각 별도 loopback ROS Master를 사용한다. 운영 ROS Master·로봇과 통신하지 않는다. static TF의 latching을 보존한다.

비교군은 설치돼 있고 실제 실행 검증한 Gmapping이다. 넓은 공간에서 Gmapping이 우수하다는 추천·판정이 아니다. Cartographer는 미설치이며 이번 검증에 포함하지 않았다.

알고리즘별 PGM·YAML, Toolbox graph, 설정, 처리 로그 및 비교 입력 SHA256을 남긴다. 지도 미리보기는 각자 관측된 영역에 맞춰 표시하며 원점·방향이 서로 다를 수 있다. 두 이미지의 자동 겹침 정렬이나 절대 정확도 계산은 구현하지 않았다.

## OBSERVED: 실제 검증 결과

### 실제 알고리즘 호환성

기존 옥상 평지 BAG ui_test_20261016_094003_2cd7ba7f.bag의 앞 45초를 사용했다. 원본 BAG은 읽기만 했다.

- 입력 scan 437개, 구간 44.938초.
- 두 worker 입력 감시 구독자 모두 437개 수신. 이 수치는 알고리즘이 내부적으로 사용한 scan 수나 처리 손실 0을 입증하는 값은 아니다.
- 동일 비교 입력 SHA256: ec9d48ca42e362b3aa844821a6905a784e5d36562cfefe6e7926081aca55bac6.
- SLAM Toolbox: 339×238 셀, 0.05m/셀, 지도 및 그래프 저장 성공. worker 처리 시간 약 51.0초.
- Gmapping: 4000×4000 셀, 0.05m/셀, 지도 저장 성공. worker 처리 시간 약 51.4초.
- Gmapping grid의 큰 크기는 초기 미관측 영역을 포함한다. 지도 크기를 더 넓게 매핑했거나 더 정확하다는 의미로 해석하지 않는다.

### 실제 브라우저와 ROS 입력 연결

운영과 분리한 127.0.0.1:19466 Master 및 18766 UI를 사용했다.

1. 브라우저 버튼으로 작성 시작 → 실제 BAG 센서 입력 → 지도·그래프 저장 → 작성 종료. PASS.
2. GUI가 직접 기록한 BAG을 A/B 버튼으로 비교. 8.338초/83 scan 입력으로 Toolbox·Gmapping 결과 모두 생성. PASS.
3. 저장 그래프를 UI 이어쓰기 버튼으로 선택. Toolbox의 실제 Load From File 로그 확인 후 지도·그래프 재저장. PASS.
4. 두 지도 미리보기 표시. PASS.
5. 브라우저 JavaScript 오류 0, 모바일 가로 넘침 없음. PASS.
6. 인증 없는 변경 요청 거절, 동일 request_id 재요청 같은 job 반환, 다른 작업에 request_id 재사용 거절. PASS.
7. 센서 없는 시작 대기 중 중단 요청 → 대기 종료 및 작업 잠금 해제 약 0.21초. PASS. 모든 종류의 작업 중단 지연을 대표하는 수치는 아니다.
8. 기존 콘솔 테스트 12개, 새 저장/입력 필터 테스트 5개, 기존 실행 스크립트 테스트 11개, 운영 노드 구성 테스트 5개 통과. 총 33개.
9. validate_bundle.py 및 --validate-production-config: BUNDLE_OK.

## 저장·이어쓰기의 범위

PGM·YAML만 저장하면 매핑 이력을 이어갈 수 없으므로 Toolbox graph.posegraph와 graph.data도 저장한다. Gmapping 결과에는 이 GUI의 그래프 이어쓰기 기능이 없다.

이어쓰기는 사용자가 로봇을 해당 그래프의 첫 기록 위치·방향에 배치하고 확인한 뒤 시작한다. 임의 위치에서 자동 재측위하거나 서로 다른 장소의 지도를 병합하는 기능은 아니다. GUI에 이 제한을 명시했다.

현재 실시간 grid snapshot을 저장한 후 live graph를 serialize하므로, 이동 중 저장하면 두 파일 세트의 획득 시각이 완전히 같다고 보장하지 않는다. 정확히 같은 상태를 보존하려면 잠시 조종을 멈춘 뒤 저장한다. 원본 입력 BAG은 별도로 남는다.

## 한계 / UNVERIFIED

- 넓은 새로운 실내·실외 공간에서 지도 정합성이 실제로 개선되는지는 아직 검증하지 않았다.
- 독립 기준점·실측 없이 정합 점수, 점유 셀 수, 루프 폐쇄 성공만으로 정확도를 주장하지 않는다.
- 기존 높이 필터와 평면 odom deskew를 재사용한다. 경사·큰 roll/pitch·특징 없는 야외·바퀴 slip에 대한 새 보정 알고리즘을 구현한 것은 아니다.
- 기존 BAG 호환성 성공은 새 장소에서 물리 주행 성공이나 장거리 loop closing 정확도 보증이 아니다.
- 운영 지도 자동 적용, 기존 미션 좌표 변환, Cartographer 비교, 자동 대규모 품질 점수는 이번 새 장소 매핑 연결 범위에 포함하지 않았다.

## 수정·검토·정리

blind code review와 실제 ROS 실행으로 다음 문제를 찾아 수정했다: 불필요한 odom subscriber 대기, absolute map remap, 실제 serialize 서비스명, 결과 폴더 이동 후 YAML image 참조, 모바일 가로 넘침, 매핑 비정상 종료 후 잔류 기록기 정리.

runtime source는 repository의 src/mission_manager/src/mission_manager/mapping_*.py, 화면은 config/mapping_console.html, 실행은 run_mapping.sh다. 기존 FSM이나 ROS action 메시지는 변경하지 않았다. 비교 worker만 임시 ROS client 초기화를 사용한다. bundle contract에는 이 일회성 worker를 명시했다.

기존 config.env 및 .omo의 사전 변경은 유지했다. 커밋·푸시는 수행하지 않았다. 테스트 Master·GUI·알고리즘·재생 프로세스는 종료 시 정리하고, 재현 로그·검증 지도·BAG excerpt·화면 캡처는 이 보고서의 evidence로 의도적으로 보존한다.

검증 증거: /home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/mapping-gui-implementation-20261021 (compatibility.log, ui-live/compare/resume-result.json, 회귀 로그, gui-desktop.png, gui-mobile.png).

[지도 전용 OpenAPI 명세](mapping-api.openapi.json).
