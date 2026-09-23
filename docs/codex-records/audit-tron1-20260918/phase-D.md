# Phase D — safety/mobility 제약의 정적 검토

전체 감사는 AUDIT_INCOMPLETE다. repository 기준 경로는 coverage.json과 같다. 모든 값은 파일에 정의된 값이며 live ROS parameter 조회 결과가 아니다. 공식 noetic-devel source의 URL 및 hash는 upstream/manifest*.json에 보존했다. 설치 binary와 이 branch의 일치 여부는 UNVERIFIED다.

## 읽은 파일과 증거 경계

production navigation.launch, nav YAML 4개, robot/stair profiles, supervisor/evidence/mission state/entry gate를 재대조했다. docs/stair-up-commissioning.md를 전체 열람했고 production PGM 5개는 요청 범위인 header metadata만 확인했다(map-metadata.json). 이전 hash 획득을 pixel 해석으로 계산하지 않는다. 공식 costmap/AMCL/move_base/navfn source의 관련 함수들을 읽었다.

확정 사실 OBSERVED: 아래 설정·공식 알고리즘·로직 반례. INFERRED: 이 조합이 만드는 통행성과 실패 경로. UNVERIFIED: 실제 기체 외곽, 부착물, 제동, localization 오차, 통로 실측, scan rate/jitter, firmware 정지 반응. 아직 측정 근거를 충족한 KEEP은 없다.

## Navigation margin 계산

근거: src/multifloor_manager/config/nav/costmap_common_params.yaml:1-4, local_costmap_params.yaml:1-26, global_costmap_params.yaml:1-14; 공식 footprint.cpp:138-166, costmap_2d.cpp:181-185, inflation_layer.h:104-119 및 inflation_layer.cpp:300. 계산 결과: navigation-geometry-model.json.

| 항목 | 저장소/공식 구현에서 확인한 값 | 해석·분류 |
|---|---|---|
| robot_radius | 0.28m, 직경 0.56m | PHYSICAL_COLLISION_BOUNDARY의 설정 후보; 실측 검증은 UNVERIFIED |
| footprint_padding | 좌표 축마다 부호에 따라 0.02m | OPTIONAL_MARGIN 후보; 단순 반지름 0.30m 원과 동일하지 않음 |
| padded footprint | 공식 16각형 모델 내접 약 0.2940m, 외접 약 0.3083m | 방향별 외곽 차이; 실제 몸체 보증 아님 |
| inflation_radius | 0.33m, 0.05m grid에서 ceil→7cell, 축방향 최대 0.35m | 추가 비용 영역; 전체를 금지 영역으로 취급하면 잘못 |
| cost scaling | 4.0; 축거리 0.25m cost253, 0.30m cost246, 0.35m cost201 | cost253과 그 바깥 soft cost를 구별; COMFORT_MARGIN 후보 |
| grid | global/local 0.05m | cell 하나=5cm. 양 벽의 표현 오차가 통행 판정에 영향을 줌 |
| local 관측 | obstacle 1.3m, raytrace1.4m, 4×4m rolling odom map | 감지·clearing 범위. 제동거리와 실제 scan coverage 실측 필요 |
| global 관측 | map의 static+inflation, 1Hz | live obstacle은 global layer에 없고 local에서 처리 |
| local 갱신 | obstacle+inflation 5Hz, publish2Hz | publish 속도와 실제 update 속도 구분 |

INFERRED 계산 경계: 이상적인 연속 평행벽에서 footprint 전폭은 자세에 따라 약 0.588~0.617m이며 이 값과 실측 여유를 비교해야 한다. 이 숫자는 grid planner가 보장하는 최소 통로폭이 아니다. 점 장애물 cell center를 기준으로 하는 Navfn 금지 비용과 local polygon collision 검사는 서로 다르다. 0.66m보다 좁으면 inflation이 겹쳐 무조건 통행 불가라는 결론은 성립하지 않는다. Navfn은 inscribed/lethal을 차단하고 그 아래 비용은 경로 비용으로 처리한다(navfn.cpp:228-259). 실제 PGM에서 특정 좁은 통로의 plan 성공은 아직 UNVERIFIED다.

물리 외곽·오차 근거가 확보되지 않아 radius가 과도하거나 부족하다고 단정할 수 없다. 따라서 임의 축소 권고나 UNJUSTIFIED_MARGIN 확정은 보류한다. 여유 폭·부착물 실측과 현재 지도에서의 격리 planner 검증이 먼저다.

## 속도·시간·복구 제약

OBSERVED: base_local_planner_params.yaml:1-20과 navigation.launch:47-74를 대조하면 launch 인자가 YAML 속도를 덮어쓴다. production 상위 설정의 x=0.30m/s, yaw=0.8rad/s, min-in-place=0.25rad/s, ax=0.4m/s², aθ=2.0rad/s² 경로를 기준으로 봐야 한다. yaml의 yaw1.57/min-in-place0.4/ax0.6만 보고 runtime 값을 판단하면 안 된다. live override는 UNVERIFIED다.

모형상 0.3m/s에서 감속0.4m/s²이면 제동거리 v²/(2a)=0.1125m이고 5Hz 한 cycle 이동은 0.06m다. 이는 firmware 감속·네트워크 지연·미끄러짐을 포함하지 않아 실제 안전거리 증명이 아니다. sim_time1.2s의 최고속 직선 길이는 0.36m. goal xy0.25m/yaw0.2rad와 latch_xy=true, planner patience5s, oscillation10s/0.2m가 설정돼 있다. 이 값들이 과도하다는 빈도 증거는 미확인이다.

공식 move_base.cpp:901-990은 recovery disabled 시 CLEARING에서 abort로 이어짐을 보여준다. 이것은 작은 오류가 항상 abort된다는 뜻은 아니다. planner/controller의 재시도·mission의 제한 재시도와 구별한다. 평면에서만 bounded recovery를 검토하고 계단 인근에서 임의 회전 recovery를 일괄 활성화하는 개선은 제안하지 않는다. 또한 min_vel_x=0이나 recovery=false가 local planner의 모든 후진 경로를 제거한다는 뜻도 아니다: 공식 trajectory_planner_ros.cpp:223-228은 별도 기본 escape velocity를 로드한다. 실제 escape 경로는 E에서 더 대조한다.

## 새 조사 후보 D-01 — scan freshness와 costmap current가 같은 보장이 아님

Safety: S2. Mobility: M1. OBSERVED 기본값 결합, INFERRED 보호 공백의 조건부 경로. 실제 blind driving 발생은 UNVERIFIED.

local_costmap_params.yaml:14-23은 expected_update_rate를 지정하지 않는다. 공식 obstacle_layer.cpp:96-97 기본값은 0이고 observation_buffer.cpp:231-244는 이때 isCurrent를 항상 true로 반환한다. observation_persistence=0 역시 '나이 0만 허용'이 아니라 최신 관측 하나를 보존한다(:204-216). move_base.cpp:829의 current 검사는 이 설정에서 scan 나이를 별도로 증명하지 않는다. mission ros_state.py:151-166와 supervisor.py:95-109도 scan 수신 나이를 감시하지 않는다.

단, scan 상실은 AMCL map→odom 갱신 및 TF freshness 실패로 간접 정지를 유발할 수 있으므로 무한 blind driving으로 단정하지 않는다. costmap_2d_ros.cpp:546-593은 robot pose TF age도 검사한다. 독립적인 TF가 계속 갱신되는 경우 및 짧은 gap의 정확한 정지 시간은 미검증이다. 최소 개선 후보: FLAT_NAV에 필요한 scan의 측정된 gap budget을 기존 observation buffer에 적용하고, fresh sample 회복 시 자동으로 current가 회복되도록 한다. 아주 짧은 임계나 전체 subsystem FAULT latch를 추가하지 않는다. 검증: TF는 정상인 상태에서 scan만 일시/장기 상실, 회복 후 재개, 통신 jitter 정상분포.

## C-04 추가 증명

OBSERVED: stair_profiles.yaml:16-21은 정지 tolerance 0.50m/0.35rad, 표본 불연속 상한 0.10m/0.20rad다. stair_evidence.py:110-125에서 수용된 표본의 이동량은 :184-185 정지 기준보다 항상 작다. 따라서 수용된 이동 표본으로 정지 조건을 반증할 수 없는 구성이다. 별도 finding을 만들지 않고 C-04 Safety S4 / Mobility M2에 통합한다. 절대 누적 이동·속도·독립 관측으로 정지/landing을 확인해야 하며 수치만 임의로 낮추는 것은 해결 증명이 아니다.

## 단계 종료 기록

읽은 파일은 위 목록과 coverage.json에 기록했다. D의 정적 gate/기하 계산 검토를 완료했다. bag jitter·commissioning evidence, 실제 margin 근거와 전체 제약 누락 여부는 E에서 교차 검증해야 하므로 D의 최종 판정은 아직 열려 있다. 새 조사 대상: scan-current 계약, escape velocity, bag 정지/계단참 증거, startup map/floor/pose/anchor 원자성. 다음 순서: E 테스트·문서·보안과 evidence → D 판정 보강 → F 15개 E2E. KEEP 0, 모든 E2E PENDING 유지.
