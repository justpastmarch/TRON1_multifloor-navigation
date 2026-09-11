# 5F/RF commissioning status

이 문서는 다음 세션에서 5F/RF 주행 준비 상태를 잃지 않고 이어가기 위한
현재 기준 문서다. Synthetic 테스트 통과를 실제 로봇 주행 승인으로 해석하지
않는다.

## 현재 확인된 사실

- 5F map: `src/multifloor_manager/config/maps/floor_5F.{yaml,pgm}`
- RF map: `src/multifloor_manager/config/maps/floor_RF.{yaml,pgm}`
- 5F/RF map resolution: `0.05 m/px`
- 5F/RF의 home, stair entry/landing, roof-loop 위치는
  `src/mission_manager/config/locations.yaml`에 기록되어 있다.
- AprilTag `501`은 5F DOWN landing, `600`은 RF UP landing으로 등록되어 있다.
  두 tag 모두 `tagStandard41h12`, `0.18 m`이다.
- 5F→RF 계단 관찰 기록은 `10 steps → landing → 9 steps → 2 exit steps`이다.
- 계단 형상 기록에는 폭 `1.23 m`, landing `2.70 m × 1.48 m`가 있다.
- `building_graph.yaml`에는 확인된 5F→RF 및 RF→5F stair edge를 dormant
  topology로 연결했다. `configured: false`이므로 실행에는 사용되지 않는다.
- `stair_captures/`에는 3F→4F 계단 BAG가 있고, `manual_captures/`에는
  5F rooftop route 기록이 있다. 최신 시도본은 `manual-5F-rooftop_route_1788920816079496577.bag.active.invalid`이다.
- synthetic 5F/RF `Mission.action` 테스트는 9/9 통과했다. 이는 mock child와
  no-motion recording fixture의 결과다.

## ROSBAG 직접 확인 결과

### 최신 시도본

`manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid`
는 실제 주행은 정상 완료했지만 마지막 bag finalize에서 실패해
`.active.invalid` 확장자가 붙은 8분 28초, 1.4 GB 원본 기록이다.
`rosbag info`로 읽히고 원본 데이터가 남아 있다.

- RGB 10,407장, `/scan` 4,912개, LiDAR 2,690개, IMU 95,998개,
  wheel odometry 47,755개
- 시간 순서 RGB 샘플에서 계단 flight, intermediate landing, RF rooftop가
  실제로 보인다.
- 일부 프레임에서 AprilTag 형태의 표식이 보인다. 후보 crop을 직접 확대해
  `600`과 `501` 숫자를 식별했다. 이는 이미지 기반 시각 확인이며 detector
  topic의 자동 검출 결과는 아니다.
- BAG topic에는 `/tag_detections`, `/mission/feedback`,
  `/multifloor/floor_state`가 없다. 이미지로 계단·RF 장면과 tag 501/600을
  확인할 수 있고, operator report는 5F→RF→5F 주행 정상 완료를 기록한다.
  다만 mission completion topic이 없고 bag finalize가 실패했으므로, 이를
  독립적인 finalized acceptance artifact로 취급하지 않는다.
- wheel odometry 기준 기록 시간은 약 497초, 누적 경로는 약 73.9 m이다.
- 승인 전 profile 산출 결과는
  `stair-profile-analysis-2026-09-11.md`에 기록했다. 상승 두 flight의 관찰
  odom path는 `4.771 m`, `3.619 m`, 중간 turn은 `+2.964 rad`다. 하강은
  combined `6.782 m`만 확인되어 두 flight로 임의 분할하지 않았다.

### 최신 정상 완료본

`manual_captures/manual-5F-rooftop_route_1788918551828137154.bag`는
2026-09-09 10:49:31부터 34.8초 동안 기록된 정상 완료본이다. 이 파일은
실내 평탄 구간 샘플 위주이며 계단이나 501/600을 확인할 수 없다.

따라서 이후 분석은 최신 `.active.invalid` 원본을 우선 사용한다. 이 파일은
주행 결과 분석 자료로 사용하되 finalized production acceptance BAG로 부르지
않는다. 다음 재기록에서는 finalize된
BAG에 `/tag_detections`, `/multifloor/floor_state`, `/mission/feedback`,
`/mission/result`를 반드시 포함한다.

## 현재 실행 차단 상태

다음 파일은 fail-closed 상태이며, 임의로 `configured: true`로 바꾸지 않는다.

```text
src/mission_manager/config/locations.yaml       configured: false
src/mission_manager/config/building_graph.yaml  configured: false
src/mission_manager/config/scan_profiles.yaml   configured: false
src/multifloor_manager/config/floors.yaml       configured: false
src/multifloor_manager/config/stairs.yaml       configured: false, stairs: []
src/multifloor_manager/config/apriltags.yaml    configured: false
src/multifloor_manager/config/transitions.yaml  configured: false, transitions: []
src/stair_supervisor/config/stair_profiles.yaml  configured: false, profiles: []
```

현재 production validator의 첫 차단은 다음과 같다.

```text
BUNDLE_ERROR: locations.yaml:configured: must be true before use
```

## 다음 작업 순서

1. 저장된 5F/RF map, 위치, tag 기록을 각 production YAML의 실제 schema와
   대조한다. 이미 있는 값은 재요청하지 않는다.
2. `stairs.yaml`에 5F↔RF 양방향 endpoint, landing quaternion, expected tag를
   연결한다. endpoint covariance는 실제 AMCL/commissioning 기록에서만 채운다.
3. `stair_profiles.yaml`에 UP/DOWN profile을 연결한다. 다음 값은 추정하지
   않는다: 두 flight signed distance, landing turn yaw, pitch enter/exit,
   tolerance, sensor freshness, sample gap, odom/yaw jump bound, timeout.
   BAG 분석 후보는 별도 승인 문서에만 있으며 production에 아직 복사하지 않았다.
4. `building_graph.yaml`에 5F↔RF의 `STAIR_UP`/`STAIR_DOWN` edge를 연결하고,
   기존 roof loop와 복귀 경로를 검증한다. 이 단계의 dormant edge는 이미
   추가되어 있다.
5. transition policy와 scan profile의 실제 topic을 검증한다. `/scan` publisher는
   하나여야 하고, D435F는 `camera1`이어야 한다.
6. 모든 파일의 값과 map fingerprint를 검토한 뒤에만 `configured: true`를
   일괄 활성화한다. 일부 파일만 활성화하지 않는다.
7. 다음 명령으로 정적 검증을 실행한다.

   ```bash
   python3 validate_bundle.py --validate-production-config
   ./run.sh --check
   ./run.sh --preflight
   ```

8. 실제 주행 전 readiness를 확인한다: action server, floor `READY`, stair
   supervisor `NAV`, `/scan`, odom, IMU, TF, tag, map, costmap, 단일 `/scan`
   publisher.
9. 첫 실제 주행은 짧은 `navigate` 구간으로 하고, 그 뒤 5F→RF stair,
   roof loop, RF→5F 복귀를 단계별로 승인한다.

## 다시 확인하지 말아야 할 것

- 5F/RF map이 존재하는지부터 다시 조사하지 않는다.
- 501/600 등록 사실을 3F→4F BAG의 tag 결과와 혼동하지 않는다.
- synthetic fixture를 production map/profile로 복사하지 않는다.
- geometry만으로 stair profile의 제어·허용오차 값을 만들어내지 않는다.
- validator를 통과시키기 위해 `configured` flag만 먼저 켜지 않는다.

## 재개 명령

```bash
cd /home/m3tron/Desktop/TRON1_Control/TRON1_RViz_Navigation
python3 validate_bundle.py --validate-production-config
```

실패 시 이 문서의 **현재 실행 차단 상태**와 **다음 작업 순서**를 갱신한 뒤
그 다음 한 단계만 진행한다.
