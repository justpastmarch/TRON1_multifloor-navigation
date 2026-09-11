# 5F/RF stair profile analysis

분석 대상:

```text
manual_captures/manual-5F-rooftop_route_1788920816079496577.bag.active.invalid
```

이 문서는 승인 전 분석 결과다. `stair_profiles.yaml`과 production
configuration에는 아직 값을 쓰지 않았다.

여기서 `5F`는 5층 landing, `RF`는 rooftop landing을 뜻한다. 이 문서의
승인 대상은 **5F→RF UP 후보만**이다. RF→5F DOWN profile은 flight 분리가
안 되어 있으므로 이 승인 대상에 포함하지 않는다.

## 데이터 범위

| 항목 | 결과 |
|---|---:|
| recording duration | 508.44 s |
| RGB | 10,407 frames |
| wheel odometry | 47,755 samples |
| IMU | 95,998 samples |
| image tag 600 | 약 195.10 s |
| image tag 501 | 약 239.42 s |

이미지 시간축 샘플을 직접 확인한 결과 상승 flight는 다음과 같다.

| 구간 | 영상 근거 | wheel odom path |
|---|---|---:|
| UP flight 1 | stair treads 73.7–126.0 s | 4.771 m |
| landing/turn | landing·door 130.0–161.8 s | 5.303 m |
| UP flight 2 | stair treads 166.3–178.0 s, exit 182.0 s | 3.619 m |

상승 전체 관찰 구간의 odom path는 15.222 m이고, 두 flight 사이의 관찰된
turn은 약 `+2.964 rad`다. 이 path에는 정지·재정렬·wheel slip이 포함될 수
있으므로 건축 도면상의 signed stair distance로 간주하지 않는다.

하강은 `322–382 s`가 6F→5F 복귀 흐름과 일치한다. `322–354 s`에는
`6 → 5` 표식이 보이고, `386–450 s`에는 5F 표식이 보인다. 그러나 sampled
image에 stair tread가 없어 upper/lower flight 및 intermediate landing을
분리할 수 없다. 해당 구간의 combined odom path는 6.782 m, yaw 변화는
`-1.963 rad`다.

## 승인용 후보값

아래는 schema 형식에 맞춘 **UP 후보**다. 실행 가능한 production 값이 아니라,
다음 현장 확인에서 비교할 초기 후보다.

```yaml
# DO NOT COPY TO PRODUCTION WITHOUT OPERATOR APPROVAL
- id: stair_5f_rf_up_candidate
  direction: UP
  enabled: false
  linear_speed: 0.40
  angular_speed: 1.57
  alignment_yaw_rad: 0.715
  flight_1_distance_m: 4.771
  landing_dwell_sec: 5.0
  landing_turn_yaw_rad: 2.964
  flight_2_distance_m: 3.619
  exit_dwell_sec: 5.0
  distance_tolerance_m: 0.50
  yaw_tolerance_rad: 0.35
  pitch_enter_rad: UNRESOLVED
  pitch_exit_rad: UNRESOLVED
  sensor_freshness_sec: 0.20
  max_sample_gap_sec: 0.10
  max_odom_step_m: 0.10
  max_yaw_step_rad: 0.20
  timeout_sec: 300.0
```

수치 선택 근거:

- `flight_*_distance_m`: 영상으로 분리한 flight 구간의 wheel-odom path다.
- `landing_turn_yaw_rad`: 두 flight 사이 odom yaw 변화다.
- `linear_speed`, `angular_speed`: operator-provided candidate values
  (`0.40 m/s`, `1.57 rad/s`). BAG는 commanded stair speed topic을 기록하지
  않아 실제 계단 제어 입력값을 증명하지 않는다.
- `distance_tolerance_m`, `yaw_tolerance_rad`, dwell, timeout은 안전 승인값이
  아니다. profile schema를 채우기 위한 검토 대상이며 현장 재측정이 필요하다.
- `pitch_enter_rad`, `pitch_exit_rad`는 산출하지 않았다. 기록된 IMU에는 유효한
  orientation/pitch ground truth가 없고, linear acceleration만으로 pitch를
  안전 threshold로 변환할 수 없다.
- `timeout_sec: 300.0`은 현재 schema에서 profile 전체에 적용되는 단일 timeout이다.
  사용자가 요청한 flight별 2분 timeout을 정확히 구현하려면 schema와 supervisor의
  phase별 timeout 지원을 별도로 바꿔야 한다. 현재 값은 flight별 timeout으로
  해석하지 않는다.

## 하강 후보 및 차단 항목

하강은 현재 다음 수준까지만 산출 가능하다.

```text
combined observed interval: 322–382 s
combined wheel-odom path:   6.782 m
combined yaw change:       -1.963 rad
flight split:              unresolved
pitch thresholds:          unresolved
```

따라서 `flight_1_distance_m`, `flight_2_distance_m`, `landing_turn_yaw_rad`,
`pitch_enter_rad`, `pitch_exit_rad`를 하강 profile에 임의로 배분하지 않는다.

## 승인 전 확인 필요

| 확인 항목 | 상태 | 승인 조건 |
|---|---|---|
| UP flight 거리 | **UNRESOLVED** | wheel-odom 후보를 사용할지 endpoint 실측으로 교체할지 결정 |
| DOWN 두 flight 분리 | **UNRESOLVED** | tread가 보이는 재기록 또는 endpoint 실측으로 분리 |
| pitch threshold | **UNRESOLVED** | orientation 또는 독립 pitch ground truth 기록 |
| 속도·tolerance·freshness·jump·timeout | **UNRESOLVED** | 현장 안전 담당자가 각 값을 명시적으로 승인 |
| production 기록/검증 | **NOT STARTED** | 위 항목 승인 후 YAML 기록, validator, preflight 실행 |

## 승인 게이트

현재 상태에서 승인할 수 있는 것은 `stair_5f_rf_up_candidate`의 관찰값을
**현장 검증 입력으로 사용하는 것**뿐이다. 자율주행 enable 또는 production
YAML 반영 승인은 아직 금지한다. 다음 조건을 모두 만족하기 전에는 승인 완료로
간주하지 않는다.

1. 거리와 turn 값의 측정 방식 및 허용 오차를 승인한다.
2. pitch 값과 모든 safety bound를 빈칸 없이 확정한다.
3. 후보의 `enabled`를 false에서 true로 바꾸는 별도 승인과 production YAML
   전체의 configured gate를 수행한다.
4. `validate-production-config`, `./run.sh --check`, `./run.sh --preflight`가
   모두 통과한다.
5. 첫 실행은 정지·수동 개입 가능한 현장 gate 아래에서 UP 단독으로 수행한다.

위 조건 중 하나라도 미충족이면 이 문서는 승인 요청서가 아니라 분석 결과로만
유효하다.

## 분석 제한

- BAG에는 `/tag_detections`, `/mission/feedback`, `/multifloor/floor_state`가
  없다.
- tag 501/600은 이미지 crop의 육안 식별이며 detector message의 자동 검출이
  아니다.
- `.active.invalid`는 finalize된 acceptance BAG가 아니다.
- wheel odom은 stair geometry의 독립 측량값이 아니다.
