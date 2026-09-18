STATUS: final research synthesis — implementation not authorized

# TRON1 계단 중심·방향 보정 방식 연구

## 결론 요약

현재 방식은 odometry로 거리와 회전 완료만 판정하는 open-loop phase sequencer다.
계단 기준 lateral offset과 heading error를 관측하지 않으므로 중심 이탈을 능동적으로
막을 수 없다.

가장 단순하고 방어 가능한 개선안은 **유효성이 검증된 2D wall boundary line observer +
제한된 IMU/odometry propagation + bounded yaw correction**이다. 3D LiDAR와 RGB는 기본
필수요소가 아니다. 검증된 failure mode를 보완할 때만 admission, cross-check, recovery에
추가한다.

그러나 이 구조는 아직 배포 가능하지 않다. `/scan`이 계단 주행의 pitch·roll 범위에서
동일한 wall boundary를 계속 관측하는지, line association이 고유한지, zero command가
중간 계단 자세에서 물리적으로 안전한지 확인되지 않았다. 다음 작업은 controller 구현이
아니라 독립 기준을 사용한 static pose-grid 실험이다.

## 현재 구현과 기록 데이터

### 구현

현재 `stair_supervisor`는 `/tron/wheel_odom_raw`의 `(x, y, yaw)`만 사용한다. 거리
진행률은 phase 시작 이후 변위를 진행 방향으로 투영한 값이며, yaw 진행률은 연속 sample
사이의 wrapped yaw 차이를 누적한 값이다. 이 evidence는 phase 완료와 fault 판정에만
사용된다.

두 계단 flight에서는 `(±linear_speed, 0)`을 고정 전송하고 landing turn에서는 고정
angular speed를 전송한다. `ALIGN` phase도 실제 회전이나 횡이동을 명령하지 않는다.
따라서 현재 구현에는 다음 항목이 없다.

- 계단 중심선이나 난간에 대한 횡방향 오차
- 계단 축에 대한 상대 heading 오차
- 이 오차에 비례한 steering correction
- IMU, LiDAR, camera, 접촉, pitch 또는 roll 기반 correction

더 근본적으로 physical command conversion은 WebSocket `y` 값을 항상 0으로 만든다.
즉 현 상태에서는 횡방향 오차를 관측하지 못할 뿐 아니라 lateral velocity도 명령하지
못한다. 다만 yaw를 조절하며 전진하는 differential-style correction은 transport shape를
바꾸지 않고도 가능하다.

현재 안전 동작은 보정 기능과 별개다. stale·future·비단조 timestamp, sensor gap,
position/yaw jump, reverse progress, timeout을 fault로 처리하고 command를 0으로 만든다.
취소는 이동 중 즉시 수행하지 않고 `VERIFY_ENTRY`, `LANDING`, `EXIT_CONFIRM`에서만
수행한다. fault 뒤 자동 reconnect나 traversal resume은 없다.

### 기록 데이터

저장소의 계단 bag 17개는 모두 읽을 수 있지만 topic 구성이 동일하지 않다. 비교적 완전한
bag은 IMU 약 200 Hz, wheel odometry 약 100 Hz, LiDAR와 `/scan` 약 10 Hz, camera 약
20~30 Hz를 포함한다. 주 5F/RF 기록은 508.439초 동안 IMU 약 188.8 Hz, wheel odometry
약 93.9 Hz, LiDAR 약 5.3 Hz, RGB 약 20.5 Hz, `/scan` 약 9.7 Hz를 포함하지만 기록
finalize에 실패한 incomplete artifact다.

기존 추출물에서는 0.5초를 넘는 gap이 232개였고 232.026~236.569초에는 여러 sensor가
동시에 끊겼다. 상대 LiDAR registration은 처리한 1,345 frame 중 839 update만
수락했다. 기존 stair-state replay 결과도 모두 `FAULTED` 또는 `INCOMPLETE`다.

이 데이터는 sensor 가용성, 상대 궤적, dropout, odometry discontinuity, 현재 FSM의
reject 동작을 분석하는 데는 충분하다. 그러나 surveyed stair frame, 독립 pose truth,
동기화된 command 기록이 없으므로 다음을 정량적으로 확정할 수 없다.

- 계단 중심선에 대한 실제 lateral drift
- 독립 기준에 대한 heading drift
- 동적 계단 주행 중 IMU bias
- wheel slip과 odometry dropout의 구분
- 새 controller의 기존 방식 대비 성능 향상

따라서 현재 기록은 architecture hypothesis와 다음 실험 설계의 근거로만 사용하고,
controller 검증 증거로 과장하지 않는다.

## 후보 방식 비교

1. **Validity-gated 2D wall line:** 조건부로 lateral error를 직접 관측한다. 복잡도는
   낮고 mobility는 높다. Static pose-grid를 통과할 때의 최소 권고안이다.
2. **계단 flight별 fiducial:** 직접 pose를 관측하지만 환경 개조가 필요하다. 2D natural
   boundary가 실패할 때의 강한 fallback이다.
3. **3D stair boundary 또는 edge:** 직접 관측 가능성이 높고 mobility를 유지하지만
   복잡하다. 2D 실패가 입증될 때 평가한다.
4. **Physical guide:** 설비 논리는 단순하고 기계적으로 lateral motion을 제한하지만
   일반 mobility가 낮고 contact와 jam dynamics가 검증되지 않았다.
5. **단순 단일 wall 거리 유지:** line identity, heading, dropout gate가 없으면 잘못된
   line을 따라갈 수 있어 불충분하다.
6. **사전 정렬과 heading hold:** 구조는 매우 단순하지만 slip이나 비대칭 접촉 뒤의
   lateral displacement를 관측하지 못한다.
7. **저속 open loop:** disturbance 결과를 줄일 수 있어도 uncertainty 누적을 관측하지
   못한다.
8. **Landing에서만 보정:** flight 중에는 lateral drift를 보지 못하므로 edge에 접근한
   뒤에야 수정할 수 있다.
9. **2D+3D+RGB full hybrid:** 관측 후보는 많지만 현재 증거로는 검증 부담만 키우는
   과설계다.

한쪽 wall만 쓸 때 얻는 값은 wall에 대한 perpendicular distance와 방향이다. 이것을
stair centerline error라고 부르려면 wall과 유효 stair center 사이의 거리, robot body
envelope, turning sweep, perception uncertainty가 측정되어 있어야 한다. rail은 연속선으로
관측된다는 실험이 없는 한 wall과 동등하게 취급하지 않는다.

## IMU·Odometry·LiDAR 오차 분석

IMU와 wheel odometry를 결합하면 짧은 시간의 body motion 추정은 개선된다. 그러나 두
sensor 모두 계단 경계나 중심선을 직접 관측하지 않으므로 stair-relative lateral
offset은 관측 불가능하다. 짧은 traversal은 drift 크기를 줄일 수 있을 뿐 이 관측성
문제를 없애지 않는다.

IMU의 적절한 역할은 roll/pitch와 angular-rate propagation, impact·vibration 감지,
sensor consistency 검사다. 계단 충격 중 acceleration을 중력으로 간주하면 attitude가
오염될 수 있으므로 해당 update를 downweight하거나 reject해야 한다. yaw와 수평 위치는
외부 constraint 없이는 절대 기준이 되지 않는다.

wheel odometry는 local progress와 실제 motion 반응을 추정하는 데 유용하지만 stair edge,
impact, load transfer, compliance, slip이 contact assumption을 깨뜨린다. 따라서 slip
residual에 따라 covariance를 키우거나 measurement를 거부해야 하며 ground truth로 쓰면
안 된다.

RGB는 반복되는 stair edge나 양쪽 boundary가 보일 때 상대 heading과 boundary ratio를
제공할 수 있다. 그러나 line/stair detection score는 metric centering 정확도나 traversal
safety가 아니다. 단안 RGB는 depth가 없고 blur, gait vibration, lighting, low texture,
occlusion에 취약하므로 primary geometry sensor보다 secondary semantic·consistency check로
두는 것이 현재 증거에 부합한다.

LiDAR 비교는 다음 연구 wave에서 확정한다.

LiDAR 문헌상 가장 단순한 직접 관측은 stair flight와 나란한 wall 또는 rail line이다.
robust line fit으로 signed distance와 line heading을 얻으면 controller가 필요한
`e_y`, `e_psi`가 바로 나온다. 두 wall이 보이면 폭 consistency와 midpoint를 사용할 수
있고, 한쪽만 보이면 side identity와 목표 wall distance를 고정해야 한다.

다만 이 결론은 조건부다. 로컬 `system.launch`는 `/scan`을 만들지 않고 mini PC의
`mid360s_laserscan`이 publish한다고 명시한다. 해당 projection의 height·angle filter가
저장소에 없으므로 현재 `/scan`이 계단 자세에서 wall이나 rail을 지속적으로 포함하는지는
코드만으로 확인할 수 없다. 실제 bag과 mini PC 설정 검증 전에는 2D observer를 확정된
primary sensor라고 부르지 않는다.

3D Livox에서는 여러 tread/riser edge의 공통 방향과 boundary를 fit할 수 있다. 이는
stair identity, 초기 정렬, 2D line 검증과 recovery에 적합하지만 sparse하고 비반복적인
scan, motion distortion, 반복 geometry 때문에 단일 primary loop로 쓰기에는 더 복잡하다.
LIO도 deskew와 짧은 propagation에는 유용하지만 명시적인 stair 또는 wall model 없이는
stair-relative error를 만들지 못한다.

## 권고 제어 구조

현재 최우선 후보는 다음 bounded proportional yaw law다.

```text
omega_cmd = sat(k_psi * e_psi + k_y * atan2(e_y, L))
```

`atan2(e_y, L)`는 lateral error를 bounded heading correction으로 바꾸며 저속에서
`e_y / v` 형태처럼 발산하지 않는다. yaw-rate saturation과 slew limit를 적용하고 초기
구현에는 integral term을 넣지 않는다. forward speed는 perception confidence와 physical
clearance에 따라 별도로 제한한다.

상태는 `TRACKING → PROPAGATING → DEGRADED → STOP/ABORT`로 명시한다. 신뢰할 수 있는
stair-relative geometry가 있을 때만 full correction을 사용한다. 짧은 dropout에서는
IMU·odometry로 제한 시간 동안만 propagate하고, uncertainty가 커지면 속도와 yaw authority를
줄인다. timeout, persistent saturation, command-motion disagreement 또는 edge margin 접근
시에는 stale correction을 계속 쓰지 않고 정지한다.

이 구조는 straight flight의 작은 error 영역에서는 양의 gain과 nonzero forward speed 아래
local convergence를 설명할 수 있다. 그러나 saturation, delay, false geometry, stair edge
constraint가 포함된 안전성은 이 이론만으로 보장되지 않으며 별도 실험이 필요하다.

### 최소 architecture

1. `StairBoundaryObservation`
   - `e_y`, `e_psi`, timestamp, covariance 또는 error bound
   - boundary side/identity, fit support, residual, association validity
2. `TRACKING`
   - fresh하고 identity-consistent한 2D line만 feedback에 사용
3. `COAST` 또는 `PROPAGATING`
   - IMU·odometry로 짧게만 propagation
   - timeout이 아니라 남은 clearance에서 도출한 시간·거리·uncertainty budget 사용
4. `LOST/CONTRADICTED`
   - wrong-side line, innovation jump, stale transform, budget 초과 시 zero와 latched fault
5. 단일 command owner
   - 기존 `StairSupervisor`가 linear speed, yaw rate, stop, watchdog을 계속 단독 소유

3D fit과 RGB는 동일 LiDAR projection을 검증하는 독립 truth가 아니다. 같은 sensor,
extrinsic, visibility 오류를 공유할 수 있다. 3D는 실제로 2D failure mode를 해결한다는
독립 실험이 있을 때만 추가한다.

## 최소 검증 실험

새 방식을 곧바로 계단에서 시험하지 않는다. 검증은 같은 estimator와 controller를
유지한 채 위험도를 단계적으로 높인다.

### 1. 정지 sensor 기준선

- 주행 전·후 각각 충분한 stationary 구간을 기록한다.
- raw IMU, wheel odometry, LiDAR, RGB, `/scan`, TF, 실제 command, supervisor phase를
  동일 bag에 저장한다.
- stationary 구간에서 gyro bias, noise, timestamp 역전, gap을 측정한다.
- 정지 상태인데 추정 heading이나 위치가 계속 변하면 다음 단계로 진행하지 않는다.

### 2. 평지 corridor replay와 저속 주행

- 폭과 중심선이 측정된 corridor에서 lateral error와 heading error를 독립적으로
  계산한다.
- 같은 입력 기록에 current open-loop와 candidate correction을 offline replay하여
  command continuity, saturation, dropout 동작을 비교한다.
- 이후 낮은 속도의 실제 주행에서 동일 지표를 측정한다. perception stale 또는
  confidence 저하 시 correction을 0으로 만들고 안전 정지해야 한다.

### 3. 모의 계단 또는 넓은 단차 구간

- 추락 위험이 없는 넓은 구조물에서 centerline, heading, edge clearance를 외부 기준으로
  측정한다.
- 시작 lateral offset과 heading offset을 의도적으로 달리해 capture한다.
- sensor dropout과 한쪽 wall/rail 소실을 재현해 controller가 잘못된 방향으로 누적
  보정하지 않는지 확인한다.

### 4. 실제 계단 gated trial

- 안전 tether와 즉시 정지 가능한 operator를 둔다.
- 최초 trial은 nominal center에서 저속으로 수행한다.
- 이후에만 작은 초기 offset을 한 축씩 적용한다. lateral offset과 heading offset을
  동시에 크게 주지 않는다.
- 각 flight와 landing을 별도 trial로 검증한 뒤 전체 traversal을 수행한다.

### 필수 지표

- stair centerline 기준 lateral error의 최대값과 RMS
- stair axis 기준 heading error의 최대값과 RMS
- 최소 edge clearance
- correction command의 최대값, rate, saturation 시간
- perception stale/dropout 비율과 longest gap
- wheel odometry, IMU propagation, exteroceptive observation 사이의 residual
- safety fault, operator intervention, traversal 성공 여부

정량 threshold는 로봇 폭, 실제 계단 유효 폭, perception 오차 분포를 측정하기 전에는
고정하지 않는다. 특히 현재 `0.50 m` 거리 tolerance와 `0.35 rad` yaw tolerance를
lateral safety margin으로 재사용하면 안 된다. 두 값은 phase 완료용 threshold이지
계단 edge clearance 보장이 아니다.

## 위험과 남은 불확실성

- mini PC의 `mid360s_laserscan` projection parameter가 저장소에 없으며 wall slice의
  높이·angle·pitch 민감도가 확인되지 않았다.
- 강한 RANSAC line도 doorway, riser, landing edge, 반대쪽 wall 또는 rail post일 수 있다.
- yaw steering만으로 lateral path를 바꾸므로 correction distance와 forward speed가
  필요하다. 직접 lateral actuation과 동일한 recovery를 주장할 수 없다.
- 현재 `0.40 m/s` nominal stair speed에서 sensor delay, yaw authority, body sweep,
  stopping distance가 측정되지 않았다.
- software가 zero command를 보내는 것은 확인됐지만, mid-step zero가 slip·tip 없이
  물리적으로 안정하다는 증거는 없다.
- 기존 bag과 내부 replay는 독립 stair-relative truth가 없어 controller 정확도 검증에
  사용할 수 없다.
- static pose-grid 성공은 vibration, impact, dynamic occlusion, slip을 검증하지 않는다.
  이후 speed-limited dynamic test가 별도로 필요하다.

## 출처

1. Forster et al., “On-Manifold Preintegration for Real-Time Visual-Inertial Odometry,”
   IEEE TRO, 2017. https://doi.org/10.1109/TRO.2016.2597321
2. Huang et al., “Observability-based Rules for Designing Consistent EKF SLAM Estimators.”
   https://doi.org/10.1109/IJRR.2010.2050890
3. Woodman, “An Introduction to Inertial Navigation,” UCAM-CL-TR-696.
   https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf
4. Helmick et al., “Autonomous Stair Climbing for Tracked Vehicles.”
   [JPL 원문 PDF](https://robotics.jpl.nasa.gov/media/documents/IJCV-IJRR-STAIRCLIMBING.pdf)
5. Gutmann et al., “Stair climbing for humanoid robots using stereo vision.”
   https://doi.org/10.1109/IROS.2004.1389593
6. Wang et al., “Deep learning-based ultra-fast stair detection.”
   https://doi.org/10.1038/s41598-022-20667-w
7. Westfechtel et al., “Robust stairway-detection and localization method for mobile robots.”
   https://doi.org/10.1177/0278364918798039
8. Sriganesh et al., “Fast Staircase Detection and Estimation.”
   https://arxiv.org/abs/2211.00610
9. Lin and Zhang, “LOAM-Livox.” https://arxiv.org/abs/1909.06700
10. Xu et al., “FAST-LIO2.” https://doi.org/10.1109/TRO.2022.3141876
11. Hoffmann et al., “Autonomous Automobile Trajectory Tracking for Off Road Driving.”
    https://doi.org/10.1109/ACC.2007.4282788
12. Thrun et al., “Stanley: The Robot that Won the DARPA Grand Challenge.”
    https://robots.stanford.edu/papers/thrun.stanley05.pdf
13. ROS `sensor_msgs/LaserScan` message definition.
    https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/LaserScan.msg
14. Open Robotics REP-105, “Coordinate Frames for Mobile Platforms.”
    https://reps.openrobotics.org/rep-0105/
15. Olson, “AprilTag: A Robust and Flexible Visual Fiducial System.”
    https://doi.org/10.1109/ICRA.2011.5979561

## 방법론 부록

본 연구는 네 wave, 최대 두 개의 동시 독립 작업으로 수행했다.

1. repository controller call path와 17개 bag/evidence inventory
2. IMU·odometry observability와 vision stair perception 문헌
3. 2D/3D LiDAR geometry와 controller family 비교
4. 단순 대안 adversarial review와 독립 architecture review

코드, 기록 데이터, 외부 문헌, 합성을 서로 다른 observation group으로 기록했다. 같은
LiDAR에서 파생한 2D와 3D 결과는 독립 validation으로 계산하지 않았다. 현재 결론의 가장
큰 불확실성은 paper 부족이 아니라 TRON1 실제 scan geometry와 독립 stair-relative truth
부재다.

### 단일 다음 실험

대표 계단에서 robot을 측정된 lateral offset, yaw error, pitch·roll 조합에 정적으로
배치한다. 각 pose에서 raw 3D LiDAR, mini PC가 publish한 정확한 `/scan`, TF, RGB, IMU,
odometry와 timestamp를 기록한다. onboard estimator와 독립적인 surveyed grid, external
camera 또는 motion tracking으로 `e_y`, `e_psi` label을 만든다.

데이터를 보기 전에 physical clearance와 stopping requirement에서 다음 합격 기준을
정한다.

- 허용 `e_y`, `e_psi` 오차
- flight pose envelope 전체의 coverage
- false association 비율
- longest dropout과 stale transform 한계
- required wall support length와 fit residual

2D가 전 영역을 통과하면 primary observer로 선택한다. geometry가 없거나 ambiguous하면
2D를 기각하고 3D direct boundary observer 또는 fiducial을 평가한다. 둘 다 독립 기준을
통과하지 못하면 controller 구현을 중단하고 sensing 또는 환경 reference를 변경한다.
