# 분석 재현과 자료 해석

ROS를 초기화하거나 replay·publish하지 않는 로컬 파일 분석입니다. 원본은 read-only open으로만 읽습니다. 보고서의 재현 스크립트는 원본 ROS bag을 읽기 위한 ROS Noetic Python 환경과 numpy/scipy/OpenCV/matplotlib를 사용합니다. 실행 당시 Python은 3.8.10입니다.

## 실행 순서

1. `analyze_recordings.py`: `bag-metadata.json`의 원본 21개를 순차로 읽어 `bags/<원본명>/summary.json`, `numeric.npz`, `events.json`, 선택한 `frames/`를 만듭니다. 이미 summary가 있으면 건너뜁니다. 완전한 재실행은 산출물의 별도 사본 디렉터리에서 하며 원본 bag을 지우지 않습니다.
2. `derive_findings.py`: 시각 간격, AMCL–odom 상대 불일치, scan 정합, 영상 특징점, mission/action/state와 tag를 계산합니다.
3. `validate_and_profile.py`: packed LiDAR 판독을 ROS 생성 역직렬화 클래스와 대조하고, 옥상 문제 구간과 원본 SHA-256을 저장합니다.
4. `make_figures.py`: 위 숫자에서 PNG를 만듭니다. matplotlib 캐시는 `_scratch/matplotlib`에만 만듭니다. 최종 전달 전 해당 캐시를 제거했습니다.

실행은 `PYTHONDONTWRITEBYTECODE=1 python3 <script>` 형태이며 ROS master에 연결하지 않았습니다. 설치 소스는 읽기만 했습니다. `all-bag-statistics.json`은 집계, bag별 `summary.json`은 토픽 수·시각·frames·callerid, `events.json`은 변경된 상태와 로그·검출을 보존합니다. rosout과 rosout_agg는 같은 로그가 중복될 수 있어 횟수 주장은 rosout만 사용했습니다.

## numeric.npz 열

모든 일반 배열의 첫 두 열은 bag record 초, header 초입니다. stamp가 없는 Twist 등은 두 번째 열이 NaN입니다. NaN stamp는 센서 수치 손상을 뜻하지 않습니다. `clock:<topic>`은 record, header, sequence이며 header가 없는 메시지는 NaN입니다.

| 배열 | 첫 두 열 뒤의 값 |
|---|---|
| Odometry | px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz, cov_xx, cov_yy, cov_yaw |
| AMCL / initialpose | px, py, pz, qx, qy, qz, qw, cov_xx, cov_yy, cov_yaw |
| IMU | wx, wy, wz, ax, ay, az, qx, qy, qz, qw, orientation_cov[0], angular_cov[0], acceleration_cov[0] |
| Joy | 원본 axes, 원본 buttons. 개수는 summary.topic_info에 있음 |
| Twist | linear x/y/z, angular x/y/z |
| websocket_tx | 두 번째 열은 JSON timestamp/1000, 이후 정규화 x/y/z. m/s로 변환하지 않음 |
| TF edge | tx, ty, tz, qx, qy, qz, qw. key는 `/tf:parent>child` |
| LaserScan | ray 수, 범위 내 finite 수, NaN 수, Inf 수, valid 최솟값/중앙값, angle_min, angle_increment, range_min, range_max |
| scan_ranges | scan 원본 range 배열. 첫 열이 시각인 배열이 아니며 `/scan`과 행 순서 동일 |
| Livox CustomMsg | point 수, XYZ finite 수, 영점 수, finite 거리 최소/최대, timebase 초, 최대 point offset ns |
| RGB | payload byte 수. 실제 선택 프레임의 시각·크기는 summary.selected_images |
| particlecloud | pose 수, 가중치 없는 x/y 표준편차. AMCL covariance와 동일한 지표로 취급하지 않음 |
| tag_detections | 해당 메시지 검출 수. ID/pose는 events.json |

## 수치 비교의 한계

- 모든 bag 메시지를 끝까지 읽었으며 serialized 메시지를 역직렬화했습니다. CustomMsg는 bag에 포함된 정의로 확인한 19-byte point 구조를 numpy로 읽었습니다. 18개 LiDAR bag마다 첫 메시지의 처음·중간·마지막 포인트를 ROS 생성 decoder와 대조했습니다. 모든 비교가 일치합니다. 전체 XYZ NaN/Inf 검사는 수억 포인트 전부에 적용했습니다. 이는 거리 정확도나 모든 reflectivity/tag 값의 의미가 검증됐다는 뜻은 아닙니다.
- raw RGB는 frame 크기와 payload 길이, compressed RGB는 메시지 구조·시각을 전수 검사했습니다. 이미지 디코딩·시각 검토는 208개의 선택 bag 프레임입니다. MP4 280프레임을 모두 디코딩했지만 영상 시각 검토는 12개 표본입니다. JPEG 36개는 전부 열어 contact sheet로 확인했습니다.
- scan 정합은 처음/끝 및 중간 15시점 주변 ±0.35초의 ray별 중앙값을 사용합니다. 0.25~15m의 finite 점만 남기고 trimmed point-to-point ICP를 수행합니다. 처음/끝 비교는 초기 이동 ±0.3m 및 yaw ±0.05rad 등 7개 초기값으로 확인했습니다. 이것은 map을 이용한 전역 위치 검증이나 추적 정확도 보증이 아닙니다.
- 정적 두 manual bag에서는 낮은 잔차·높은 점 대응률·다른 초기값의 일치와 별도 카메라 특징점 안정성이 함께 나왔습니다. FULL210849는 scan 정합 잔차 0.53m, 10cm 내 대응률 12.5%여서 최적합 0.38m를 실제 이동량으로 채택하지 않았습니다.
- 영상 특징점은 첫 영상의 corners를 각 선택 영상으로 추적하고 역방향 검사 및 RANSAC inlier를 사용했습니다. 서브픽셀 정합값은 물리 거리 정확도가 아닙니다. 초기 영상 대비 큰 이동·가림에서는 실패할 수 있어 full bag 연속 영상 odometry로 사용하지 않았습니다.
- AMCL 비교는 각 AMCL header 시점의 odom x/y/yaw를 보간하고, 이전 AMCL pose에 odom 상대 움직임을 적용한 뒤 새 AMCL과의 차이를 계산합니다. 보간을 감싸는 odom gap이 0.25초 이상이면 제외합니다. AMCL 두 시점 자체의 간격은 결과 JSON에 별도 표시합니다. 긴 간격에서의 잔차는 순간 점프로 표현하지 않습니다. header 자체가 수신 시각으로 대체된 사례가 있어 실제 측정 동시성을 보장하지 않습니다.
- rosbag record 순서의 미세한 간격으로 속도를 계산하지 않았습니다. 시계 동기화·재타임스탬프·latched 시작 메시지를 구분하지 못하면 record-header 차이를 순수 통신 지연으로 확정할 수 없습니다.
- odometry와 같은 발행자가 생성한 TF는 독립 검증이 아닙니다. bag에서 만든 MP4/JPEG도 센서와 독립된 외부 기준이 아닙니다. LiDAR와 RGB는 odom 이외의 관측이지만, 현재 `/scan` 생성은 raw odometry를 deskew에 사용합니다(2026-09-20 연결 코드 추가 확인). 따라서 scan 정합을 raw odom과 완전 독립으로 세지 않습니다. 독립 실측 좌표·모션캡처는 없습니다. 물리 정지·거리·계단 성공은 이 한계를 포함해 해석합니다.

## 실제 실장비 확인에 필요한 다음 기록

기존 recording 경로를 확장할 때 원시 시각·수신 시각·시각 대체 여부, NAV 요청·최종 WebSocket 송신·가능한 로봇 피드백, ownership/mode/mission/floor/map 세대, initialpose·map 전이 사건을 같은 기록에 남기는 계획이 필요합니다. 실제 밀림은 외부 고정 카메라 또는 측량 기준으로 별도 측정해야 합니다. 최종 송신 로그는 물리 실행 ACK가 아닙니다. 문 열림 확인, 사진 산출물, 이메일 결과, 복귀 승인도 각각의 결과 사건이 있어야 해당 E2E를 판정할 수 있습니다. 이것은 변경 제안이며 이번 분석에서 구현하지 않았습니다.
