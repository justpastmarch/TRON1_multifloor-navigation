# TRON1 · 계단 측위와 중앙 유지

> 보관일: 2026-09-22. 원본 HTML을 당시 내용 그대로 옮긴 기록입니다. 현재 적용 상태는 [최신 적용 안내](../../APPLICATION_GUIDE.md)를 확인합니다.

원본: [stair-centering-analysis-20260921/method.html](/home/m3tron/.codex/.chatgpt-projects/g-p-6aa37f17097c81919ca6c59ce41e856c/stair-centering-analysis-20260921/method.html) · SHA-256: `e0f105cfccea4f645b7af4d75285587c59bbab681747c3648402192da61687ef`

---

<span id="계단-상대-측위중앙-유지-오프라인-실험"></span>

# 계단 상대 측위·중앙 유지 오프라인 실험

2026-09-21 · 기록 기반 실험 완료 · 실장비 성능은 미검증

개발 bag은 20260821\_155022 하나로 제한한다. 나머지는 고정된 알고리즘으로 평가하며 결과를 본 뒤 바꾸면 별도 실험 revision으로 표시한다. RGB로 계단 장면과 평지/엘리베이터를 구분하고, 기하 추정 결과를 보고 유리한 계단 구간을 고르지 않는다.

원시 Livox 3D 점군을 기록된 static transform으로 body frame으로 변환한다. 제한된 근거리에서 좌우 측면 평면 후보를 robust fit하고, 평행성·범위·관측 지지로 쌍을 평가한다. 계단 폭의 실제 지지 경계와 평면이 같은지는 영상과 측량 없이는 확정하지 않는다. odometry는 비교 baseline과 예측에만 쓰며 같은 odometry로 deskew한 scan을 독립 정답으로 쓰지 않는다.

정량값은 ①관측 가능 비율·공백 ②좌우 방향 일치·반복 정합 안정성 ③동일 직선 계단 구간에서 odometry 누적 예측과 기하 관측의 차이 ④가상 보정 명령과 센서 지연이다. 같은 센서의 split 검증은 내부 일관성이지 절대 정확도가 아니다. 명령을 바꾸면 미래 관측도 바뀌므로 open-loop bag 비교로 실제 중앙 오차 감소율·계단 성공률·무게중심 안정성을 증명하지 않는다. 필요하면 운동학 민감도는 별도 가정으로만 표시한다.

모든 원본은 read-only로 열며 운용 코드·ROS·기체를 변경하지 않는다. 결과 스크립트와 숫자·그림은 재현 산출물로 보존하고 임시 cache와 검토 사본은 제거한다.

<span id="원본-읽기와-표본화"></span>

## 원본 읽기와 표본화

21개 bag 중 raw LiDAR가 있는 18개에서 총 28,983 CustomMsg, 569,099,136포인트를 디코딩했다. 매 5번째 패킷의 점군 5,803개를 기하 평가에 사용했다. 가변 빈도와 기록 공백이 있어 균일한 2Hz는 아니다. 빠른 19바이트 CustomPoint 디코더를 각 bag 첫 패킷의 처음·중간·마지막 포인트에서 생성 ROS 디코더와 대조했다. summary.json에 검증 결과를 남겼다.

기록된 static 변환은 xyz=(0,0,0.18), quaternion=(0,0.043619387,0,0.999048222)이다. body frame으로 변환 후 유한 점·원점 거리 \>0.25m·x=(-1,4)·|y|\<3·z=(-1.2,2.5)m 조건과 2.5cm voxel 축소를 적용했다. IMU 자세 보정·odom deskew는 하지 않았다. RGB는 record time 기준 최소 5초 간격으로 추출하고 contact sheet로 구간을 표시했다. 추출한 모든 이미지의 정밀 판독을 의미하지 않는다.

READ 19개는 명시한 토픽/표본 범위다. 원본 모든 토픽을 새로 읽었다는 의미가 아니다. LiDAR·RGB가 없는 yaw bag 2개는 원본 index와 기존 숫자 자료를 확인한 METADATA\_ONLY다. 원본 상태와 범위는 analysis-state.json에 있다.

<span id="동결평면-모델"></span>

## 동결·평면 모델

개발 bag 7의 결과만 보고 기하 코드 해시·파라미터를 frozen-method.json에 고정했다. 평가 결과를 보기 전에 RGB로 개발 2구간·평가 19구간을 정했다. 라벨에는 접근·정지가 있고 약 5초 경계 불확실성이 있다. 발의 접지 phase 정답은 아니며 자동 단계 판별도 아니다. bag 13의 긴 마지막 구간과 RF 하강 후보를 평가 후 제거하지 않았다. 시각 이상 bag은 RGB와 LiDAR의 실제 관측 순간도 불일치할 수 있다.

평면은 n·p=d, 법선은 +y로 정의한다. 양면 y절편을 공통 법선에 투영해 폭·중심을 구하고 중심의 반대 부호를 몸체의 좌우 편차로 반환한다. 방향은 body XY 투영이다. 실제 지지 경계·무게중심·6DoF 자세는 측정하지 않는다. ROI·RANSAC 360가설·3.5cm inlier 잔차·최소 65점·5.5% 지지·0.65/0.30m x/z span·8° 평행성·0.75\~2.4m 폭 조건은 실험 파라미터이며 안전 임계값이 아니다.

V1은 각 구간의 첫 유효 양면 폭 3개의 중앙값으로 초기화한다. 양면 폭 차이 ≤8cm이면 채택한다. 유효 쌍이 없으면 좌우 법선 모순 8° 초과를 배제하고 오른쪽 면, 없으면 왼쪽 면에 고정 폭을 적용한다. 유효 쌍의 폭이 달라지면 한면 보완하지 않는다. V2는 평가 후 발견한 빈틈을 좁힌다. 두 면 후보가 모두 있는데 유효 쌍이 아니면 한면 보완하지 않는다. V2는 탐색 결과이며 독립 holdout이 없다. 같은 면의 시간 추적·지지 경계 식별은 아직 없다.

<span id="비교와-가상-명령"></span>

## 비교와 가상 명령

양면 기준은 V1과 동일한 초기화·폭 검사 뒤의 DUAL이다. raw pair 비율도 별도 보존한다. 출력 수는 정답 수가 아니다. 시간 환산은 유효 표본부터 다음 표본/구간 끝/0.6초 중 가장 이른 시점까지만 인정한다. 공백을 채우지 않는다.

odom은 증가하는 header 간격 ≤0.25초에서만 보간한다. 첫 유효한 기하/odom 표본으로 기준을 맞춘 뒤 고정 계단축에 odom 변위를 투영한다. 그래서 첫 오차가 상쇄되고 독립 절대 측위 시험도 아니다. |record-header| 0.25/0.5/1/2초별 민감도는 시간 동기화 검증을 대신하지 않는다. 해당 차이를 모두 네트워크 지연으로 해석하지 않으며 odom 기준점의 시각 계약도 미확인이다.

최초 1,088표본 산출률의 라벨→LiDAR 대응에는 record time을 사용했고 위 odom 시간 필터를 적용하지 않았다. 별도 check\_time\_alignment.py는 RGB 샘플의 record→header 시각을 보간해 같은 주석 경계를 옮긴 뒤 LiDAR header로 선택한다. 첫/마지막 RGB 바깥 경계는 가장 가까운 끝 시각으로 제한한다. RGB의 두 시각 순서가 모두 증가하는지 검사했고 제외 구간은 없었다. 기하·초기화 파라미터는 그대로다. 결과는 1,058표본, 양면 616, V1 772, V2 648이다. 평가 후의 시각 진단이며 sensor clock 동기화 증거가 아니다. header-aligned-traces.csv의 measurement\_relative\_s는 bag의 기록 시작 epoch에 대한 측정 header 시각이다. 다른 CSV의 record\_relative\_s와 혼동하지 않는다.

가상 전진식은 clip(-0.6*deadband(ey,0.05)-1.2*deadband(epsi,2deg), ±0.25)이다. 하강 후보에 같은 전진식을 계산한 출력은 방향 제어 검증에 쓰지 않는다. 실제 설계는 signed velocity를 반영한다. shadow-traces.csv와 exploratory-traces.csv에 출력·미산출 이유를 보존한다. ROS replay/publish는 없다.

<span id="알려진-강체-변형-시험"></span>

## 알려진 강체 변형 시험

평가 구간의 raw pair 유효 표본을 순서대로 매 12번째 선택해 63점군을 얻었다. 각 점군에 (좌우m,yaw°)=\[(-.10,0),(-.05,0),(.05,0),(.10,0),(0,-5),(0,5),(-.10,-5),(.10,5)\]를 적용했다. 원래 추정 평면에도 같은 변환을 적용한 기대값과 재추정 결과를 비교했다. 504시도 중 464성공, 40실패를 모두 보고한다. 잔차의 분모는 성공 464개다. 5cm 또는 3° 초과 8건은 실장비 허용 오차 판정이 아니라 불연속 진단이다.

이 시험은 좌표 변화·ROI·RANSAC의 일관성이다. 실제 이동에 따른 새 가림, 원래 면의 잘못된 선택, 절대 편향, 접지 동역학은 검증하지 않는다. 최대 잔차는 좌우 7.46cm, 방향 3.36°다.

<span id="재현"></span>

## 재현

환경: 기존 Python 3.8.10, numpy 1.24.4, scipy 1.10.1, OpenCV 4.2, matplotlib 3.1.2, 설치된 bag reader. 원본 경로는 analysis-state.json을 따른다. 분석 디렉터리에서 아래 순서로 실행한다.

<span id="cb1"></span>

``` sourceCode bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 extract_samples.py
python3 evaluate_geometry.py
python3 compare_methods.py
python3 robustness_checks.py
python3 check_time_alignment.py
python3 make_figures.py
python3 build_report.py
```

기존 표본·기하가 있으면 일부 단계는 재사용한다. 완전 재계산은 별도 결과 디렉터리에서 수행하고 frozen-method.json의 해시를 대조한다. 계산 시간은 분석 PC의 기하 함수 wall time이며 I/O·전처리·통신을 제외한다.

<span id="평가-독립성-점검"></span>

## 평가 독립성 점검

mandela의 Shared hallucination/Tautology 위험: 같은 센서·자기 추정 평면이 절대 정답이 될 수 없다. 그래서 불일치·일관성으로만 표시했다. Verifier=designer·Shared-pool bias 위험: 같은 작성자의 RGB 라벨, 같은 계단 반복 기록, 평가 후 V2 수정이다. 코드와 실패 공개만으로 독립 정답이 생기지는 않는다. 새 기록·별도 측량이 실제 정확도와 주행 개선률의 근거로 필요하다.
