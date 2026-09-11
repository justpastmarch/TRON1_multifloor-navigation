# 층 전이 정책과 상태머신 개발·운영 가이드

이 문서는 `multifloor_manager`의 층 전이조건, 전이 정책 evaluator, ROS action 상태머신을 수정·검증·운영하는 개발자용 기준 문서다.

이 문서만으로 다음 작업을 수행할 수 있어야 한다.

- 현재 전이 정책의 의미를 읽고 값을 변경한다.
- 새 safety 전이조건을 설계하고 구현한다.
- 새 non-safety 운영조건을 설계하고 구현한다.
- 조건을 어느 상태와 어느 시점에 연결할지 결정한다.
- ROS action과 `FloorState` 상태 전이를 테스트한다.
- stale, future, wrong-epoch, preemption, timeout 문제를 재현하고 수정한다.
- source, fixture, test, generated 산출물의 ownership을 구분한다.

## 1. 먼저 알아야 할 현재 범위

현재 구현은 **하나의 enabled 정책**만 지원한다.

```text
policy id: floor_transition_ready
```

현재 production YAML은 의도적으로 비활성화되어 있다.

```yaml
schema_version: 1
configured: false
transitions: []
```

실제 현장 YAML 값을 넣거나 `configured: true`로 바꾸는 것은 현장 commissioning과 하드웨어 검증이 끝난 뒤에 수행한다. 로컬 개발·테스트에서는 `test/fixtures/building_valid`를 사용한다.

현재 지원되는 policy condition은 세 개뿐이다.

| 조건 | 분류 | 의미 | 평가 형태 | 현재 YAML 요구사항 |
|---|---|---|---|---|
| `T_FLOOR_CONFIRMED` | safety | 현재 transition의 목표 층 tag vote가 승인됨 | epoch event | `enabled: 1`, `required: 1` |
| `T_LOCALIZED` | safety | AMCL pose, scan/TF, odometry가 현재 기준으로 유효함 | live level | `enabled: 1`, `required: 1` |
| `T_COSTMAP_READY` | safety | 새 map에 맞는 global costmap이 barrier 이후 관찰됨 | live level | `enabled: 1`, `required: 1` |

현재 non-safety condition은 지원되지 않는다. README와 production YAML의 `T_ARRIVAL_ANNOUNCEMENT_DONE`은 구현 예시일 뿐이며, registry와 producer를 추가하기 전에는 validator가 거부한다.

## 2. 파일 구조와 ownership

```text
src/multifloor_manager/
├── action/
│   └── FloorTransition.action                 # goal/result/feedback ROS 계약
├── msg/
│   └── FloorState.msg                         # UNKNOWN/TRANSITIONING/READY/FAULT
├── config/
│   └── transitions.yaml                       # production policy; 현재 configured: false
├── src/multifloor_manager/
│   ├── transitions.py                         # immutable policy model + pure evaluator
│   ├── configuration.py                       # strict YAML parser/allowlist
│   ├── ros_node.py                            # action server + orchestration state machine
│   ├── ros_runtime.py                         # epoch, observation store, policy gate
│   ├── ros_callbacks.py                       # map/tag/pose/scan/odom/costmap callbacks
│   ├── map_evidence.py                        # post-arm map identity/generation evidence
│   ├── tag_evidence.py                        # AprilTag parsing and temporal vote
│   └── readiness.py                            # localization readiness policy
└── test/
    ├── test_transitions.py                    # pure evaluator tests
    ├── test_policy_final_gate.py              # final-gate/preemption/dwell tests
    ├── test_floor_transition_ros.py           # normal and blocked ROS action tests
    ├── floor_transition_ros.test              # valid fixture rostest launch
    └── floor_transition_policy_blocked_ros.test # stale-policy rostest launch

test/fixtures/
├── building_valid/
│   └── transitions.yaml                       # configured known-good policy
└── building_policy_blocked/
    └── transitions.yaml                       # same policy, freshness_sec: 0.01

test/
├── test_site_configuration.py                 # strict YAML/site boundary tests
└── test_policy_fixture_parity.py              # valid/blocked fixture difference contract
```

### Source와 generated 파일

수정할 파일은 `src/`, `test/`, `docs/`, `README.md` 아래의 source다.

다음 경로는 직접 수정하지 않는다.

```text
build/
devel/
install/
```

이 경로는 `catkin_make` 또는 `catkin_make install`로 재생성한다. generated copy가 source와 다르면 먼저 source import와 build 결과를 구분해 확인한다.

## 3. 데이터 모델

### 3.1 `TransitionCondition`

파일:

```text
src/multifloor_manager/src/multifloor_manager/transitions.py
```

```python
TransitionCondition(
    name: str,
    enabled: bool,
    required: bool,
)
```

- `name`: registry에 등록된 조건 이름
- `enabled`: evaluator에서 조건을 사용할지 여부
- `required`: 필수 safety 조건인지, optional quorum 대상인지 여부

현재 parser는 모든 허용 조건을 mandatory로 취급한다. 따라서 현재 production fixture에서 `enabled: 0`, `required: 0`은 모두 거부된다.

### 3.2 `PredicateObservation`

```python
PredicateObservation(
    value: bool,
    observed_at_sec: float,
    epoch: int,
)
```

조건 producer는 Boolean만 넘기지 말고 반드시 다음을 함께 기록해야 한다.

1. 결과값 `value`
2. 관찰 시점 `observed_at_sec`
3. 어느 transition에 속하는지 나타내는 `epoch`

관찰 시점이 없거나, 현재 epoch가 아니거나, 미래 시점이면 policy evaluator가 성공으로 처리하지 않는다.

### 3.3 `TransitionPolicy`

YAML 항목은 다음 typed object로 변환된다.

```python
TransitionPolicy(
    id: str,
    enabled: bool,
    conditions: tuple[TransitionCondition, ...],
    optional_count: int,
    freshness_sec: float,
    dwell_sec: float,
    timeout_sec: float,
)
```

불변성 및 validation 규칙:

- policy id는 비어 있지 않은 identifier여야 한다.
- `conditions`는 tuple이어야 한다.
- condition 이름은 unique해야 한다.
- `optional_count`는 음수가 아니어야 한다.
- `freshness_sec`와 `timeout_sec`는 양의 finite number여야 한다.
- `dwell_sec`는 0 이상이어야 한다.
- `dwell_sec < timeout_sec`이어야 한다.

### 3.4 `TransitionEvidence`

`TransitionManager.evaluate()`는 Boolean만 반환하지 않고 조건별 결과와 이유를 반환한다.

조건 결과 reason:

```text
true
false
missing
stale
future
wrong_epoch
```

운영 중 `READY`가 되지 않으면 이 evidence와 action feedback의 마지막 phase를 함께 확인한다.

## 4. YAML policy 사용법

### 4.1 현재 known-good fixture

파일:

```text
test/fixtures/building_valid/transitions.yaml
```

```yaml
schema_version: 1
configured: true
transitions:
  - id: floor_transition_ready
    enabled: 1
    conditions:
      - name: T_FLOOR_CONFIRMED
        enabled: 1
        required: 1
      - name: T_LOCALIZED
        enabled: 1
        required: 1
      - name: T_COSTMAP_READY
        enabled: 1
        required: 1
    optional_count: 0
    freshness_sec: 10.0
    dwell_sec: 0.2
    timeout_sec: 8.0
```

### 4.2 YAML 필드 설명

| 필드 | 자료형 | 의미 | 현재 제약 |
|---|---|---|---|
| `schema_version` | integer | 문서 스키마 버전 | `1`만 허용 |
| `configured` | boolean | 현장 운용 가능 여부 | production은 현재 `false` |
| `transitions` | list | policy 목록 | 정확히 하나만 허용 |
| `id` | string | policy 이름 | `floor_transition_ready`만 허용 |
| `enabled` | `0` 또는 `1` | policy 활성화 | 반드시 `1` |
| `conditions` | list | 조건 선언 | 허용된 이름, 중복 금지 |
| condition `name` | string | 조건 registry key | 현재 세 이름만 허용 |
| condition `enabled` | `0` 또는 `1` | 조건 사용 여부 | 현재 mandatory 조건은 `1` |
| condition `required` | `0` 또는 `1` | 필수 여부 | 현재 mandatory 조건은 `1` |
| `optional_count` | integer | optional true 개수 quorum | 현재 `0`만 허용 |
| `freshness_sec` | number | observation 최대 허용 age | 양수 finite |
| `dwell_sec` | number | 모든 조건이 연속 true여야 하는 시간 | 0 이상, timeout보다 작음 |
| `timeout_sec` | number | final policy gate timeout | 양수 finite |

YAML의 bit 값은 Python boolean이 아니라 정확한 integer `0` 또는 `1`이어야 한다.

```yaml
# 올바름
enabled: 1

# 잘못됨
enabled: true
enabled: "1"
enabled: 2
```

### 4.3 production과 fixture 수정 원칙

#### 기존 시간값만 튜닝하는 경우

다음 파일을 함께 검토한다.

```text
src/multifloor_manager/config/transitions.yaml
test/fixtures/building_valid/transitions.yaml
test/fixtures/building_policy_blocked/transitions.yaml
```

production은 현장 값을 넣을 때만 변경하고, 테스트 fixture는 deterministic test를 보존해야 한다.

blocked fixture는 정상 fixture와 구조가 같고 `freshness_sec`만 짧다.

```text
building_valid:          freshness_sec: 10.0
building_policy_blocked: freshness_sec: 0.01
```

이 차이를 임의로 늘리거나 조건 목록을 다르게 만들면 fixture parity 테스트의 의미가 사라진다.

#### 기존 safety 조건을 삭제하거나 optional로 바꾸는 경우

직접 수정하지 않는다. 이 변경은 safety contract 변경이므로 먼저 다음을 작성·검토해야 한다.

- 삭제/optional 전환의 위험 분석
- 대체 safety condition
- false/stale/future/wrong-epoch 테스트
- ROS action의 failure/READY 경계 테스트
- README와 이 문서의 contract 수정

## 5. Safety와 non-safety 조건 분류

### 5.1 Safety 조건

다음 질문에 “예”이면 safety 조건이다.

> 이 조건이 false인데도 `READY`를 허용하면 로봇 또는 주변 시스템이 위험해지는가?

Safety 조건은 다음 규칙을 모두 지켜야 한다.

- `enabled: 1`
- `required: 1`
- optional quorum에 포함하지 않음
- false, missing, stale, future, wrong epoch이면 fail-closed
- 현재 transition epoch에 귀속됨
- 재평가 시점에도 유효한지 확인함

현재 safety 조건:

```text
T_FLOOR_CONFIRMED
T_LOCALIZED
T_COSTMAP_READY
```

### 5.2 Non-safety 조건

다음 질문에 “예”이면 non-safety 후보이다.

> false여도 즉시 위험하지는 않지만 운영 품질 때문에 READY를 늦출 가치가 있는가?

예상 예시:

```text
T_ARRIVAL_ANNOUNCEMENT_DONE
T_OPERATOR_ACKNOWLEDGED
T_LOG_FLUSHED
```

현재는 이 분류를 YAML에서 사용할 수 없다. 새 non-safety를 지원하려면 parser의 mandatory-only 규칙을 optional-aware 규칙으로 바꾸고, producer와 테스트를 추가해야 한다.

Non-safety를 추가할 때도 다음은 safety로 유지한다.

```yaml
- name: T_LOCALIZED
  enabled: 1
  required: 1
```

예상 optional 형태:

```yaml
- name: T_ARRIVAL_ANNOUNCEMENT_DONE
  enabled: 1
  required: 0

optional_count: 1
```

`optional_count`를 1로 올리면 해당 운영조건이 실제로 READY를 지연시킨다는 뜻이다. 단순 telemetry라면 policy condition으로 넣지 않는다.

## 6. 새 조건 추가 개발 절차

아래 절차는 새 safety 조건과 새 non-safety 조건 모두에 적용한다.

### Step 1. 조건을 한 문장으로 정의

다음 형식으로 작성한다.

```text
T_<NAME> is true exactly when <observable fact> is true for epoch <N> at time <T>.
```

나쁜 정의:

```text
system is healthy
```

좋은 정의:

```text
T_WHEEL_ODOM_READY is true when a post-arm, fresh odometry sample for the active epoch is stationary within the configured velocity bounds.
```

### Step 2. safety/non-safety 분류

- false이면 위험 → safety
- false여도 안전하지만 운영상 지연 필요 → non-safety
- 단순 관찰·로그 → condition으로 만들지 않음

### Step 3. event인지 live level인지 결정

#### Event condition

한 번 발생한 사건을 현재 epoch에 기록한다.

현재 예:

```text
T_FLOOR_CONFIRMED
```

producer 위치:

```text
src/multifloor_manager/src/multifloor_manager/ros_callbacks.py
src/multifloor_manager/src/multifloor_manager/tag_evidence.py
src/multifloor_manager/src/multifloor_manager/ros_node.py
```

tag vote가 승인되는 순간 `ros_node.py`가 다음 observation을 기록한다.

```python
self.runtime.record_policy_observation(
    token,
    "T_FLOOR_CONFIRMED",
    PredicateObservation(True, time.monotonic(), token.epoch),
)
```

#### Live-level condition

최종 `READY` 직전에도 계속 true인지 확인해야 한다.

현재 예:

```text
T_LOCALIZED
T_COSTMAP_READY
```

producer/evaluation 위치:

```text
src/multifloor_manager/src/multifloor_manager/ros_node.py
src/multifloor_manager/src/multifloor_manager/ros_runtime.py
src/multifloor_manager/src/multifloor_manager/readiness.py
src/multifloor_manager/src/multifloor_manager/ros_callbacks.py
```

`_wait_for_policy_ready()`에서 최종 평가 직전에 observation을 새로 만든다.

```python
localization = self.runtime.localization_evidence()
self.runtime.record_policy_observation(
    token,
    "T_LOCALIZED",
    PredicateObservation(localization is not None and localization.ready, now, token.epoch),
)
```

### Step 4. registry 추가

파일:

```text
src/multifloor_manager/src/multifloor_manager/transitions.py
```

예를 들어 safety 조건 `T_WHEEL_ODOM_READY`를 추가할 때:

```python
SUPPORTED_CONDITION_NAMES: Final[Tuple[str, ...]] = (
    "T_FLOOR_CONFIRMED",
    "T_LOCALIZED",
    "T_COSTMAP_READY",
    "T_WHEEL_ODOM_READY",
)
MANDATORY_CONDITION_NAMES: Final[Tuple[str, ...]] = SUPPORTED_CONDITION_NAMES
```

non-safety를 추가할 때는 `SUPPORTED_CONDITION_NAMES`에는 넣되 `MANDATORY_CONDITION_NAMES`에는 넣지 않는 구조가 필요하다. 이때 parser도 optional condition을 허용하도록 함께 변경해야 한다.

### Step 5. parser 규칙 추가

파일:

```text
src/multifloor_manager/src/multifloor_manager/configuration.py
```

현재 `_load_transitions()`는 다음 순서로 검증한다.

1. policy document root key
2. schema version
3. configured flag
4. policy 개수
5. policy id와 enabled bit
6. condition key/name/중복
7. supported condition allowlist
8. mandatory condition 존재 여부
9. optional_count
10. freshness/dwell/timeout

새 safety 조건은 registry와 mandatory 목록에 넣으면 기존 mandatory validation이 적용된다.

새 non-safety 조건은 다음 parser 규칙을 별도로 구현해야 한다.

- safety condition만 `required: 1` 강제
- non-safety condition은 `required: 0` 허용
- `optional_count`가 활성화된 optional condition 수를 넘지 않도록 검증
- `optional_count`는 음수가 아니어야 함
- unknown/duplicate condition은 계속 거부

### Step 6. producer 연결

새 producer는 가능한 한 pure module에서 관찰값을 계산하고 ROS callback은 입력 경계와 epoch commit만 담당한다.

현재 module 역할:

| 관심사 | 파일 |
|---|---|
| AMCL pose/scan/odom readiness | `readiness.py` |
| AprilTag parsing/vote | `tag_evidence.py` |
| map fingerprint/generation | `map_evidence.py` |
| ROS callback과 active epoch commit | `ros_callbacks.py` |
| policy observation store/evaluator 호출 | `ros_runtime.py` |
| action orchestration/final gate | `ros_node.py` |

외부 action, command, URL, socket payload를 YAML condition에 넣지 않는다. 외부 동작은 별도 action과 completion observation으로 분리한다.

### Step 7. YAML와 fixture 수정

새 조건을 선언한다.

```yaml
- name: T_WHEEL_ODOM_READY
  enabled: 1
  required: 1
```

수정 대상:

```text
src/multifloor_manager/config/transitions.yaml
```

production 파일은 현재 `configured: false`라서 실제 policy list를 바로 활성화하지 않는다. 먼저 fixture에서 개발·검증하고, 현장 commissioning 때 production profile을 완성한다.

### Step 8. 테스트 추가

#### Pure evaluator

파일:

```text
src/multifloor_manager/test/test_transitions.py
```

최소 테스트 matrix:

| 입력 | 기대 결과 |
|---|---|
| 모든 required true | dwell 후 true |
| 새 조건 false | false |
| 새 조건 missing | false, `missing` |
| observation stale | false, `stale` |
| observation future-dated | false, `future` |
| wrong epoch | false, `wrong_epoch` |
| exact timeout | false, `timed_out: true` |
| condition false 후 true | 이전 dwell 폐기, 새 dwell 시작 |

#### Parser/site boundary

파일:

```text
test/test_site_configuration.py
```

테스트할 것:

- 새 이름이 allowlist에서 허용됨
- 누락 시 path-qualified error
- duplicate 시 path-qualified error
- safety가 `required: 0`이면 거부
- non-safety가 지원되는 경우 optional_count 규칙
- bit가 Boolean/string/2가 아니라 정확한 integer 0/1인지 확인

#### Final gate

파일:

```text
src/multifloor_manager/test/test_policy_final_gate.py
```

테스트할 것:

- live condition을 final gate에서 새로 샘플링
- 첫 평가가 true여도 두 번째 preemption check에서 cancel되면 commit 금지
- condition이 dwell 중 false가 되면 dwell 재시작
- costmap identity가 틀리면 dwell 재시작

#### ROS action

파일:

```text
src/multifloor_manager/test/test_floor_transition_ros.py
src/multifloor_manager/test/floor_transition_ros.test
src/multifloor_manager/test/floor_transition_policy_blocked_ros.test
```

정상 fixture에서는 `SUCCEEDED`와 target `READY`, blocked fixture에서는 `ABORTED`와 `FAULT`를 검증한다.

## 7. 상태머신

### 7.1 외부 상태: `FloorState`

파일:

```text
src/multifloor_manager/msg/FloorState.msg
```

```text
UNKNOWN      = 0
TRANSITIONING = 1
READY        = 2
FAULT        = 3
```

상태의 의미:

| 상태 | 의미 | 전이 가능 조건 |
|---|---|---|
| `UNKNOWN` | 초기 map 또는 초기 floor 상태를 아직 신뢰할 수 없음 | matching initial map 관찰 시 `READY` 가능 |
| `TRANSITIONING` | 하나의 active epoch이 map/tag/localization/costmap/policy를 처리 중 | 성공 시 `READY`, 실패/cancel 시 `FAULT` |
| `READY` | 현재 floor가 운영 가능한 상태 | 올바른 directed stair goal 수신 시 `TRANSITIONING` |
| `FAULT` | 전이 실패 또는 cancel 이후 terminal fault 상태 | 다음 정상 운영 cycle에서 명시적 복구 정책 필요 |

실제 publisher는 `RosEvidenceRuntime.publish_floor()`다.

```text
src/multifloor_manager/src/multifloor_manager/ros_runtime.py
```

`/multifloor/floor_state`에 latched message를 publish하며 다음 필드가 함께 전달된다.

- `floor_id`
- `map_generation`
- `state`
- `detail`
- ROS header timestamp

### 7.2 Action 계약

파일:

```text
src/multifloor_manager/action/FloorTransition.action
```

Goal:

```text
string transition_id
string target_floor
```

Result code:

```text
OK=0
BUSY=1
INVALID_GOAL=2
LOCALIZATION_FAILED=6
```

Feedback:

```text
string phase
string detail
```

Action endpoint:

```text
/multifloor/floor_transition
```

상태 topic:

```text
/multifloor/floor_state
```

### 7.3 내부 phase 순서

`ros_node.py::_run_transaction()`의 실제 phase 순서는 다음과 같다.

```text
READY
  │ valid directed goal
  ▼
ARM_TARGET
  │ arm epoch, tag route, map guard
  ▼
TRANSITIONING
  │ wait for target tag vote
  ▼
T_FLOOR_CONFIRMED
  │ change_map service
  ▼
CHANGE_MAP
  │ wait for post-arm matching map
  ▼
MAP_CONFIRM
  │ publish stored landing pose
  ▼
INITIALPOSE
  │ wait for newer AMCL pose samples
  ▼
AMCL_READY
  │ clear costmaps
  ▼
COSTMAP_READY
  │ wait for newer target-matching costmap
  ▼
POLICY_READY
  │ live predicates + dwell + preemption fence
  ├───────────────┐
  │ success       │ failure/cancel/timeout
  ▼               ▼
READY            FAULT
```

### 7.4 Goal validation

`_run_transaction()`은 실제 작업 전에 다음을 모두 확인한다.

- 현재 `FloorState`가 `READY`인가
- `transition_id`가 설정된 stair인가
- stair의 `from_floor`가 현재 floor인가
- stair의 `to_floor`가 요청한 target floor인가
- target map identity가 존재하는가

실패하면 action result가 `INVALID_GOAL`이 되고 map 변경이나 robot motion을 시작하지 않는다.

### 7.5 Busy, cancel, preemption

- `_active_lock`으로 동시에 하나의 transition만 실행한다.
- 이미 active이면 `BUSY`로 abort한다.
- `_wait()`와 final gate loop에서 `is_preempt_requested()`를 검사한다.
- policy evaluation이 true여도 commit 직전에 preemption을 다시 검사한다.
- cancel/preemption이면 `finish_transition(FAULT, ...)` 후 action을 preempted로 종료한다.

### 7.6 Epoch fencing

파일:

```text
src/multifloor_manager/src/multifloor_manager/ros_runtime.py
```

`arm_transition()`이 새 epoch와 `EpochToken`을 만든다.

```text
epoch N 시작
  ↓
EpochToken(N)
  ↓
observation 기록/평가/commit
```

callback은 입력을 lock 밖에서 parse할 수 있지만, commit할 때는 다음을 다시 확인한다.

```text
active_epoch == callback이 캡처한 epoch
```

다르면 해당 callback은 무시한다. `finish_transition()`은 epoch를 다시 증가시키고 active epoch을 해제하여 늦은 callback을 무효화한다.

## 8. 상태별 관리·수정 위치

| 바꾸려는 것 | 수정 파일 | 함께 수정할 테스트 |
|---|---|---|
| 조건 이름 allowlist | `transitions.py` | `test_site_configuration.py` |
| safety mandatory 목록 | `transitions.py` | parser invalid-case tests |
| YAML 시간값 | `config/transitions.yaml`, fixture YAML | site/config + blocked ROS test |
| tag vote 의미 | `tag_evidence.py` | tag evidence tests, normal ROS test |
| localization readiness | `readiness.py` | readiness tests, final gate tests |
| map/costmap evidence | `map_evidence.py`, `ros_callbacks.py` | map/costmap tests |
| policy observation 저장 | `ros_runtime.py` | epoch/final gate tests |
| phase/transition 순서 | `ros_node.py` | final gate + ROS action tests |
| 외부 상태 숫자/문자열 | `FloorState.msg` | action/state integration tests |
| action goal/result/feedback 계약 | `FloorTransition.action` | all action clients/rostests |

메시지나 action 파일을 바꾸면 Python source만 수정하는 것이 아니라 catkin message generation과 모든 action client/test를 재검증해야 한다.

## 9. 검증 명령

### 9.1 Local contract

```bash
./run.sh --check
```

이 명령은 site/bundle/launch 금지사항과 production disabled 상태를 확인한다.

### 9.2 Pure transition evaluator

```bash
PYTHONDONTWRITEBYTECODE=1 \
python3 -m unittest src/multifloor_manager/test/test_transitions.py -v
```

### 9.3 Site/YAML parser

```bash
PYTHONDONTWRITEBYTECODE=1 \
python3 -m unittest test/test_site_configuration.py -v
```

### 9.4 Fixture parity

```bash
PYTHONDONTWRITEBYTECODE=1 \
python3 -m unittest test/test_policy_fixture_parity.py -v
```

### 9.5 Final-gate unit tests

```bash
PYTHONDONTWRITEBYTECODE=1 \
python3 -m unittest src/multifloor_manager/test/test_policy_final_gate.py -v
```

### 9.6 Normal ROS action

```bash
ROS_IP=127.0.0.1 \
ROS_HOSTNAME=127.0.0.1 \
rostest --text multifloor_manager floor_transition_ros.test
```

기대 결과:

```text
RESULT SUCCESS
action SUCCEEDED
terminal FloorState.READY
```

### 9.7 Blocked policy ROS action

```bash
ROS_IP=127.0.0.1 \
ROS_HOSTNAME=127.0.0.1 \
rostest --text multifloor_manager floor_transition_policy_blocked_ros.test
```

기대 결과:

```text
RESULT SUCCESS
action ABORTED
terminal FloorState.FAULT
target floor READY 없음
```

### 9.8 Build and registered tests

```bash
catkin_make
catkin_make run_tests_multifloor_manager
```

### 9.9 LSP/Ruff

변경한 Python 파일마다:

```text
lsp_diagnostics
```

그리고:

```bash
ruff check \
  src/multifloor_manager/src/multifloor_manager/transitions.py \
  src/multifloor_manager/src/multifloor_manager/configuration.py \
  src/multifloor_manager/src/multifloor_manager/ros_runtime.py \
  src/multifloor_manager/src/multifloor_manager/ros_node.py
```

## 10. 실패 원인별 확인 순서

### `unknown condition`

확인:

1. `transitions.py`의 `SUPPORTED_CONDITION_NAMES`
2. YAML spelling과 대소문자
3. parser가 import하는 source가 `src/`인지 generated copy인지
4. 새 이름에 producer가 연결됐는지

### `missing mandatory condition`

확인:

1. `MANDATORY_CONDITION_NAMES`
2. fixture YAML의 conditions 목록
3. duplicate 제거 후 모든 필수 이름이 한 번씩 있는지

### `READY`가 되지 않고 `missing`

확인:

1. `ros_node.py`가 해당 이름으로 `record_policy_observation()`을 호출하는지
2. active epoch token이 맞는지
3. final gate 전에 `arm_transition()`이 호출됐는지
4. observations dictionary에 실제 key가 있는지

### `stale`

확인:

1. `freshness_sec`
2. ROS message header timestamp와 receive time
3. wall clock과 monotonic clock을 혼용하지 않았는지
4. fixture의 blocked freshness 값이 의도된 것인지

### `future`

producer가 관찰 시각으로 현재 시각보다 미래인 header timestamp를 넘기고 있다. 센서 clock/NTP와 timestamp source를 확인한다. 미래 evidence를 억지로 허용하지 않는다.

### `wrong_epoch`

늦은 callback 또는 이전 transition의 observation이다.

1. callback이 epoch를 캡처하는 위치 확인
2. lock 안에서 active epoch 재확인
3. `EpochToken`을 다른 transition에 재사용하지 않음
4. `finish_transition()` 후 callback이 commit하지 않는지 확인

### `READY` 직전에 `FAULT`

다음을 순서대로 확인한다.

1. 마지막 action feedback phase
2. `TransitionEvidence.conditions`
3. `timed_out`
4. preemption request 여부
5. `commit_ready()` 시점의 epoch
6. `T_LOCALIZED`와 `T_COSTMAP_READY`가 final gate에서 fresh한지

## 11. 새 조건 개발 체크리스트

### 설계

- [ ] 조건을 한 문장으로 정의했다.
- [ ] safety/non-safety를 분류했다.
- [ ] false일 때 READY를 허용해도 되는지 검토했다.
- [ ] event/live-level을 결정했다.
- [ ] 기존 조건과 중복되지 않는지 확인했다.
- [ ] 외부 command가 아닌 Boolean evidence로 표현했다.

### 구현

- [ ] `transitions.py` registry를 수정했다.
- [ ] `configuration.py` parser 규칙을 수정했다.
- [ ] producer를 올바른 pure/ROS boundary에 추가했다.
- [ ] `PredicateObservation(value, observed_at_sec, epoch)`를 생성한다.
- [ ] active epoch에서만 observation을 commit한다.
- [ ] YAML과 valid/blocked fixture를 수정했다.
- [ ] generated `build/`, `devel/`, `install/` 파일을 직접 수정하지 않았다.

### 테스트

- [ ] true/false
- [ ] missing
- [ ] stale
- [ ] future
- [ ] wrong epoch
- [ ] preemption
- [ ] exact timeout
- [ ] dwell interruption
- [ ] normal ROS action
- [ ] blocked ROS action
- [ ] parser invalid cases
- [ ] `./run.sh --check`
- [ ] `catkin_make`
- [ ] registered multifloor tests
- [ ] LSP/Ruff

### 운영 전환

- [ ] production YAML은 현장 값으로 검토했다.
- [ ] `configured: false`에서 `true`로 바꾸는 승인/기록이 있다.
- [ ] 실제 map/tag/costmap/AMCL/odom topic을 확인했다.
- [ ] 두 컴퓨터 clock synchronization을 확인했다.
- [ ] 실제 로봇에서 success/failure/cancel을 확인했다.
- [ ] `/multifloor/floor_state`와 action result를 관찰했다.

## 12. 운영 관찰 명령

```bash
rostopic echo -n 1 /multifloor/floor_state
rostopic echo /multifloor/floor_transition/feedback
rostopic echo /multifloor/floor_transition/result
```

현재 runtime은 조건별 debug Boolean도 publish한다.

```text
/multifloor/debug/<predicate-name>
```

예상 prefix:

```text
/multifloor/debug/tag_*
/multifloor/debug/map_*
/multifloor/debug/localization_*
/multifloor/debug/amcl_*
/multifloor/debug/scan_*
/multifloor/debug/odom_*
```

이 debug topic은 원인 관찰용이며 policy contract 자체를 대체하지 않는다. 최종 성공 여부는 action result, `FloorState`, policy evidence를 함께 확인한다.

## 13. 변경 금지 경계

전이조건 관리 기능을 추가한다는 이유로 다음을 섞지 않는다.

- 새 ROS node 추가
- 새로운 action/message 계약 추가
- `cmd_vel` 직접 publish
- robot WebSocket 직접 호출
- YAML에서 shell/network command 실행
- RViz goal tool을 통한 우회 이동
- 이전 epoch 결과 재사용
- stale/future observation을 현재 evidence로 승격

외부 장치 동작이 정말 필요하면 별도 typed action과 `T_<DEVICE>_CONFIRMED` safety confirmation을 설계하고, 해당 action의 실패·cancel·timeout을 층 전이 상태머신과 명확히 연결한다.
