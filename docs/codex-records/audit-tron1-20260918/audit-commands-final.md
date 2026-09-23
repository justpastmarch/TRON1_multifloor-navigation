# 감사 명령·실행·산출물 원장

이 원장은 수행한 작업의 종류와 보존된 실행 근거를 연결한다. 모든 대화의 shell 명령을 처음부터 기록한 transcript라고 주장하지 않는다. 정확한 시험 결과·probe 코드·runtime 관찰은 각 원문에 보존돼 있다. 아래 감사 script는 로봇 운용 명령이 아니며 반복 실행을 요청하는 안내도 아니다.

| 수행 범위 | 실행한 명령/방법 | 관찰·산출물 | 상태 변경 경계 |
|---|---|---|---|
| inventory/Git baseline | `rg --files`, filesystem walk/lstat/hash; `git --no-optional-locks -C <repository> rev-parse HEAD`, `status --porcelain=v2 --branch --untracked-files=all`, `diff`, `diff --cached` | coverage.json, baseline-status.txt, baseline-diff.patch, baseline-staged.patch | target 읽기 전용; 최초 baseline 보존 |
| 파일 의미 검토 | `cat`, `sed`, `nl`, `rg`; Python으로 YAML/XML/JSON/AST, CSV·log 인덱스 읽기 | phase 기록 및 read-batch-*.json; lossless log index와원문hash 대조 | parse·검색만으로 READ 인정하지 않음 |
| 설치·공식 upstream | 설치 /opt/ros/noetic 관련 source 읽기; 공식 ros-planning/navigation source 수집 | upstream/manifest*.json의 URL·SHA; phase-D/보안 기록 | upstream source와 실제 binary/runtime 동일성 미확인 |
| source/install 비교 | 78개 대응 파일 bytes 비교, devel loader 및 generated action 필드 읽기 | phase-E-security-final.md | generated 전체를 READ로 올리지 않음 |
| 초기 선택 unit/contract | 검토한 unit3module 10개, read-only contract6개를 Python unittest로 실행 | selected-unit-results.json(10/10), selected-contract-results.json(6/5/1) | socket 금지·subprocess fake; ROS/로봇 연결 없음 |
| 추가 pure unit | selected-tests-resume2.py의 검토된 suite, socket connect/bind·subprocess guard | resume2-unit-results.json, selected-tests-resume2.json:60/60 | 한정 temp root의 파일 작업만 허용, 정리 receipt |
| 추가 contract | `python3 -B .../selected-contracts-resume4.py` | selected-contracts-resume4.json:12/10/2 | file/XML/문자열 계약; network/subprocess/target쓰기 없음 |
| 순수 반례 | production planner/evidence/FSM 코드+메모리 peer; logic-probe-resume2.py 등 | pure-C-probe.json, logic-probe-resume2.json, selected-tests-resume2.json | 새 goal/service/velocity 없음. 실제 코드 반례를 실장비 사고로 세지 않음 |
| 로드 중단 | generated srv import 부재; uuid/platform subprocess guard 차단 | selected-unit-results.json의 previous_attempt, phase-E-resume2.md:9, cleanup receipt | 적어도2회 각각0개실행;88시험합계에 포함하지 않음 |
| bag/CSV/log/image/video | rosbag 파일 read-only metadata/선택 메시지, CSV50618행, log lossless/template 대조, JPEG64개 개별 시각 열람, MP4280frame decode metadata | bag-*.json, csv/log/video/map metadata와read-batch 원장 | ROS playback·운용master 주입 없음. binary bulk는metadata_only |
| 로컬 process/network 관찰 | `date -u`, `ps -eo pid,ppid,stat,comm`의ROS/SSH/Python 필터, `ss -ltn` | phase-B.md: 02:42:09UTC. 일치 행 없음;ss netlink권한거절 | 이 namespace 관찰로host/장비ROS정지를 단정하지 않음 |
| Mini PC read-only 관찰 | SSH key 인증 뒤 date/hostname, systemctl --user is-active/is-enabled/show, ps/pgrep 및 sensor launch/bridge wrapper 원문 | remote-sensor-observation.txt, phase-C-continuation.md:02:52:52/02:53:32UTC | process/service상태 조회만. 비밀번호 사용·저장 없음 |
| 보고서 조립·검토 | 외부 OUT에서 Python JSON/Markdown 생성, render-final-report.py, 근거경로·행·hash·schema 대조, 문서copy cold read | FINAL-AUDIT.md, final 원장, 최종 검토 기록 | target 수정 없음; 보존 보고서만 작성 |
| 종료 무결성 | `python3 -B .../check-integrity.py` | final-integrity-check.json | target read-only; OUT receipt만 갱신 |

## 실행하지 않은 것

옵션 없는 run.sh, mission/navigation goal, cmd_vel/initialpose, Robot WebSocket command, 상태 변경 ROS service, dynamic_reconfigure/map/parameter 변경, Mini PC process 시작/종료, systemd enable/disable/start/stop, ROS replay, 통합 launch, 실장비 주행은 실행하지 않았다. `run.sh --check`는 허용된 명령이지만 이번 실행 근거가 없어 실행 목록에 넣지 않았다.

## Artifact·process 정리

시험 temporary root3개와 생성·제거 이벤트는 resume2-loader-attempt0-cleanup.json, resume2-unit-run-cleanup.json, resume2-temp-artifacts.json에 기록했다. 각 root가 삭제됐고 remaining_threads=[]였음을 종료 대조에서 다시 확인한다. final shower에 준 보고서 copy는 final-review-temp-receipt.json에 생성 시 정리 대상으로 등록했고 검토 종료 후 제거한다. 최종 상태는 artifact-cleanup-final.json이 기준이다.

모든 분석 명령은 foreground로 종료되며 robot/ROS/background runtime process를 생성하지 않았다. 본 디렉터리의 보고서·원장·원문snapshot·검증script·결과·baseline은 사용자에게 넘길 보존 산출물이다. 기존 worktree의 log/cache/사용자 process는 감사 artifact가 아니므로 건드리지 않는다.
