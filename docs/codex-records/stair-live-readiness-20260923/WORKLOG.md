# Live driving readiness

Understood as: 기존 ROS 노드 소스·구성과 측위 방식을 바꾸지 않고 운영 실행 설정을 configured=true/control에 연결하고, 정상 주행을 막는 실제 원인을 확인·수정한다. 설정 완료를 물리 주행 검증과 혼동하지 않는다.

현재 미니PC 192.168.1.56 SSH가 No route to host. 로봇 위치·전원 정보를 비동기 요청했고 소프트웨어/설정 작업은 계속한다.

변경 범위 후보: config.env, 기존 run.sh 및 stair_control_session.sh, 현재 실행 문서. 새 상시 노드 및 ROS 노드 소스 변경 없음. readchk를 적용해 범위를 확인했고 mandela 검토에 따라 합성 제어 검증을 물리 성공으로 표시하지 않는다.

## 적용 결과

실제 저장소 11개 파일 반영. src 전체 해시 동일: ROS 노드 소스/launch/메시지/action/제어 설정 변화 없음. config.env가 기존 configured=true 시험 설정과 control 모드/자동 기록을 기본으로 연결. 기존 실행 전 검사에서 일회성 클라이언트·문서 보관 영역을 정확히 구분하고 ROS 환경 로딩 순서, 종료신호, 원격실패 전달을 수정. 관련 시험 27개 PASS. 실제 배포 run.sh --preflight의 로컬/운영 설정 검사 PASS, 미니PC 연결 단계 No route to host(exit255).

Kinematic 모델 3조건 전체 단계 통과는 소프트웨어 일관성 근거만이며 물리 성공으로 사용하지 않음. commissioned=false 유지. 불확실한 실제 위치·균형/센서 지연을 검증 없이 승인하지 않음.

미완료: 사용자 장비 전원/네트워크와 현재 위치 응답 필요. 다음 시작점: 192.168.1.56 연결 확인 → 현재 로봇 위치 확인 → 기존 stair_control_session.sh start → runtime tracking_status/config hash/control 모드 → 지정 배치 preview → 첫 flight/회전/상부 시험. 정상 /mission 층 이동 commissioning은 실기 근거 후 확인.

시험 임시파일 1909개 약130MB 제거, ROS/시험 프로세스 없음. 기존 변경 및 작업 디렉터리 보존. 정리/배포/검증 구조화 기록은 이 폴더와 실제 docs/stair-drive-readiness-20260923/evidence/live-readiness.json에 저장. 물리 주행 완료 상태로 보고하지 않음.
