# TRON live sensor check — 2026-09-21

센서 스택과 기존 Stair Supervisor 관측 전용 측위를 실행한 상태입니다. 이 기록은 실시간 입력 확인이며 실제 자동 등반 수락이 아닙니다.

- **OBSERVED:** Mini PC SSH(192.168.1.56), TRON API(10.192.1.2:5000), 기존 onboard ROS master 응답 확인.
- **OBSERVED:** 기존 설정대로 제어 PC master `http://192.168.1.26:11311` 실행, Mini PC `wf_mapping` 세션에서 LiDAR·IMU·wheel odom·scan·D435 카메라 실행. 기존 onboard graph는 변경하지 않음.
- **OBSERVED:** Supervisor 실행 파라미터 `lidar_mode=observe`, `lidar_observe_only=true`. RobotTransport 미생성 경로이며 주행 명령 publisher 없음.
- **OBSERVED:** 20초간 odom 100개, 약 5Hz. 고유 추적 표본 100개 모두 TRACKED, epoch 0 유지, 누락/worker error 0.
- **UNVERIFIED:** 실제 위치 정확도·밀림 보정·자동 등반·전체 mission/RViz 부하.

## 수신 지연 finding

**OBSERVED; Safety S2 / Mobility M2:** 원본 컬러/깊이 영상을 동시에 짧게 구독한 초기 측정에서 LiDAR 수신 나이 최대 1.257초. 영상 구독 종료 뒤 측위만 실행한 창의 정확한 처리/전달 지연은 `tracking-summary.json` 참고.

**INFERRED:** 원본 영상 전송과 네트워크/수신 부하의 영향 가능성. 통제된 A/B 실험은 아니므로 원인을 확정하지 않음. 제어 age 한계를 임의로 늘리거나 stamp를 바꾸지 않음.

## 실행 수명과 기록

사용자가 실행을 요청한 센서·관측 프로세스 및 그 운용 로그는 의도적으로 유지합니다. 임시 수신율/측위 구독은 종료하고 구독 해제했습니다. 원본 저장소 Git 상태는 실행 전후 동일합니다. 상세 PID·경로는 `state.json`에 있습니다.

- 제어 PC master 및 관측 전용 Supervisor: `state.json`의 `owned_processes`
- Mini PC 센서: tmux 세션 `wf_mapping`
- ROS 진단: `/stair_supervisor/tracking_status`, `/stair_supervisor/lidar_odom`
- RViz에서 관측 odom을 볼 경우 현재 frame은 `stair_local_0`. 이 점검에서는 RViz를 새로 실행하지 않았습니다.

전체 실행 스크립트는 별도 master/명령 소유자를 시작하므로 현재 관측 세션과 중복 실행하지 않도록 인계가 필요합니다. 자동 주행/계단 제어는 활성화하지 않았습니다.
