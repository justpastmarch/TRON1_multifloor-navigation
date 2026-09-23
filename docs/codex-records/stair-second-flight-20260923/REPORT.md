# 계단참에서 두 번째 상승 구간 단독 시험

기존 stair_entry_test.py 클라이언트에 preview --reuse-reference, run --single-phase 옵션을 추가했다. 기존 profile_from_local은 그대로 유지하며 같은 추적 epoch의 현재 odom stamp로 capture_entry 근거를 갱신한다. 현재 위치를 첫 진입점이나 새 위치로 재지정하지 않는다. 서버의 기존 from_entry=false 프로토콜과 bound-ticket 취소 경로를 사용한다. 기존 Supervisor·설정·노드 구성은 변경하지 않았고 재시작하지 않았다.

명령: preview CONFIG --reuse-reference 후 run CONFIG --single-phase --phase FORWARD_SEGMENT_2 --seconds 45. 전체 몸체의 계단참 지지영역과 높이, 신선한 관측 검사는 기존 서버가 수행한다. 이 시험은 두 번째 상승만 실행하고 이후 0 명령과 STAIR 소유권을 유지한다. EXIT_CONFIRM이나 정상 층 이동 완료를 주장하지 않는다. 0 명령은 물리적 정지를 보장하지 않는다.

OBSERVED: 95개 테스트 통과. 기준 좌표 보존, 원본 근거 불변, epoch/route 불일치 거부를 추가 검증했다. 새 주행 목표는 보내지 않았다. 실제 단독 상승 성공은 UNVERIFIED.

현재 관찰 pose-check.json: x=2.535, y=1.339, z=1.534m, yaw=177.67도. 계단참 높이 오차 0.004m, 방향은 두 번째 상승방향에 가깝다. 현재는 full-body landing margin 검사가 실패한다. 첫 단 시작은 x=2.52이고 두 번째 상승 방향은 -x이다. 따라서 사용자가 두 번째 첫 단의 45cm 앞 평지에 중심을 놓고 위를 향하게 한 후 실행하도록 안내한다. 현재 위치에서 출발 가능하다고 주장하지 않는다. 위치 수치는 LiDAR 추정이며 실측 진실값이 아니다.

독립 cold-read: 현재 자세 검사와 stamp/epoch 일치가 필수임을 확인했다. service는 현재 샘플과 epoch 및 timestamp 일치를 재검사하고 기존 고정 좌표를 현재 pose에 합성한다. FORWARD_SEGMENT_2의 방향·영역은 기존 제어기의 flight_2를 사용한다. 원본·patch·테스트·조회 기록은 증빙으로 보존했다. 임시 ROS 로그는 삭제했다.
