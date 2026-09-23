# 검토와 근거 구분

sip/shower: 코드 실행부를 별도 context의 독자가 검토했다. evaluate/feedback 예외 처리, arm 거절 때 기존 hold 보존, 송신 직전 취소/시간 재검사를 반영했다. 후속 계약 검토는 변경 설명만 읽었으며 코드 전체 독립 감사라고 주장하지 않는다. 실제 transport의 ACK polling은 저장소 source에서 확인했다. ACK의 실제 모드/물리 상한은 UNVERIFIED로 문서화했다.

운영 안내 냉독: 종료·RC 인계 절차 누락을 지적해 기존 cancel 및 acknowledge_physical_handoff 인터페이스와 모드 변경 의미를 추가했다. 생략 대상은 phase_test_limits라고 원문에 명시되어 있다. 냉독에 전달한 일반 텍스트에는 링크를 생략했지만 최종 MD에는 보고서·작업 폴더 직접 링크가 있다.

factchk: ROS 동작은 설치된 actionlib/action_client.py cancel_all_goals와 rosbag/rosbag_main.py의 -O/--split/--size 옵션을 확인했다. 이 저장소의 mode/phase/소유권 계약은 해당 코드·시험으로 확인했다. 현재 TRON runtime 관찰로 바꾸어 표현하지 않았다.

mandela: 수동 BAG pose는 가상 명령을 따르지 않는다. 동일 LiDAR로 만든 지도-궤적 정합은 독립 정확도 검증이 아니다. 합성 unicycle은 횡방향/방향 오차 감소의 코드 반응만 검증한다. 기하 오라클은 별도 면적 계산이지만 같은 로컬 작성자의 검증으로 실기 경계 정밀도를 증명하지 않는다.

ssotize(read-only): phase/완료 동작의 권위 원천은 stair_feedback.py/supervisor.py, 현재 적용 안내는 APPLICATION_GUIDE.md, HTML 변환본은 역사 기록이다. 코드의 from_entry/phase_test_limits 검색과 문서의 '계획/미반영/아직' 검색을 대조했다. 과거 보관본문을 수정/통합하지 않고 현재 안내만 구현 상태로 갱신했다.

re0: 현재 안내를 현재 상태·실행 경로·확인할 값·종료/인계·남은 입력 중심으로 다시 썼다. 이전 문서화 작업의 과정 설명은 과거 결과 링크로 보존했다. portability 주장 없음으로 detool 생략.
