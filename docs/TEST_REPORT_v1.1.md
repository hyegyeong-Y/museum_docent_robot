# v1.1 수동 통합 테스트 기록

- 검증일: 2026-10-15
- 환경: Windows WSL2 Ubuntu 22.04, ROS2 Humble, TurtleBot3 Burger, Gazebo Classic, Nav2, RViz
- World: 공식 turtlebot3_world
- Map: ~/turtlebot3_ws/maps/turtlebot3_world.yaml
- 사용자 제공 터미널 로그에 근거한 수동 검증이며 CI/자동 테스트 결과가 아닙니다.

## 정상 및 회귀 테스트

| ID | 시나리오 | 결과 | 관찰한 핵심 증거 |
|---|---|---|---|
| N01 | 빌드 | 통과 | 초기 symlink 충돌 정리, std::make_shared 컴파일 수정 후 빌드 성공 보고 |
| N02 | Patrol 전체 경로 | 통과 | Movement Start→Artwork1→Artwork2→Artwork3→Goal→Start Goal 수신 |
| N03 | User Request 이동 Cancel | 통과 | Cancel requested 및 StateManager Waiting |
| N04 | Guide Yes | 통과 | Artwork2 안내 수락, 작품 설명 출력 |
| N05 | Guide No | 통과 | Artwork1 취소 후 동일 Artwork1 재요청, 다음 Artwork2 |
| N06 | Waiting 중 Stop | 통과 | Stop 요청 수락, Start 복귀, Idle |
| N07 | Guide 이동 중 Stop | 통과 | Artwork2 취소→Start Goal, Idle |
| N08 | Stop 후 Patrol 재시작 | 통과 | /start_signal success=True 및 Movement Start 수신 |
| N09 | 중복 Start / 중복 Stop | 통과 | 두 번째 요청 각각 success=False |

## Error 및 Recovery

| ID | 시나리오 | 결과 | 관찰한 핵심 증거 |
|---|---|---|---|
| E01 | 강제 Error에서 정상 요청 거부 | 통과 | start_signal false, user_request false, stop_patrol false |
| E02 | Recovery 중복 요청 거부 | 통과 | 첫 요청 Recovery queued, 다음 Recovery request rejected |
| E03 | Recovery 성공 → Idle | 통과 | Recovery Start goal accepted / Recovery success: Start / robot_state Idle / recovery_result SUCCEEDED |
| E04 | Nav2 실제 ABORTED | 통과 | (100,100) 목표의 off global costmap, bt_navigator Goal failed, Action ABORTED |
| E05 | Nav2 실패 → 상위 Error | 통과 | 임시 Start 목표 이동 실패 후 robot_state Error |
| E06 | Recovery 실패 후 Error 유지 | 통과 | Recovery failed, robot_state Error |
| E07 | 원복 뒤 Recovery 재시도 | 통과 | Recovery Start goal accepted → Recovery success: Start |
| E08 | 임시 설정 원복 | 통과 | 최종 robot_state Idle, Start (1.228803,2.333385,3.141593) |

## 테스트 해석과 범위

- 서비스의 success는 요청 접수를 의미하는 경우가 있으므로 결과·상태 로그를 함께 확인했습니다.
- 단독 /navigate_to_pose ABORTED 테스트와, 임시 waypoint를 사용한 상위 프로젝트 Error 통합 테스트를 별도로 수행했습니다.
- Nav2 내부 BT Recovery(예: Costmap Clear)를 거쳤고 최종 실패할 때 ErrorHandler 흐름으로 넘어갔습니다.
- 높은 부하에서의 모든 취소/콜백 경합, Service 응답 미수신 Timeout, 자동 회귀 테스트, 실물 하드웨어 안전성은 별도 검증하지 않았습니다.

## v1.1 범위 결정

- 공식 World의 왼쪽 세 기둥을 작품 위치로 가정한 시나리오를 최종 범위로 확정.
- 신규 Museum Gazebo World 또는 Unity 환경 구축은 별도 버전에서 진행.
- 기존 검증 코드는 추가 기능을 위한 불필요한 수정을 하지 않기로 결정.
