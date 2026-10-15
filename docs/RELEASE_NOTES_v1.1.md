# Release v1.1 — Nav2 기반 박물관 도슨트 로봇

## 이번 버전

ROS2 Humble/C++로 구현한 5개 노드의 Patrol·Guide·Error/Recovery 구조를 TurtleBot3 Burger, Gazebo 공식 World, Nav2와 연결했습니다. 공식 World의 9개 기둥 중 왼쪽 첫째·둘째·셋째를 가상 작품 위치로 사용했습니다.

## 구현된 변경

- 실제 Nav2 `NavigateToPose` 기반 Waypoint 순찰·안내
- Action Feedback/Cancel/Result(SUCCEEDED, CANCELED, ABORTED) 연계
- MainController의 WorkPhase 기반 작업 상태 통합
- Goal 중복 수락·늦은 결과 처리 방어
- 점검 Stop(작업 종료→Start 복귀→Idle)
- 명시적인 Recovery 완료 이벤트(`/recovery_result`)와 재시도

## 검증

정상 Patrol 순환, Guide 수락·거절 및 작품 설명, Waiting/Guide 중 점검 Stop, 중복 요청 차단, Nav2 실제 ABORTED→Error, Recovery 실패 유지·재시도 성공, 최종 Waypoint 원복을 2026-10-15 수동 테스트에서 확인했습니다.

## 제한

별도 Museum Gazebo World/Unity 및 실물 로봇 구현은 포함되지 않습니다. 통합 Launch와 자동화·장시간 안정성 검증은 향후 과제입니다.
