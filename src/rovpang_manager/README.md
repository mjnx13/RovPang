# rovpang_manager (팀원 1)

배송 작업 관리(F-14), 수령자 인증(F-10), 시험용 가짜 노드. 가짜 노드만 쓰면 로봇은 움직이지 않는다.

| 실행 파일 | 역할 |
| --- | --- |
| task_manager | 배송 상태 머신, /rovpang/status 발행, ~/rovpang_logs/deliveries.csv 기록, /rovpang/cancel |
| (모듈) recipient_auth | 배송별 일회용 QR 토큰 등록·검증 (task_manager가 사용, ROS 없이 단위 시험 가능) |
| (모듈) alerts | 오류 코드별 부저 소리 종류·안내 문장 (/rovpang/alert, 로봇 부저) |
| mock_robot | 팀원 2의 pick / place / go_to / set_arm_pose 흉내 (fail_on 으로 실패 시험) |
| mock_vision | 팀원 1의 detections / sign / qr 흉내 (wrong_sign, wrong_qr, wrong_qr_for) |
| send_request | 관리자 프로그램 대신 배송 요청 1건 발행 |

```bash
colcon build --packages-select rovpang_interfaces rovpang_manager
colcon test --packages-select rovpang_manager   # recipient_auth·alerts 단위 시험
source install/setup.zsh
ros2 launch rovpang_manager mock_test.launch.py            # 창 1
ros2 topic echo /rovpang/status                            # 창 2
ros2 run rovpang_manager send_request --dest A --item buds # 창 3
ros2 service call /rovpang/cancel std_srvs/srv/Trigger     # 취소 시험
```
실패 시험: `fail_on:=pick` / `fail_on:=go_to` / `wrong_sign:=true` / `wrong_qr:=true` (QR 2번 모두 틀림) / `wrong_qr_for:=3.0` (첫 QR만 틀림)

QR은 2번 기회(qr_attempts), 모두 틀리면 CARRY 자세로 물품을 가지고 복귀. 예외 시 `/rovpang/alert` 발행과 부저 비프음(use_buzzer, 음성 없음).

복귀하지 못하고 멈추면 '복귀 필요' 상태가 되어 새 요청을 거절한다. 원인 해결 후 `ros2 service call /rovpang/return_home std_srvs/srv/Trigger`.
시험: `wrong_qr:=true fail_on:=arm_away` → `ros2 param set /mock_robot fail_on none` → return_home 호출.
