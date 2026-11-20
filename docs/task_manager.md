# 배송 작업 관리 노드 (rovpang_manager / task_manager) – 시험 결과와 연동 시 주의할 점

작성: 정서인 (팀원 1) · 2026-11-19 작성, 2026-11-20 패키지 이름 변경(rovpang_mission → rovpang_manager), QR 2회 기회·예외 알림 추가

## 1. 무엇을 하는 패키지인가

| 실행 파일 | 역할 |
| --- | --- |
| task_manager | F-14 배송 상태 관리. 요청을 받아 팀원 2의 액션·서비스를 순서대로 호출하고 /rovpang/status 발행, ~/rovpang_logs/deliveries.csv 기록 |
| recipient_auth (모듈) | F-10 배송별 일회용 QR 토큰 등록·검증. QR_INVALID / QR_REUSED 판정 |
| alerts (모듈) | F-16 오류 코드별 부저 소리 종류와 사용자 안내 문장 |
| mock_robot | 팀원 2 기능(pick, place, go_to, set_arm_pose)을 흉내 내는 가짜 노드. 로봇은 움직이지 않음 |
| mock_vision | 팀원 1 비전 노드(detections, sign, qr)를 흉내 내는 가짜 노드 |
| send_request | 관리자 프로그램 대신 배송 요청 1건 발행 |

상태 흐름: IDLE → DETECTING → PICKING → MOVING → VERIFYING → PLACING → RETURNING → DONE (실패 FAILED, 취소 CANCELED)

### QR 인증 기회 (11/20 추가)
- QR은 2번 기회를 준다(qr_attempts=2). 첫 QR이 틀리면 짧은 소리 1번과 "QR을 다시 보여 주세요" 안내를 낸다. 2초(qr_retry_delay) 뒤에 새로 읽힌 QR만 다시 확인한다.
- 2번 모두 틀리면 하역하지 않는다. 팔을 **CARRY 자세로 바꿔 물품을 쥔 채** 출발지로 돌아간다. 하역 후 복귀할 때는 HOME 자세를 쓴다.
- 복귀 전에 팔 자세를 바꾸지 못하면, 팔을 뻗은 채 주행하지 않도록 **멈추고** ARM_POSE_FAILED 알림을 낸다.
- qr_timeout(30초)은 두 번의 기회를 합친 시간이다. NOT_REGISTERED는 프로그램 쪽 문제라 다시 시도하지 않는다.

### 복귀 필요 상태와 수동 복귀 (11/20 추가)
- 출발지로 돌아오지 못하고 멈추면 **복귀 필요** 상태가 된다. 원인은 복귀 전 팔 자세 실패(ARM_POSE_FAILED)나 복귀 주행 실패(RETURN_FAILED)다.
- 복귀 필요 상태에서는 새 배송 요청을 **거절**하고 NEED_RETURN 알림을 낸다. 출발지가 아닌 곳에서 물품을 찾기 시작하는 것을 막기 위해서다.
- 관리자가 원인(서보 전원, 걸림, 경로 막힘 등)을 해결한 뒤 아래 명령을 호출하면, 저장된 팔 자세(하역 전 CARRY, 하역 후 HOME) → 출발지 이동을 다시 시도한다. 성공하면 IDLE로 돌아가 다음 요청을 받는다.
  ```
  ros2 service call /rovpang/return_home std_srvs/srv/Trigger
  ```
  관리자 프로그램(admin_pc)의 "복귀" 버튼이 이 서비스를 호출한다. 다시 실패하면 복귀 필요 상태가 유지된다.
- 팔이 계속 움직이지 않으면 하드웨어 문제이므로 전원을 끄고 물품을 손으로 빼낸 뒤 로봇을 출발지로 옮긴다. 그다음 task_manager를 다시 실행한다(복귀 필요 상태는 노드를 다시 켜면 풀린다).

### 예외 알림 (11/20 추가)
예외가 생기면 이유를 `/rovpang/alert`(std_msgs/String)로 발행한다(관리자 프로그램 F-15가 표시). 로봇 부저도 울린다(`/ros_robot_controller/set_buzzer`, ros_robot_controller_msgs/BuzzerState).
PC처럼 이 메시지가 없는 환경에서는 소리 없이 안내 문장만 발행한다. 부저를 끄려면 `use_buzzer:=false`로 실행한다.

| 소리 | 의미 | 오류 코드 |
| --- | --- | --- |
| 짧게 1번 | 다시 시도해 주세요 | QR_RETRY |
| 짧게 2번 | 배송지·수령자 인증 실패 | SIGN_NOT_FOUND, WRONG_DESTINATION, QR_INVALID, QR_REUSED, QR_TIMEOUT |
| 짧게 3번 | 물품 처리 실패 | ITEM_NOT_FOUND, PICK_FAILED, PLACE_FAILED |
| 길게 1번 | 주행·로봇 동작 오류 | NAV_FAILED, RETURN_FAILED, TIMEOUT, SERVER_UNAVAILABLE, GOAL_REJECTED, ARM_POSE_FAILED, NOT_REGISTERED, INTERNAL_ERROR, NEED_RETURN |
| 없음 | 관리자 취소 | CANCELED, RETURN_CANCELED |

예: `[QR_INVALID] 이 배송의 QR이 아닙니다. 물품을 가지고 출발지로 돌아갑니다`

## 2. 가짜 노드 시험 결과 (2026-11-19, 로봇 도커 Humble, 이름 변경 전 rovpang_mission으로 시험)

| 시험 | 실행 옵션 | 결과 |
| --- | --- | --- |
| 정상 배송 | (없음) | DONE, 약 11.7초 |
| 집기 실패 | fail_on:=pick | FAILED / PICK_FAILED, 출발 전이라 복귀 없음 |
| 다른 배송지 | wrong_sign:=true | FAILED / WRONG_DESTINATION (요청 A / 인식 B 5/5), 하역 안 함, 출발지 복귀 |
| 틀린 QR | wrong_qr:=true | FAILED / QR_INVALID, 하역 안 함, 출발지 복귀 |
| 이동 중 취소 | delay:=10.0 | CANCELED, 출발지 복귀 성공 |
| 복귀 중 취소 | delay:=10.0 | DONE / RETURN_CANCELED (하역까지 끝났으므로 배송 성공으로 기록) |

### 가짜 노드 시험 결과 (2026-11-20, qr-retry-alert 브랜치)

| 시험 | 실행 옵션 | 결과 |
| --- | --- | --- |
| 단위 시험 | colcon test | 11개 통과 (recipient_auth 5, alerts 6) |
| 첫 QR만 틀림 | wrong_qr_for:=3.0 | QR 불일치 1/2회 안내 → 약 2초 뒤 두 번째 QR로 인증 → DONE |
| QR 2번 모두 틀림 | wrong_qr:=true | FAILED / QR_INVALID (2/2회), 하역 안 함, CARRY 자세로 바꾼 뒤 출발지 복귀 성공 |
| 복귀 전 멈춤 | wrong_qr:=true fail_on:=arm_away | CARRY 실패 → 주행 안 함, 새 요청 NEED_RETURN 거절, 고장 해결 후 return_home → IDLE → 새 요청 정상 접수 |
| 부저 연결 | ros2 topic info /ros_robot_controller/set_buzzer | BuzzerState(freq, on_time, off_time, repeat), 구독 1 (ros_robot_controller 실행 중) |

wrong_qr_for:=1.0은 팔 LOOK 전환(약 0.5초)과 표지판 5장 수집(약 1초)이 끝나기 전에 틀린 QR이 끝나 두 번째 기회가 시험되지 않았다. 3.0 이상으로 시험한다.

## 3. 팀원 2 기능과 연동할 때 주의할 점

mock_robot 자리에 실제 노드가 들어갈 때 아래를 지켜야 지금 시험 결과가 그대로 유지된다.

1. **go_to 취소 처리 (중요)**
   이동 중 취소되면 배송 작업 관리 노드는 곧바로 go_to(START)를 다시 보낸다.
   실제 go_to 액션은 취소 요청을 받으면 Nav2 목표를 취소해 **로봇을 먼저 멈춘 뒤** canceled로 끝나야 한다.
   그렇지 않으면 이전 목적지와 출발지 이동이 겹칠 수 있다.
2. **pick 목표 좌표계**
   Pick.Goal.target은 /rovpang/detections의 header.frame_id(카메라 좌표계) 그대로 보낸다.
   팔 좌표계 변환은 팀원 2가 TF로 처리한다. 실패 시 result.success=false와 reason을 채워야 오류가 기록된다.
3. **place**
   지금은 use_default=true(정해진 하역대 위치)만 쓴다.
4. **set_arm_pose (HOME / CARRY / LOOK)**
   인증에 실패하면 VERIFYING의 LOOK 자세에서 **물품을 쥔 채** CARRY로 바꾸라는 요청이 온다. 물품을 쥔 상태에서도 LOOK → CARRY 전환이 되어야 한다.
   LOOK 자세에서 물품·표지판·QR이 카메라에 보여야 하고, 깊이 카메라 특성상 카메라–대상 거리를 25~30cm 이상 두어야 한다(약 20cm 이내는 깊이 없음).
5. **액션 서버 구조**
   액션 콜백 안에서 spin_until_future_complete를 쓰면 멈출 수 있다.
   MultiThreadedExecutor + ReentrantCallbackGroup으로 만들고 서비스 결과는 future가 끝날 때까지 기다리는 방식이 안전하다(mock_robot.py 참고).
6. **시간 제한**
   배송 작업 관리 노드는 액션 서버를 5초 기다리고, 액션 결과는 최대 60초(action_timeout), 팔 자세 서비스는 15초 기다린다. 넘으면 TIMEOUT으로 실패 처리한다.
7. **부저**
   task_manager는 ros_robot_controller 노드의 `/ros_robot_controller/set_buzzer`로 소리를 낸다(팔 서보와 같은 노드). 이 노드가 실행 중이어야 소리가 난다.

## 4. 집기·하역 결과 확인 방법 (11/20 팀장 확인)

F-05·F-12는 그리퍼 상태 + 카메라 재탐지로 확인한다.
하역 확인 순서: 그리퍼 열기 → 로봇팔 안전 위치로 이동 → 카메라 재탐지 → 하역 위치 검증.
하역 위치는 표지판 받침대 앞쪽 다리 사이 영역이다.

## 5. 실행 방법

```bash
cd /home/ubuntu/share/tmp/<워크스페이스>
colcon build --packages-select rovpang_interfaces rovpang_manager
source install/setup.zsh
colcon test --packages-select rovpang_manager             # recipient_auth·alerts 단위 시험
ros2 launch rovpang_manager mock_test.launch.py            # 창 1 (옵션: fail_on, wrong_sign, wrong_qr, wrong_qr_for, delay)
ros2 topic echo /rovpang/alert                             # 창 2 (예외 안내 문장)
ros2 run rovpang_manager send_request --dest A --item buds # 창 3
ros2 service call /rovpang/cancel std_srvs/srv/Trigger     # 취소
```
delay, wrong_qr_for 값은 10.0, 3.0처럼 소수점을 붙여야 한다.
- `wrong_qr:=true`: QR 2번 모두 틀림 → FAILED / QR_INVALID, CARRY 자세로 복귀
- `wrong_qr_for:=3.0`: 첫 QR만 틀림 → 다시 보여 주세요 안내 후 두 번째에 성공 → DONE
- `wrong_qr:=true fail_on:=arm_away`: QR 실패 → 배송지에서 CARRY 자세 실패 → 멈춤(복귀 필요). 이 상태에서 새 요청은 거절된다.
  `ros2 param set /mock_robot fail_on none`(고장 해결) → `ros2 service call /rovpang/return_home std_srvs/srv/Trigger` → 복귀 후 IDLE
