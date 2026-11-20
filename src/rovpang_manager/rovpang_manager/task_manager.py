#!/usr/bin/env python3
"""RovPang 배송 작업 관리 노드 task_manager (F-14 배송 상태 관리)

배송 요청(/rovpang/delivery_request)을 받으면 아래 순서로 팀원 2의 액션·서비스를 호출한다.

  DETECTING : 팔 LOOK 자세 -> /rovpang/detections 에서 요청 물품 찾기
  PICKING   : /rovpang/pick
  MOVING    : 팔 CARRY 자세 -> /rovpang/go_to (목적지)
  VERIFYING : 팔 LOOK 자세 -> /rovpang/sign 다수결로 목적지 확인 -> /rovpang/qr 토큰 확인(recipient_auth)
  PLACING   : /rovpang/place
  RETURNING : 팔 HOME 자세 -> /rovpang/go_to START
  DONE
  (실패·취소: 하역 전이면 팔 CARRY 자세, 하역 후면 HOME 자세로 출발지 복귀)

단계가 바뀔 때마다 /rovpang/status 를 발행하고 ~/rovpang_logs/deliveries.csv 에 기록한다.
/rovpang/cancel (std_srvs/Trigger) 로 진행 중인 배송을 취소할 수 있다.

QR 인증은 qr_attempts(기본 2)번 기회를 준다. 모두 틀리면 하역하지 않고
팔을 CARRY 자세로 바꿔 물품을 쥔 채 출발지로 복귀한다.
예외가 생기면 부저를 울리고(alerts.py 의 소리 종류) /rovpang/alert 에 이유를 발행한다.

복귀하지 못하고 멈추면(팔 자세 실패, 복귀 주행 실패) '복귀 필요' 상태가 되어 새 배송 요청을 거절한다.
원인을 해결한 뒤 /rovpang/return_home (std_srvs/Trigger) 을 호출하면 팔 자세 -> 출발지 이동을
다시 시도하고, 성공하면 IDLE 로 돌아가 다시 요청을 받는다. (관리자 프로그램의 '복귀' 버튼용)
"""
import csv
import os
import threading
import time
from collections import Counter

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rovpang_interfaces.action import GoTo, Pick, Place
from rovpang_interfaces.msg import DeliveryRequest, DeliveryStatus, DetectionArray, Sign
from rovpang_interfaces.srv import SetArmPose

from rovpang_manager.alerts import PATTERNS, get_alert, make_text
from rovpang_manager.recipient_auth import RecipientAuth

try:  # 로봇(JetRover)에만 있는 메시지. PC에서는 부저 없이 동작
    from ros_robot_controller_msgs.msg import BuzzerState
except ImportError:
    BuzzerState = None

S = DeliveryStatus


class MissionFailed(Exception):
    def __init__(self, code, message=''):
        super().__init__(message)
        self.code = code
        self.message = message


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        # 파라미터 (값은 초안, 요구사항 정의서와 맞춤)
        self.declare_parameter('confidence_min', 0.8)
        self.declare_parameter('detect_timeout', 10.0)
        self.declare_parameter('sign_frames', 5)
        self.declare_parameter('sign_timeout', 10.0)
        self.declare_parameter('qr_timeout', 30.0)       # QR 기회 전체에 걸친 시간
        self.declare_parameter('qr_attempts', 2)         # QR 인증 기회
        self.declare_parameter('qr_retry_delay', 2.0)    # 틀린 뒤 다음 QR을 받기까지 기다리는 시간
        self.declare_parameter('use_buzzer', True)
        self.declare_parameter('buzzer_freq', 2000)
        self.declare_parameter('action_timeout', 60.0)
        self.declare_parameter('log_dir', os.path.expanduser('~/rovpang_logs'))

        cb = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(DeliveryStatus, '/rovpang/status', 10)
        self.create_subscription(DeliveryRequest, '/rovpang/delivery_request', self.on_request, 10,
                                 callback_group=cb)
        self.create_subscription(DetectionArray, '/rovpang/detections', self.on_detections, 10,
                                 callback_group=cb)
        self.create_subscription(Sign, '/rovpang/sign', self.on_sign, 10, callback_group=cb)
        self.create_subscription(String, '/rovpang/qr', self.on_qr, 10, callback_group=cb)
        self.create_service(Trigger, '/rovpang/cancel', self.on_cancel, callback_group=cb)
        self.create_service(Trigger, '/rovpang/return_home', self.on_return_home, callback_group=cb)

        self.pick_client = ActionClient(self, Pick, '/rovpang/pick', callback_group=cb)
        self.place_client = ActionClient(self, Place, '/rovpang/place', callback_group=cb)
        self.goto_client = ActionClient(self, GoTo, '/rovpang/go_to', callback_group=cb)
        self.arm_client = self.create_client(SetArmPose, '/rovpang/set_arm_pose', callback_group=cb)

        # 예외 알림: 이유는 /rovpang/alert (관리자 프로그램 F-15가 표시), 소리는 로봇 부저
        self.alert_pub = self.create_publisher(String, '/rovpang/alert', 10)
        self.buzzer_pub = None
        if self.get_parameter('use_buzzer').value and BuzzerState is not None:
            self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

        self.lock = threading.Lock()
        self.state = S.STATE_IDLE
        self.request = None
        self.cancel_event = threading.Event()
        self.active_goal = None
        self.auth = RecipientAuth()   # F-10 수령자 인증
        self.need_return = False      # 출발지로 돌아오지 못하고 멈춘 상태
        self.return_pose = SetArmPose.Request.POSE_HOME   # 복구할 때 쓸 팔 자세

        self.last_detections = None
        self.sign_buffer = None      # 수집 중일 때만 list
        self.last_qr = None          # (문자열, 받은 시각)

        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, 'deliveries.csv')
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w', newline='') as f:
                csv.writer(f).writerow(['time', 'delivery_id', 'state', 'error_code', 'message'])

        self.set_state(S.STATE_IDLE, '', '대기 중')
        self.get_logger().info('배송 작업 관리 노드 시작 (로그: {})'.format(self.log_path))

    # ---------- 상태 발행·기록 ----------
    def set_state(self, state, error_code='', message=''):
        self.state = state
        msg = DeliveryStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.delivery_id = self.request.delivery_id if self.request else ''
        msg.state = state
        msg.error_code = error_code
        msg.message = message
        self.status_pub.publish(msg)
        with open(self.log_path, 'a', newline='') as f:
            csv.writer(f).writerow([time.strftime('%Y-%m-%d %H:%M:%S'), msg.delivery_id,
                                    state, error_code, message])
        self.get_logger().info('[{}] {} {}'.format(state, error_code, message).strip())

    def notify(self, code, extra=''):
        """오류 코드에 맞춰 부저를 울리고 이유를 /rovpang/alert 에 발행한다."""
        alert = get_alert(code)
        if alert is None:
            return
        text = make_text(code, extra)
        self.alert_pub.publish(String(data=text))
        self.get_logger().warn('알림: ' + text)
        if self.buzzer_pub is not None:
            repeat, on_time, off_time = PATTERNS[alert[0]]
            try:   # 부저 문제로 배송 흐름이 멈추지 않도록
                msg = BuzzerState()
                msg.freq = int(self.get_parameter('buzzer_freq').value)
                msg.on_time = float(on_time)
                msg.off_time = float(off_time)
                msg.repeat = int(repeat)
                self.buzzer_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn('부저 실패: {}'.format(e))

    # ---------- 구독 콜백 ----------
    def on_request(self, req):
        with self.lock:
            if self.state not in (S.STATE_IDLE, S.STATE_DONE, S.STATE_FAILED, S.STATE_CANCELED):
                self.get_logger().warn('진행 중인 배송이 있어 요청 {} 를 거절합니다'.format(req.delivery_id))
                return
            if self.need_return:
                self.notify('NEED_RETURN', '요청 {} 거절'.format(req.delivery_id))
                return
            if req.destination not in (GoTo.Goal.DEST_A, GoTo.Goal.DEST_B):
                self.get_logger().warn('알 수 없는 목적지: {}'.format(req.destination))
                return
            self.request = req
            self.auth.register(req.delivery_id, req.qr_token)
            self.cancel_event.clear()
        threading.Thread(target=self.run_mission, args=(req,), daemon=True).start()

    def on_detections(self, msg):
        self.last_detections = msg

    def on_sign(self, msg):
        if self.sign_buffer is not None and msg.label:
            self.sign_buffer.append(msg.label)

    def on_qr(self, msg):
        self.last_qr = (msg.data, time.time())

    def on_cancel(self, request, response):
        if self.state in (S.STATE_IDLE, S.STATE_DONE, S.STATE_FAILED, S.STATE_CANCELED):
            response.success = False
            response.message = '진행 중인 배송이 없습니다'
            return response
        self.cancel_event.set()
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()
        response.success = True
        response.message = '취소 요청을 보냈습니다'
        return response

    def on_return_home(self, request, response):
        """복귀 필요 상태에서 관리자가 원인을 해결한 뒤 호출: 팔 자세 -> 출발지 이동 재시도."""
        with self.lock:
            if self.state not in (S.STATE_IDLE, S.STATE_DONE, S.STATE_FAILED, S.STATE_CANCELED):
                response.success = False
                response.message = '진행 중인 작업이 있습니다'
                return response
            if not self.need_return:
                response.success = False
                response.message = '복귀가 필요한 상태가 아닙니다'
                return response
            self.state = S.STATE_RETURNING   # 다른 요청이 끼어들지 않게 먼저 표시
            self.cancel_event.clear()
        threading.Thread(target=self.run_recovery, daemon=True).start()
        response.success = True
        response.message = '출발지 복귀를 시작합니다'
        return response

    def run_recovery(self):
        self.set_state(S.STATE_RETURNING, '', '수동 복귀 시작')
        if self.try_return_home(self.return_pose):
            self.set_state(S.STATE_IDLE, '', '복귀 완료, 다음 요청 대기')
        else:
            self.set_state(S.STATE_FAILED, 'NEED_RETURN', '수동 복귀 실패, 원인 확인 후 다시 시도')

    # ---------- 기다리기 도우미 ----------
    def check_cancel(self):
        if self.cancel_event.is_set():
            raise MissionFailed('CANCELED', '관리자 취소')

    def wait_future(self, future, timeout):
        t0 = time.time()
        while not future.done():
            self.check_cancel()
            if time.time() - t0 > timeout:
                return None
            time.sleep(0.05)
        return future.result()

    def call_action(self, client, goal, name):
        if not client.wait_for_server(timeout_sec=5.0):
            raise MissionFailed('SERVER_UNAVAILABLE', '{} 액션 서버 없음'.format(name))
        handle = self.wait_future(client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            raise MissionFailed('GOAL_REJECTED', '{} 목표 거절'.format(name))
        self.active_goal = handle
        try:
            res = self.wait_future(handle.get_result_async(),
                                   self.get_parameter('action_timeout').value)
        finally:
            self.active_goal = None
        if res is None:
            handle.cancel_goal_async()
            raise MissionFailed('TIMEOUT', '{} 응답 시간 초과'.format(name))
        return res.result

    def set_arm(self, pose):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            raise MissionFailed('SERVER_UNAVAILABLE', 'set_arm_pose 서비스 없음')
        req = SetArmPose.Request()
        req.pose = pose
        res = self.wait_future(self.arm_client.call_async(req), 15.0)
        if res is None or not res.success:
            raise MissionFailed('ARM_POSE_FAILED', '팔 자세 {} 실패'.format(pose))

    # ---------- 단계 ----------
    def detect_item(self, item_class):
        conf_min = self.get_parameter('confidence_min').value
        t0 = time.time()
        while time.time() - t0 < self.get_parameter('detect_timeout').value:
            self.check_cancel()
            msg = self.last_detections
            if msg is not None:
                cands = [d for d in msg.detections
                         if d.class_name == item_class and d.has_position and d.confidence >= conf_min]
                if cands:
                    best = max(cands, key=lambda d: d.confidence)
                    target = PointStamped()
                    target.header = msg.header
                    target.point = best.position
                    return target, best.confidence
            time.sleep(0.1)
        raise MissionFailed('ITEM_NOT_FOUND', '{} 탐지 실패'.format(item_class))

    def verify_sign(self, destination):
        need = self.get_parameter('sign_frames').value
        self.sign_buffer = []
        t0 = time.time()
        try:
            while len(self.sign_buffer) < need:
                self.check_cancel()
                if time.time() - t0 > self.get_parameter('sign_timeout').value:
                    raise MissionFailed('SIGN_NOT_FOUND', '표지판 인식 실패')
                time.sleep(0.1)
            label, count = Counter(self.sign_buffer).most_common(1)[0]
        finally:
            self.sign_buffer = None
        if label != destination:
            raise MissionFailed('WRONG_DESTINATION',
                                '요청 {} / 인식 {} ({}/{})'.format(destination, label, count, need))
        return label

    def verify_qr(self, delivery_id):
        """QR 인증. qr_attempts 번까지 기회를 주고, 틀릴 때마다 새로 읽힌 QR만 다시 확인한다."""
        attempts = max(1, int(self.get_parameter('qr_attempts').value))
        retry_delay = self.get_parameter('qr_retry_delay').value
        messages = {'QR_INVALID': '다른 배송의 QR', 'QR_REUSED': '이미 사용한 QR',
                    'NOT_REGISTERED': '등록되지 않은 배송'}
        t_start = time.time()
        accept_after = t_start   # 이 시각 이후에 읽힌 QR만 확인 (같은 QR을 두 번 세지 않기)
        tried = 0
        while time.time() - t_start < self.get_parameter('qr_timeout').value:
            self.check_cancel()
            if self.last_qr is not None and self.last_qr[1] >= accept_after:
                ok, code = self.auth.verify(delivery_id, self.last_qr[0])
                if ok:
                    return
                if code == 'NOT_REGISTERED':   # 프로그램 쪽 문제라 다시 시도하지 않음
                    raise MissionFailed(code, messages[code])
                tried += 1
                if tried >= attempts:
                    raise MissionFailed(code, '{} ({}/{}회 불일치)'.format(
                        messages.get(code, code), tried, attempts))
                self.set_state(S.STATE_VERIFYING, '', 'QR 불일치 {}/{}회, 다시 확인'.format(tried, attempts))
                self.notify('QR_RETRY', '{}/{}회'.format(tried, attempts))
                accept_after = time.time() + retry_delay
            time.sleep(0.1)
        raise MissionFailed('QR_TIMEOUT', 'QR 인식 시간 초과 ({}/{}회 시도)'.format(tried, attempts))

    # ---------- 전체 흐름 ----------
    def run_mission(self, req):
        left_start = False
        delivered = False
        try:
            self.set_state(S.STATE_DETECTING, '', '물품 {} 탐색'.format(req.item_class))
            self.set_arm(SetArmPose.Request.POSE_LOOK)
            target, conf = self.detect_item(req.item_class)

            self.set_state(S.STATE_PICKING, '', '신뢰도 {:.2f}, 위치 ({:.2f}, {:.2f}, {:.2f})'.format(
                conf, target.point.x, target.point.y, target.point.z))
            goal = Pick.Goal()
            goal.target = target
            goal.item_class = req.item_class
            result = self.call_action(self.pick_client, goal, 'pick')
            if not result.success:
                raise MissionFailed('PICK_FAILED', result.reason)

            self.set_arm(SetArmPose.Request.POSE_CARRY)
            self.set_state(S.STATE_MOVING, '', '목적지 {}'.format(req.destination))
            goal = GoTo.Goal()
            goal.destination = req.destination
            left_start = True
            result = self.call_action(self.goto_client, goal, 'go_to')
            if not result.success:
                raise MissionFailed('NAV_FAILED', result.reason)

            self.set_state(S.STATE_VERIFYING, '', '표지판·QR 확인')
            self.set_arm(SetArmPose.Request.POSE_LOOK)
            self.verify_sign(req.destination)
            self.verify_qr(req.delivery_id)

            self.set_state(S.STATE_PLACING, '', '하역')
            goal = Place.Goal()
            goal.use_default = True
            result = self.call_action(self.place_client, goal, 'place')
            if not result.success:
                raise MissionFailed('PLACE_FAILED', result.reason)
            delivered = True   # 하역까지 끝나면 이후 문제는 '배송 완료'로 기록

            self.set_state(S.STATE_RETURNING, '', '출발지로 복귀')
            self.set_arm(SetArmPose.Request.POSE_HOME)
            goal = GoTo.Goal()
            goal.destination = GoTo.Goal.DEST_START
            result = self.call_action(self.goto_client, goal, 'go_to')
            if not result.success:
                raise MissionFailed('RETURN_FAILED', result.reason)
            left_start = False

            self.set_state(S.STATE_DONE, '', '배송 완료')
        except MissionFailed as e:
            if delivered:
                # 하역은 끝났으므로 배송 자체는 성공. 복귀 중 문제만 오류 코드로 남김
                code = 'RETURN_CANCELED' if e.code == 'CANCELED' else e.code
                self.set_state(S.STATE_DONE, code, '배송 완료, 복귀 중 문제: ' + e.message)
                # 복귀를 한 번 더 시도하고, 그래도 실패하면 try_return_home 이 알림
                if left_start:
                    self.try_return_home(SetArmPose.Request.POSE_HOME)
                    # 실패하면 try_return_home 이 need_return 을 켬
            else:
                if e.code == 'CANCELED':
                    self.set_state(S.STATE_CANCELED, e.code, e.message)
                else:
                    self.set_state(S.STATE_FAILED, e.code, e.message)
                extra = '물품을 가지고 출발지로 돌아갑니다' if left_start else ''
                self.notify(e.code, extra)
                if left_start:
                    # 하역하지 않았으므로 물품을 쥔 채 주행 자세(CARRY)로 복귀
                    self.try_return_home(SetArmPose.Request.POSE_CARRY)
        except Exception as e:  # 예상하지 못한 오류도 기록
            self.set_state(S.STATE_FAILED, 'INTERNAL_ERROR', str(e))
            self.notify('INTERNAL_ERROR', str(e))

    def try_return_home(self, arm_pose):
        """팔 자세를 바꾸고 출발지 복귀 시도. 성공하면 True.
        실패하면 '복귀 필요' 상태로 두고 알림 (/rovpang/return_home 으로 다시 시도)."""
        self.cancel_event.clear()
        self.return_pose = arm_pose
        try:
            self.set_arm(arm_pose)
        except MissionFailed as e:
            # 팔을 뻗은 채 주행하면 부딪힐 수 있으므로 멈추고 관리자에게 알림
            self.get_logger().error('복귀 전 팔 자세 변경 실패, 주행하지 않음: {}'.format(e.message))
            self.need_return = True
            self.notify('ARM_POSE_FAILED', '복귀하지 않고 멈춥니다. 원인 해결 후 복귀 버튼(/rovpang/return_home)')
            return False
        try:
            goal = GoTo.Goal()
            goal.destination = GoTo.Goal.DEST_START
            result = self.call_action(self.goto_client, goal, 'go_to')
            ok = result.success
            msg = '출발지 복귀 {}'.format('성공' if ok else '실패')
        except MissionFailed as e:
            ok = False
            msg = '출발지 복귀 실패: {}'.format(e.message)
        self.get_logger().info(msg)
        self.need_return = not ok
        if not ok:
            self.notify('RETURN_FAILED', msg + '. 원인 해결 후 복귀 버튼(/rovpang/return_home)')
        return ok


def main():
    rclpy.init()
    node = TaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
