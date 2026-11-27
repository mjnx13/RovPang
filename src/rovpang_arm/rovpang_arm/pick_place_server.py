#!/usr/bin/env python3
"""RovPang 고정 위치 집기·하역 서버 (/rovpang/pick, /rovpang/place) - 시연용

팀장 pick_server(IK + TF)가 준비되기 전, '정해 둔 자리'에 놓인 물품을 정해 둔 펄스 동작으로 집고 내려놓는다.
동작 값은 Hiwonder navigation_pick 동작(팀장 11/25 보고서)을 바탕으로 한다.
  pick : 그리퍼 열기 -> 팔 내리기(grasp_pose) -> 그리퍼 닫기 -> 그리퍼 위치로 집었는지 확인 -> 들기(lift_pose)
         카메라가 본 물품 위치가 expected_xyz 에서 tolerance 이상 떨어져 있으면 움직이지 않고 OUT_OF_RANGE
  place: 팔 내리기(place_pose) -> 그리퍼 열기 -> 들기(retract_pose)
  - pick 은 한 번만 시도 (재시도는 task_manager 가 물품을 다시 찾아서, 11/25 합의)
  - dry_run:=true 면 동작을 로그로만 보여 주고 움직이지 않음 (처음 시험용)
  config/pick_place.yaml 에 값이 있음
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition

from rovpang_interfaces.action import Pick, Place

try:
    from servo_controller_msgs.msg import ServoStateList
except ImportError:
    ServoStateList = None

COMMAND_TOPIC = '/ros_robot_controller/bus_servo/set_position'
FEEDBACK_TOPIC = '/rovpang/servo_states'
GRIPPER_ID = 10
OUT_OF_RANGE = getattr(Pick.Result, 'REASON_OUT_OF_RANGE', 'OUT_OF_RANGE')


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        p = self.declare_parameter
        p('dry_run', False)
        p('grasp_pose', [500, 190, 300, 300, 500])     # 물품 높이까지 내린 자세 (navigation_pick 1행)
        p('lift_pose', [500, 700, 15, 215, 500])       # 집은 뒤 들어 올린 자세 (= CARRY)
        p('place_pose', [500, 190, 300, 300, 500])     # 내려놓는 자세
        p('retract_pose', [500, 765, 15, 150, 500])    # 놓은 뒤 빼는 자세 (= HOME)
        p('gripper_open', 200)
        p('gripper_close', 560)
        p('grasped_max', 545)          # 닫은 뒤 그리퍼 위치가 이 값보다 작으면 물품에 막힌 것 = 집음
        p('move_time', 2.0)            # 팔 이동 시간(초)
        p('grip_time', 1.0)            # 그리퍼 여닫는 시간(초)
        p('expected_xyz', [0.0, 0.0, 0.0])   # LOOK_DOWN 에서 '집는 자리'에 놓인 물품의 카메라 좌표(m). 0,0,0 이면 검사 안 함
        p('tolerance', 0.04)           # 집는 자리에서 이만큼(m) 넘게 벗어나면 OUT_OF_RANGE

        cb = ReentrantCallbackGroup()
        self.pub = self.create_publisher(ServosPosition, COMMAND_TOPIC, 10)
        self.lock = threading.Lock()
        self.gripper_pos = None
        self.gripper_time = 0.0
        if ServoStateList is not None:
            self.create_subscription(ServoStateList, FEEDBACK_TOPIC, self.on_feedback, 10, callback_group=cb)
        common = dict(callback_group=cb, goal_callback=lambda g: GoalResponse.ACCEPT,
                      cancel_callback=lambda g: CancelResponse.ACCEPT)
        ActionServer(self, Pick, '/rovpang/pick', self.exec_pick, **common)
        ActionServer(self, Place, '/rovpang/place', self.exec_place, **common)
        self.get_logger().info('고정 위치 집기·하역 서버 준비 (/rovpang/pick, /rovpang/place){}'.format(
            ' - dry_run: 움직이지 않음' if self.get_parameter('dry_run').value else ''))

    def on_feedback(self, msg):
        for s in msg.servo_state:
            if int(s.id) == GRIPPER_ID:
                with self.lock:
                    self.gripper_pos = int(s.position)
                    self.gripper_time = time.time()

    def param(self, name):
        return self.get_parameter(name).value

    def move(self, label, joints=None, gripper=None, duration=2.0):
        """joints(5개)·gripper 펄스로 이동하고 duration 동안 기다림"""
        targets = []
        if joints is not None:
            targets += list(enumerate([int(v) for v in joints], start=1))
        if gripper is not None:
            targets.append((GRIPPER_ID, int(gripper)))
        if any(not 0 <= v <= 1000 for _, v in targets):
            raise ValueError('펄스 범위(0~1000) 밖: {}'.format(targets))
        self.get_logger().info('  {} -> {} ({:.1f}초){}'.format(
            label, targets, duration, ' [dry_run]' if self.param('dry_run') else ''))
        if not self.param('dry_run'):
            msg = ServosPosition()
            msg.duration = float(duration)
            for sid, pulse in targets:
                sp = ServoPosition()
                sp.id, sp.position = int(sid), int(pulse)
                msg.position.append(sp)
            self.pub.publish(msg)
        time.sleep(duration + 0.2)

    def feedback(self, goal_handle, action, stage):
        fb = action.Feedback()
        fb.stage = stage
        goal_handle.publish_feedback(fb)

    def exec_pick(self, goal_handle):
        g = goal_handle.request
        result = Pick.Result()
        result.attempts = 1
        t = g.target.point
        self.get_logger().info('[pick] {} 카메라 좌표 ({:.3f}, {:.3f}, {:.3f}) frame={}'.format(
            g.item_class, t.x, t.y, t.z, g.target.header.frame_id))
        ex = list(self.param('expected_xyz'))
        if any(ex):
            dist = math.dist((t.x, t.y, t.z), ex)
            if dist > self.param('tolerance'):
                result.success = False
                result.reason = '{}: 집는 자리에서 {:.1f}cm 벗어남 (허용 {:.1f}cm)'.format(
                    OUT_OF_RANGE, dist * 100, self.param('tolerance') * 100)
                self.get_logger().warn(result.reason)
                goal_handle.abort()
                return result
        try:
            self.feedback(goal_handle, Pick, 'APPROACH')
            self.move('그리퍼 열기', gripper=self.param('gripper_open'), duration=self.param('grip_time'))
            self.move('팔 내리기', joints=self.param('grasp_pose'), duration=self.param('move_time'))
            self.feedback(goal_handle, Pick, 'GRASP')
            self.move('그리퍼 닫기', gripper=self.param('gripper_close'), duration=self.param('grip_time'))
            time.sleep(0.3)
            with self.lock:
                pos, ptime = self.gripper_pos, self.gripper_time
            self.feedback(goal_handle, Pick, 'LIFT')
            self.move('들기', joints=self.param('lift_pose'), duration=self.param('move_time'))
        except ValueError as e:
            result.success, result.reason = False, 'PICK_FAILED: ' + str(e)
            goal_handle.abort()
            return result
        if self.param('dry_run'):
            result.success, result.reason = True, ''
        elif pos is None or time.time() - ptime > 3.0:
            # 피드백이 없으면 집었는지 모름 -> 성공으로 두되 알림 (2차 확인은 카메라 재탐지 몫)
            self.get_logger().warn('그리퍼 피드백 없음 - 집었는지 확인 못 함 (servo_feedback 실행 확인)')
            result.success, result.reason = True, ''
        elif pos < self.param('grasped_max'):
            self.get_logger().info('집음 (그리퍼 위치 {} < {})'.format(pos, self.param('grasped_max')))
            result.success, result.reason = True, ''
        else:
            result.success = False
            result.reason = 'PICK_FAILED: 빈 손 (그리퍼 위치 {} >= {})'.format(pos, self.param('grasped_max'))
            self.get_logger().warn(result.reason)
        (goal_handle.succeed if result.success else goal_handle.abort)()
        return result

    def exec_place(self, goal_handle):
        result = Place.Result()
        self.get_logger().info('[place] use_default={}'.format(goal_handle.request.use_default))
        try:
            self.feedback(goal_handle, Place, 'ALIGN')
            self.move('팔 내리기', joints=self.param('place_pose'), duration=self.param('move_time'))
            self.feedback(goal_handle, Place, 'RELEASE')
            self.move('그리퍼 열기', gripper=self.param('gripper_open'), duration=self.param('grip_time'))
            self.feedback(goal_handle, Place, 'RETRACT')
            self.move('팔 빼기', joints=self.param('retract_pose'), duration=self.param('move_time'))
        except ValueError as e:
            result.success, result.reason = False, 'PLACE_FAILED: ' + str(e)
            goal_handle.abort()
            return result
        result.success, result.reason = True, ''
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = PickPlaceServer()
    executor = MultiThreadedExecutor(num_threads=3)
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
