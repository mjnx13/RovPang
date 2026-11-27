#!/usr/bin/env python3
"""RovPang 팔 자세 서버 (/rovpang/set_arm_pose, rovpang_interfaces/srv/SetArmPose)

task_manager 가 배송 단계마다 부르는 팔 자세를 '정해 둔 펄스 값'으로 움직인다 (IK 사용 안 함).
  자세 이름 (11/25 팀장·팀원 1 합의): HOME, CARRY, LOOK_DOWN, LOOK_FRONT, SAFE (+ 예전 이름 LOOK)
  자세 값: config/arm_poses.yaml  -  [ID1, ID2, ID3, ID4, ID5, 그리퍼(ID10)] 펄스
           그리퍼가 -1 이면 그리퍼는 움직이지 않음 (물품을 쥔 채 자세만 바꿀 때 놓치지 않게)
  명령: /ros_robot_controller/bus_servo/set_position (controller.launch.py 가 켜져 있어야 함)
  확인: /rovpang/servo_states (rovpang_feedback servo_feedback) 가 있으면 목표 펄스에 도착했는지 확인,
        없으면 이동 시간만큼 기다린 뒤 성공으로 돌려줌

  ros2 run rovpang_arm arm_pose_server --ros-args --params-file <config/arm_poses.yaml>
  ros2 service call /rovpang/set_arm_pose rovpang_interfaces/srv/SetArmPose "{pose: 'HOME'}"
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition

from rovpang_interfaces.srv import SetArmPose

try:   # 서보 피드백 (팀장 rovpang_feedback). 없으면 확인 없이 동작
    from servo_controller_msgs.msg import ServoStateList
except ImportError:
    ServoStateList = None

COMMAND_TOPIC = '/ros_robot_controller/bus_servo/set_position'
FEEDBACK_TOPIC = '/rovpang/servo_states'
GRIPPER_ID = 10

# 기본 자세 (팀장 11/25 작업보고서 값). yaml 로 덮어쓸 수 있음
DEFAULT_POSES = {
    'HOME':       [500, 765, 15, 150, 500, -1],    # 보고서 HOME
    'CARRY':      [500, 700, 15, 215, 500, -1],    # 보고서 CARRY (navigation_pick 마지막 줄)
    'LOOK_FRONT': [500, 750, 0, 375, 500, -1],     # 보고서 DEST_SCAN (수평) - 표지판·QR
    'LOOK_DOWN':  [500, 720, 100, 120, 500, -1],   # 보고서 참고 P1 '물체 추적 초기' - 로봇에서 보며 조정 필요
    'SAFE':       [500, 765, 15, 150, 500, -1],    # 우선 HOME 과 같게
}


class ArmPoseServer(Node):
    def __init__(self):
        super().__init__('arm_pose_server')
        self.declare_parameter('duration', 2.0)          # 자세 바꾸는 시간(초). 처음엔 천천히
        self.declare_parameter('tolerance', 15)          # 목표 펄스와 이만큼 이내면 도착
        self.declare_parameter('settle_timeout', 3.0)    # 이동 시간 뒤 도착을 기다리는 최대 시간(초)
        self.poses = {}
        for name, default in DEFAULT_POSES.items():
            self.declare_parameter('poses.' + name, default)
        self.declare_parameter('poses.LOOK', DEFAULT_POSES['LOOK_DOWN'])   # 예전 이름 호환
        for name in list(DEFAULT_POSES) + ['LOOK']:
            self.poses[name] = [int(v) for v in self.get_parameter('poses.' + name).value]

        cb = ReentrantCallbackGroup()
        self.pub = self.create_publisher(ServosPosition, COMMAND_TOPIC, 10)
        self.lock = threading.Lock()
        self.feedback = None          # {id: 펄스}
        self.feedback_time = 0.0
        if ServoStateList is not None:
            self.create_subscription(ServoStateList, FEEDBACK_TOPIC, self.on_feedback, 10, callback_group=cb)
        self.create_service(SetArmPose, '/rovpang/set_arm_pose', self.on_request, callback_group=cb)
        self.get_logger().info('팔 자세 서버 준비 (/rovpang/set_arm_pose): {}'.format(', '.join(sorted(self.poses))))

    def on_feedback(self, msg):
        with self.lock:
            self.feedback = {int(s.id): int(s.position) for s in msg.servo_state}
            self.feedback_time = time.time()

    def check_pose(self, values):
        """값이 6개이고 관절은 0~1000, 그리퍼는 -1 또는 0~1000 인지"""
        if len(values) != 6:
            return '값이 6개(ID1~5, 그리퍼)가 아님: {}'.format(values)
        if any(not 0 <= v <= 1000 for v in values[:5]):
            return '관절 펄스가 0~1000 밖: {}'.format(values[:5])
        if values[5] != -1 and not 0 <= values[5] <= 1000:
            return '그리퍼 펄스가 -1 또는 0~1000 밖: {}'.format(values[5])
        return ''

    def send(self, values, duration):
        msg = ServosPosition()
        msg.duration = float(duration)
        targets = list(enumerate(values[:5], start=1))
        if values[5] != -1:
            targets.append((GRIPPER_ID, values[5]))
        for sid, pulse in targets:
            sp = ServoPosition()
            sp.id = int(sid)
            sp.position = int(pulse)
            msg.position.append(sp)
        self.pub.publish(msg)

    def wait_arrival(self, values, start):
        """피드백으로 도착 확인. 피드백이 없으면 None (확인 못 함)"""
        tol = int(self.get_parameter('tolerance').value)
        end = time.time() + float(self.get_parameter('settle_timeout').value)
        worst = None
        while time.time() < end:
            with self.lock:
                fb, ft = self.feedback, self.feedback_time
            if fb is None or ft < start:
                time.sleep(0.1)
                continue
            diffs = [abs(fb.get(i, -9999) - v) for i, v in enumerate(values[:5], start=1)]
            worst = max(diffs)
            if worst <= tol:
                return True, worst
            time.sleep(0.1)
        return (None, None) if worst is None else (False, worst)

    def on_request(self, request, response):
        name = request.pose
        values = self.poses.get(name)
        if values is None:
            response.success, response.message = False, '모르는 자세: {} (가능: {})'.format(
                name, ', '.join(sorted(self.poses)))
            self.get_logger().warn(response.message)
            return response
        bad = self.check_pose(values)
        if bad:
            response.success, response.message = False, '{} 자세 값 오류 - {}'.format(name, bad)
            self.get_logger().error(response.message)
            return response
        duration = float(self.get_parameter('duration').value)
        self.get_logger().info('[{}] {} ({}초)'.format(name, values, duration))
        start = time.time()
        self.send(values, duration)
        time.sleep(duration)
        arrived, worst = self.wait_arrival(values, start)
        if arrived is None:
            response.success, response.message = True, '{} 이동 (피드백 없음, 도착 확인 안 함)'.format(name)
        elif arrived:
            response.success, response.message = True, '{} 도착 (최대 오차 {}펄스)'.format(name, worst)
        else:
            response.success, response.message = False, '{} 도착 못 함 (최대 오차 {}펄스, 걸림·전원 확인)'.format(name, worst)
        (self.get_logger().info if response.success else self.get_logger().error)(response.message)
        return response


def main():
    rclpy.init()
    node = ArmPoseServer()
    executor = MultiThreadedExecutor(num_threads=2)
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
