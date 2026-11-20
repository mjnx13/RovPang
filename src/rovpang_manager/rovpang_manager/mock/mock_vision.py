#!/usr/bin/env python3
"""팀원 1의 Vision·QR 노드를 흉내 내는 가짜 노드 (카메라 필요 없음)

배송 요청을 받으면 그 물품이 앞에 있는 것처럼 /rovpang/detections 를,
요청 목적지 표지판이 보이는 것처럼 /rovpang/sign 을,
수령자가 올바른 QR을 보여주는 것처럼 /rovpang/qr 을 5Hz로 발행한다.
  wrong_sign: true 면 다른 배송지 표지판을 보냄
  wrong_qr:   true 면 틀린 QR 문자열을 보냄
  wrong_qr_for: 0보다 크면 인증 단계(VERIFYING) 시작 후 이 시간(초) 동안만 틀린 QR을 보내고
                그다음 올바른 QR을 보냄 (QR 두 번째 기회 성공 시험용)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rovpang_interfaces.msg import DeliveryRequest, DeliveryStatus, Detection, DetectionArray, Sign


class MockVision(Node):
    def __init__(self):
        super().__init__('mock_vision')
        self.declare_parameter('wrong_sign', False)
        self.declare_parameter('wrong_qr', False)
        self.declare_parameter('wrong_qr_for', 0.0)
        self.declare_parameter('frame_id', 'depth_cam_color_frame')
        self.req = None
        self.verify_start = None   # 인증 단계가 시작된 시각
        self.create_subscription(DeliveryStatus, '/rovpang/status', self.on_status, 10)
        self.create_subscription(DeliveryRequest, '/rovpang/delivery_request', self.on_request, 10)
        self.det_pub = self.create_publisher(DetectionArray, '/rovpang/detections', 10)
        self.sign_pub = self.create_publisher(Sign, '/rovpang/sign', 10)
        self.qr_pub = self.create_publisher(String, '/rovpang/qr', 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info('가짜 비전 준비 완료')

    def on_request(self, req):
        self.req = req
        self.verify_start = None
        self.get_logger().info('요청 수신: {} -> {}'.format(req.item_class, req.destination))

    def on_status(self, msg):
        if msg.state == DeliveryStatus.STATE_VERIFYING and self.verify_start is None:
            self.verify_start = self.get_clock().now()

    def qr_is_wrong(self):
        if self.get_parameter('wrong_qr').value:
            return True
        wrong_for = self.get_parameter('wrong_qr_for').value
        if wrong_for <= 0.0:
            return False
        if self.verify_start is None:
            return True
        elapsed = (self.get_clock().now() - self.verify_start).nanoseconds / 1e9
        return elapsed < wrong_for

    def tick(self):
        if self.req is None:
            return
        now = self.get_clock().now().to_msg()

        arr = DetectionArray()
        arr.header.stamp = now
        arr.header.frame_id = self.get_parameter('frame_id').value
        d = Detection()
        d.class_name = self.req.item_class
        d.confidence = 0.95
        d.x_min, d.y_min, d.x_max, d.y_max = 300, 200, 340, 240
        d.has_position = True
        d.position.x, d.position.y, d.position.z = 0.02, 0.05, 0.40
        arr.detections.append(d)
        self.det_pub.publish(arr)

        sign = Sign()
        sign.header.stamp = now
        dest = self.req.destination
        if self.get_parameter('wrong_sign').value:
            dest = 'B' if dest == 'A' else 'A'
        sign.label = dest
        sign.confidence = 0.9
        self.sign_pub.publish(sign)

        qr = String()
        qr.data = 'WRONG-TOKEN' if self.qr_is_wrong() else self.req.qr_token
        self.qr_pub.publish(qr)


def main():
    rclpy.init()
    node = MockVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
