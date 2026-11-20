"""가짜 로봇 + 가짜 비전 + 배송 작업 관리 노드(task_manager)를 한 번에 실행 (로봇은 움직이지 않음).

  ros2 launch rovpang_manager mock_test.launch.py
  ros2 launch rovpang_manager mock_test.launch.py fail_on:=pick
  ros2 launch rovpang_manager mock_test.launch.py wrong_sign:=true
  ros2 launch rovpang_manager mock_test.launch.py wrong_qr:=true          # QR 2번 모두 틀림 -> CARRY 복귀
  ros2 launch rovpang_manager mock_test.launch.py wrong_qr_for:=3.0      # 첫 QR만 틀림 -> 두 번째에 성공
  ros2 launch rovpang_manager mock_test.launch.py delay:=10.0   # 취소 시험용으로 동작을 느리게
  ros2 launch rovpang_manager mock_test.launch.py wrong_qr:=true fail_on:=arm_away   # 복귀 전 멈춤 -> return_home 시험
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fail_on = LaunchConfiguration('fail_on')
    wrong_sign = LaunchConfiguration('wrong_sign')
    wrong_qr = LaunchConfiguration('wrong_qr')
    wrong_qr_for = LaunchConfiguration('wrong_qr_for')
    delay = LaunchConfiguration('delay')
    return LaunchDescription([
        DeclareLaunchArgument('fail_on', default_value='',
                              description='pick / place / go_to / arm / arm_away 중 하나를 실패시키기'),
        DeclareLaunchArgument('wrong_sign', default_value='false'),
        DeclareLaunchArgument('wrong_qr', default_value='false'),
        DeclareLaunchArgument('wrong_qr_for', default_value='0.0',
                              description='인증 시작 후 몇 초 동안 틀린 QR을 보낼지 (3.0처럼 소수점, 팔 LOOK·표지판 확인 시간보다 길게)'),
        DeclareLaunchArgument('delay', default_value='2.0', description='가짜 로봇 동작 시간(초)'),
        Node(package='rovpang_manager', executable='mock_robot', name='mock_robot',
             output='screen', parameters=[{'fail_on': fail_on, 'delay': delay}]),
        Node(package='rovpang_manager', executable='mock_vision', name='mock_vision',
             output='screen', parameters=[{'wrong_sign': wrong_sign, 'wrong_qr': wrong_qr,
                                                'wrong_qr_for': wrong_qr_for}]),
        Node(package='rovpang_manager', executable='task_manager', name='task_manager',
             output='screen'),
    ])
