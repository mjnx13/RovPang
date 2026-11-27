"""RovPang 통합 실행 (시연용)

먼저 로봇 본체에서: sudo systemctl stop start_app_node.service   (제조사 앱이 CPU 2개를 씀)

  ros2 launch rovpang_bringup rovpang.launch.py                       # 로봇 동작(pick·place·go_to·팔)은 가짜, 나머지 실제
  ros2 launch rovpang_bringup rovpang.launch.py mock:=pick,place,go_to      # 팔 자세만 실제 (arm_pose_server 자동 실행)
  ros2 launch rovpang_bringup rovpang.launch.py mock:=pick,place,arm nav:=true   # 주행만 실제(팀장 nav_bringup)
  ros2 launch rovpang_bringup rovpang.launch.py mock:=none nav:=true  # 모두 실제 (팀장 노드가 모두 준비됐을 때)

켜지는 것 (순서: 카메라 -> 5초 뒤 비전 -> 배송 작업 관리)
  camera    : peripherals depth_camera.launch.py depth_registration:=true
  controller: controller controller.launch.py (비프음, 팔 서보)
  vision    : rovpang_vision vision.launch.py (물품·표지판·QR)
  manager   : task_manager (+ mock 에 적은 기능만 mock_robot 이 흉내)
  arm       : mock 에 arm 이 없으면 rovpang_arm arm_pose_server (config/arm_poses.yaml 펄스 값)
  rosbridge : rosbridge_websocket (관리자 프로그램, 포트 9090)
  nav       : rovpang_navigation nav_bringup.launch.py (지도·Nav2·goto_server)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node

FUNCS = ['pick', 'place', 'go_to', 'arm']


def include(pkg, rel, args=None):
    path = os.path.join(get_package_share_directory(pkg), rel)
    src = PythonLaunchDescriptionSource(path) if path.endswith('.py') else AnyLaunchDescriptionSource(path)
    return IncludeLaunchDescription(src, launch_arguments=(args or {}).items())


def on(ctx, name):
    return ctx.launch_configurations[name].lower() in ('true', '1', 'yes')


def setup(ctx):
    acts = []
    if on(ctx, 'camera'):
        acts.append(include('peripherals', 'launch/depth_camera.launch.py', {'depth_registration': 'true'}))
    if on(ctx, 'controller'):
        acts.append(include('controller', 'launch/controller.launch.py'))
    if on(ctx, 'rosbridge'):
        acts.append(include('rosbridge_server', 'launch/rosbridge_websocket_launch.xml'))
    if on(ctx, 'nav'):
        acts.append(include('rovpang_navigation', 'launch/nav_bringup.launch.py'))

    later = []
    if on(ctx, 'vision'):
        later.append(include('rovpang_vision', 'launch/vision.launch.py'))

    mock = ctx.launch_configurations['mock'].replace(' ', '')
    serve = FUNCS if mock == 'all' else [] if mock in ('', 'none') else mock.split(',')
    bad = [f for f in serve if f not in FUNCS]
    if bad:
        raise RuntimeError('mock 에는 {} 중에서 고르세요: {}'.format(','.join(FUNCS), bad))
    # 팔 자세를 가짜로 하지 않으면 실제 팔 자세 서버(정해 둔 펄스 값)를 켬
    if 'arm' not in serve and on(ctx, 'arm_server'):
        poses = os.path.join(get_package_share_directory('rovpang_arm'), 'config', 'arm_poses.yaml')
        acts.append(Node(package='rovpang_arm', executable='arm_pose_server', name='arm_pose_server',
                         output='screen', parameters=[poses]))
    if serve:
        later.append(Node(package='rovpang_manager', executable='mock_robot', name='mock_robot', output='screen',
                          parameters=[{'serve': serve}]))
    later.append(Node(package='rovpang_manager', executable='task_manager', name='task_manager',
                      output='screen'))
    # 카메라가 완전히 켜진 뒤 비전·배송 관리 시작 (11/25: 순서가 꼬이면 영상이 안 들어온 적 있음)
    acts.append(TimerAction(period=float(ctx.launch_configurations['start_delay']), actions=later))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('controller', default_value='true', description='비프음·팔 서보 (팀장이 따로 켰으면 false)'),
        DeclareLaunchArgument('vision', default_value='true'),
        DeclareLaunchArgument('rosbridge', default_value='true'),
        DeclareLaunchArgument('nav', default_value='false', description='true: 팀장 nav_bringup (지도 필요)'),
        DeclareLaunchArgument('mock', default_value='all',
                              description='가짜로 대신할 기능: all / none / pick,place,go_to,arm 중 쉼표로'),
        DeclareLaunchArgument('arm_server', default_value='true',
                              description='mock 에 arm 이 없을 때 rovpang_arm arm_pose_server 를 켤지'),
        DeclareLaunchArgument('start_delay', default_value='5.0', description='카메라 켠 뒤 비전·관리 노드 시작까지(초)'),
        OpaqueFunction(function=setup),
    ])
