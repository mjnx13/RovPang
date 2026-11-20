"""예외 알림 표 (F-16): 오류 코드 -> 소리 종류 + 사용자 안내 문장

ROS 없이 동작하는 순수 Python 모듈이라 pytest로 바로 시험할 수 있다.
task_manager 가 이 표를 보고 부저를 울리고 /rovpang/alert 에 안내 문장을 발행한다.

소리 종류 (부저 패턴: 반복 횟수, 켜짐 초, 꺼짐 초)
  retry  : 짧게 1번  - 다시 시도해 달라는 안내 (QR 다시 보여 주세요)
  auth   : 짧게 2번  - 배송지·수령자 인증 실패 (표지판, QR)
  item   : 짧게 3번  - 물품 탐지·집기·하역 실패
  system : 길게 1번  - 주행·복귀·로봇 동작 오류
취소(CANCELED, RETURN_CANCELED)는 관리자가 직접 한 일이라 소리를 내지 않는다.
"""

PATTERNS = {
    'retry': (1, 0.15, 0.10),
    'auth': (2, 0.15, 0.15),
    'item': (3, 0.15, 0.15),
    'system': (1, 1.00, 0.10),
}

ALERTS = {
    # 재시도 안내
    'QR_RETRY': ('retry', 'QR이 맞지 않습니다. QR을 다시 보여 주세요'),
    # 인증 실패
    'SIGN_NOT_FOUND': ('auth', '배송지 표지판을 찾지 못했습니다'),
    'WRONG_DESTINATION': ('auth', '요청한 배송지가 아닙니다'),
    'QR_INVALID': ('auth', '이 배송의 QR이 아닙니다'),
    'QR_REUSED': ('auth', '이미 사용한 QR입니다'),
    'QR_TIMEOUT': ('auth', 'QR이 확인되지 않았습니다'),
    # 물품 처리 실패
    'ITEM_NOT_FOUND': ('item', '요청한 물품을 찾지 못했습니다'),
    'PICK_FAILED': ('item', '물품을 집지 못했습니다'),
    'PLACE_FAILED': ('item', '물품을 내려놓지 못했습니다'),
    # 주행·시스템 오류
    'NAV_FAILED': ('system', '배송지로 이동하지 못했습니다'),
    'RETURN_FAILED': ('system', '출발지로 돌아가지 못했습니다'),
    'NOT_REGISTERED': ('system', '등록되지 않은 배송입니다'),
    'TIMEOUT': ('system', '로봇 동작 시간이 초과되었습니다'),
    'SERVER_UNAVAILABLE': ('system', '로봇 동작 프로그램이 실행 중이 아닙니다'),
    'GOAL_REJECTED': ('system', '로봇이 동작 요청을 거절했습니다'),
    'ARM_POSE_FAILED': ('system', '로봇팔 자세를 바꾸지 못했습니다'),
    'INTERNAL_ERROR': ('system', '프로그램 내부 오류가 발생했습니다'),
    'NEED_RETURN': ('system', '로봇이 출발지에 없어 요청을 받을 수 없습니다. 먼저 복귀시켜 주세요'),
}

SILENT = {'CANCELED', 'RETURN_CANCELED'}


def get_alert(code):
    """오류 코드에 맞는 (소리 종류, 안내 문장). 소리를 내지 않는 코드는 None."""
    if code in SILENT:
        return None
    return ALERTS.get(code, ('system', '알 수 없는 오류: {}'.format(code)))


def make_text(code, extra=''):
    """관리자 화면·로그에 보일 한 줄: '[코드] 안내 문장. 추가 설명'"""
    alert = get_alert(code)
    if alert is None:
        return ''
    text = '[{}] {}'.format(code, alert[1])
    if extra:
        text += '. ' + extra
    return text
