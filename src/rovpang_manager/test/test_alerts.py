"""alerts.py 시험: colcon test --packages-select rovpang_manager"""
from rovpang_manager.alerts import ALERTS, PATTERNS, get_alert, make_text

# task_manager 가 실제로 낼 수 있는 오류 코드 (요구사항 정의서 4장 표)
CODES = ['ITEM_NOT_FOUND', 'PICK_FAILED', 'NAV_FAILED', 'SIGN_NOT_FOUND', 'WRONG_DESTINATION',
         'QR_INVALID', 'QR_REUSED', 'QR_TIMEOUT', 'NOT_REGISTERED', 'PLACE_FAILED', 'RETURN_FAILED',
         'TIMEOUT', 'SERVER_UNAVAILABLE', 'GOAL_REJECTED', 'ARM_POSE_FAILED', 'INTERNAL_ERROR',
         'QR_RETRY', 'NEED_RETURN']


def test_every_code_has_alert():
    for code in CODES:
        assert code in ALERTS, code


def test_every_kind_has_pattern():
    for kind, _ in ALERTS.values():
        assert kind in PATTERNS


def test_cancel_is_silent():
    assert get_alert('CANCELED') is None
    assert get_alert('RETURN_CANCELED') is None
    assert make_text('CANCELED') == ''


def test_qr_failure_sounds_differ_from_retry():
    assert get_alert('QR_RETRY')[0] == 'retry'
    assert get_alert('QR_INVALID')[0] == 'auth'


def test_text_has_code_and_extra():
    text = make_text('QR_INVALID', '물품을 가지고 출발지로 돌아갑니다')
    assert text.startswith('[QR_INVALID]')
    assert '돌아갑니다' in text


def test_unknown_code():
    assert get_alert('SOMETHING_NEW')[0] == 'system'
