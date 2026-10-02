import types

from mission_manager.state_machine import MissionSM


class _DummyNode:
    def get_logger(self):
        return types.SimpleNamespace(info=lambda *args, **kwargs: None, warn=lambda *args, **kwargs: None)


def test_areana_search_uses_configured_speed(monkeypatch):
    sm = MissionSM(ros_node=_DummyNode(), cfg=types.SimpleNamespace(
        service_timeout_s=10.0,
        lawnmower_timeout_s=600.0,
        arena_speed_ms=3.0,
    ))

    calls = {}

    def fake_call_start_qr_scan(node, target_qr_id, timeout):
        calls['start_qr_scan'] = (target_qr_id, timeout)
        return True

    def fake_run_lawnmower(node, speed, timeout=None):
        calls['run_lawnmower'] = (speed, timeout)
        return True

    class DummyThread:
        def __init__(self, target, args, kwargs=None, daemon=None):
            self.target = target
            self.args = args
            self.kwargs = kwargs or {}

        def start(self):
            self.target(*self.args, **self.kwargs)

    monkeypatch.setattr("mission_manager.srv_helpers.call_start_qr_scan", fake_call_start_qr_scan)
    monkeypatch.setattr("mission_manager.srv_helpers.call_run_lawnmower", fake_run_lawnmower)
    monkeypatch.setattr("threading.Thread", DummyThread)

    sm.on_enter_ARENA_SEARCH(None, None)

    assert calls['start_qr_scan'][0] is None
    assert calls['run_lawnmower'][0] == 3.0
    assert calls['run_lawnmower'][1] == 600.0
