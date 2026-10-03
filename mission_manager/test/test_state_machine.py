import pytest
from statemachine.exceptions import TransitionNotAllowed

from mission_manager.state_machine import MissionSM

FORWARD = ["auto_activate", "qr_scanned", "airborne", "c1_complete", "qr_matched",
           "drop_position_reached", "drop_complete", "c2_complete",
           "home_reached", "touchdown"]


def _run(sm, events):
    for ev in events:
        sm.send(ev)


def test_full_mission_ends_landed():
    sm = MissionSM()
    _run(sm, FORWARD)
    assert sm.state_id == "LANDED"


def test_no_qr_match_skips_drop_and_continues():
    sm = MissionSM()
    _run(sm, ["auto_activate", "qr_scanned", "airborne", "c1_complete",
              "search_exhausted"])
    assert sm.state_id == "CORRIDOR_2_TRANSIT"
    _run(sm, ["c2_complete", "home_reached", "touchdown"])
    assert sm.state_id == "LANDED"
    assert sm.drop_confirmed is False


def test_landing_is_a_separate_phase_from_homing():
    # HOMING must not jump straight to LANDED (that disarmed in the air).
    sm = MissionSM()
    _run(sm, FORWARD[:-2])
    assert sm.state_id == "HOMING"
    with pytest.raises(TransitionNotAllowed):
        sm.touchdown()


@pytest.mark.parametrize("n_events", range(1, len(FORWARD)))
def test_pilot_abort_from_every_active_phase(n_events):
    sm = MissionSM()
    _run(sm, FORWARD[:n_events])
    sm.abort()
    assert sm.state_id == "RC_MODE"


@pytest.mark.parametrize("n_events", range(0, len(FORWARD)))
def test_failsafe_from_every_active_phase(n_events):
    sm = MissionSM()
    _run(sm, FORWARD[:n_events])
    sm.failsafe()
    assert sm.state_id == "ABORTED"


def test_terminal_states_accept_nothing():
    sm = MissionSM()
    _run(sm, FORWARD)
    with pytest.raises(TransitionNotAllowed):
        sm.abort()
