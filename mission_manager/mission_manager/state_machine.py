# Mission 2 state machine
#
# This class only TRACKS mission phase and enforces legal transitions. It does
# no I/O and never blocks: the flight actions for each phase live in
# mission_node._run_mission (a worker thread). Keeping blocking work out of
# on_enter_* callbacks matters because transitions are also triggered from
# ROS callbacks, and a blocking callback would stall the executor.
#
# Phase flow:
#   IDLE -> SCAN_REFERENCE_QR -> TAKEOFF -> CORRIDOR_1_TRANSIT -> ARENA_SEARCH
#        -> TARGET_FOUND -> DROPPING -> CORRIDOR_2_TRANSIT -> HOMING -> LANDING
#        -> LANDED
#   ARENA_SEARCH -> CORRIDOR_2_TRANSIT  (search_exhausted: no match, keep payload)
#
# Exits from any active phase:
#   abort    -> RC_MODE  (pilot flipped the auto switch low; pilot flies)
#   failsafe -> ABORTED  (companion-detected failure: LAND / stay on ground)

import time

from statemachine import StateMachine, State


class MissionSM(StateMachine):

    IDLE               = State(initial=True)

    SCAN_REFERENCE_QR  = State()
    TAKEOFF            = State()
    CORRIDOR_1_TRANSIT = State()
    ARENA_SEARCH       = State()
    TARGET_FOUND       = State()
    DROPPING           = State()
    CORRIDOR_2_TRANSIT = State()
    HOMING             = State()
    LANDING            = State()

    LANDED             = State(final=True)
    RC_MODE            = State(final=True)   # pilot has control, mission over
    ABORTED            = State(final=True)   # companion failsafe, mission over

    # forward sequence
    auto_activate         = IDLE.to(SCAN_REFERENCE_QR)
    qr_scanned            = SCAN_REFERENCE_QR.to(TAKEOFF)
    airborne              = TAKEOFF.to(CORRIDOR_1_TRANSIT)
    c1_complete           = CORRIDOR_1_TRANSIT.to(ARENA_SEARCH)
    qr_matched            = ARENA_SEARCH.to(TARGET_FOUND)
    search_exhausted      = ARENA_SEARCH.to(CORRIDOR_2_TRANSIT)
    drop_position_reached = TARGET_FOUND.to(DROPPING)
    drop_complete         = DROPPING.to(CORRIDOR_2_TRANSIT)
    c2_complete           = CORRIDOR_2_TRANSIT.to(HOMING)
    home_reached          = HOMING.to(LANDING)
    touchdown             = LANDING.to(LANDED)

    abort = (
        IDLE.to(RC_MODE) |
        SCAN_REFERENCE_QR.to(RC_MODE) |
        TAKEOFF.to(RC_MODE) |
        CORRIDOR_1_TRANSIT.to(RC_MODE) |
        ARENA_SEARCH.to(RC_MODE) |
        TARGET_FOUND.to(RC_MODE) |
        DROPPING.to(RC_MODE) |
        CORRIDOR_2_TRANSIT.to(RC_MODE) |
        HOMING.to(RC_MODE) |
        LANDING.to(RC_MODE)
    )
    failsafe = (
        IDLE.to(ABORTED) |
        SCAN_REFERENCE_QR.to(ABORTED) |
        TAKEOFF.to(ABORTED) |
        CORRIDOR_1_TRANSIT.to(ABORTED) |
        ARENA_SEARCH.to(ABORTED) |
        TARGET_FOUND.to(ABORTED) |
        DROPPING.to(ABORTED) |
        CORRIDOR_2_TRANSIT.to(ABORTED) |
        HOMING.to(ABORTED) |
        LANDING.to(ABORTED)
    )

    def __init__(self, logger=None):
        self._logger = logger
        self.mission_target_qr = None
        self.drop_confirmed = False
        self._start_time = time.time()
        super().__init__()

    @property
    def state_id(self) -> str:
        return str(self.current_state_value)

    def _log(self, text: str):
        line = f"[{time.time() - self._start_time:.1f}s] {text}"
        if self._logger is not None:
            self._logger.info(line)
        else:
            print(line)

    def after_transition(self, event, source, target):
        self._log(f"[SM] {source.id} --{event}--> {target.id}")

    def on_enter_DROPPING(self):
        self.drop_confirmed = False


if __name__ == "__main__":
    sm = MissionSM()
    sm.auto_activate()
    sm.mission_target_qr = "QR_TARGET_DEMO_ID"
    for ev in ("qr_scanned", "airborne", "c1_complete", "qr_matched",
               "drop_position_reached", "drop_complete", "c2_complete",
               "home_reached", "touchdown"):
        sm.send(ev)
    print(f"Final state : {sm.state_id}")
