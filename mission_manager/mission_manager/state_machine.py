#Mission 2

from statemachine import StateMachine, State
import time


class MissionSM(StateMachine):

    IDLE              = State(initial=True)
    RC_MODE           = State()

    SCAN_REFERENCE_QR = State()   
    TAKEOFF           = State()
    CORRIDOR_1_TRANSIT= State()   
    ARENA_SEARCH      = State()   
    TARGET_FOUND      = State()   
    DROPPING          = State()   
    CORRIDOR_2_TRANSIT= State()   
    HOMING            = State()  
    LANDED            = State(final=True)

    # transitions
    # RC to Auto
    rc_activate       = IDLE.to(RC_MODE)
    rc_to_auto        = RC_MODE.to(SCAN_REFERENCE_QR)
    auto_activate     = IDLE.to(SCAN_REFERENCE_QR)

    #forward seq
    qr_scanned        = SCAN_REFERENCE_QR.to(TAKEOFF)
    airborne          = TAKEOFF.to(CORRIDOR_1_TRANSIT)
    c1_complete       = CORRIDOR_1_TRANSIT.to(ARENA_SEARCH)
    qr_matched        = ARENA_SEARCH.to(TARGET_FOUND)
    drop_position_reached = TARGET_FOUND.to(DROPPING)
    drop_complete     = DROPPING.to(CORRIDOR_2_TRANSIT)
    c2_complete       = CORRIDOR_2_TRANSIT.to(HOMING)
    touchdown         = HOMING.to(LANDED)

    #abort to rc mode
    abort             = (
        SCAN_REFERENCE_QR.to(RC_MODE) |
        TAKEOFF.to(RC_MODE) |
        CORRIDOR_1_TRANSIT.to(RC_MODE) |
        ARENA_SEARCH.to(RC_MODE) |
        TARGET_FOUND.to(RC_MODE) |
        DROPPING.to(RC_MODE) |
        CORRIDOR_2_TRANSIT.to(RC_MODE) |
        HOMING.to(RC_MODE)
    )

    # state data
    def __init__(self, ros_node=None, cfg=None):
        self.ros_node           = ros_node  
        self.cfg                = cfg
        self.mission_target_qr  = None 
        self.drop_confirmed     = False
        self.obstacle_warning   = False
        self._start_time        = time.time()
        super().__init__()

    # ── Helpers ──────────────────────────────────────────────────
    def _c(self, attr, default=0.0):
        """Read a config attribute, falling back to *default*."""
        if self.cfg is not None:
            return getattr(self.cfg, attr, default)
        return default

    def _elapsed(self):
        return f"{time.time() - self._start_time:.1f}s"

    # ── State enter callbacks ────────────────────────────────────

    def on_enter_IDLE(self, event, state):
        print(f"[{self._elapsed()}] [IDLE] System ready. Awaiting arm command.")

    def on_enter_RC_MODE(self, event, state):
        print(f"[{self._elapsed()}] [RC_MODE] Pilot has control. Geofence active. OAK-D warnings live.")

    def on_enter_SCAN_REFERENCE_QR(self, event, state):
        print(f"[{self._elapsed()}] [SCAN_REFERENCE_QR] Point camera at takeoff zone QR...")
        fallback_id = self._c("qr_fallback_id", "QR_TARGET_DEMO_ID")
        if self.ros_node:
            from mission_manager.srv_helpers import call_scan_reference_qr
            decoded = call_scan_reference_qr(
                self.ros_node,
                timeout=self._c("service_timeout_s", 10.0),
            )
            if decoded:
                self.mission_target_qr = decoded
            else:
                self.mission_target_qr = fallback_id
                print(f"[{self._elapsed()}]   QR service unavailable, using fallback ID")
        else:
            self.mission_target_qr = fallback_id   # demo mode
        print(f"[{self._elapsed()}]   Target locked: {self.mission_target_qr}")

    def on_enter_TAKEOFF(self, event, state):
        cruise_alt = self._c("cruise_altitude_m", 5.0)
        print(f"[{self._elapsed()}] [TAKEOFF] Arming and climbing to {cruise_alt} m...")
        if self.ros_node:
            from mission_manager.mavros_utils import (
                set_mode, arm, takeoff, wait_for_altitude,
            )
            svc_timeout = self._c("service_timeout_s", 10.0)

            set_mode(self.ros_node, "GUIDED", timeout=svc_timeout)
            arm(self.ros_node, timeout=svc_timeout)
            takeoff(self.ros_node, altitude_m=cruise_alt, timeout=svc_timeout)
            wait_for_altitude(
                self.ros_node, target_alt=cruise_alt,
                tolerance=self._c("altitude_tolerance_m", 0.5),
                timeout=self._c("altitude_timeout_s", 30.0),
            )
        else:
            pass

    def on_enter_CORRIDOR_1_TRANSIT(self, event, state):
        print(f"[{self._elapsed()}] [CORRIDOR_1_TRANSIT] Entering clean corridor")
        if self.ros_node:
            from mission_manager.srv_helpers import call_fly_corridor
            speed = self._c("corridor_speed_ms", 2.0)
            call_fly_corridor(
                self.ros_node, speed=speed, obstacles=False,
                timeout=self._c("service_timeout_s", 10.0),
            )

    def on_enter_ARENA_SEARCH(self, event, state):
        print(f"[{self._elapsed()}] [ARENA_SEARCH] Starting BCD lawnmower search")
        if self.ros_node:
            from mission_manager.srv_helpers import call_run_lawnmower, call_start_qr_scan
            import threading
            timeout = self._c("service_timeout_s", 10.0)
            speed = self._c("arena_speed_ms", 3.0)
            # Start QR scan BEFORE lawnmower so detections work immediately
            call_start_qr_scan(self.ros_node, target_qr_id=self.mission_target_qr,
                               timeout=timeout)
            # Lawnmower now blocks until complete — run in background thread
            # so the QR detection callback can fire and trigger qr_matched()
            self._lawnmower_thread = threading.Thread(
                target=call_run_lawnmower,
                args=(self.ros_node, speed),
                kwargs={"timeout": self._c("lawnmower_timeout_s", 600.0)},
                daemon=True,
            )
            self._lawnmower_thread.start()

    def on_enter_TARGET_FOUND(self, event, state):
        print(f"[{self._elapsed()}] [TARGET_FOUND] QR matched, pausing lawnmower.")
        if self.ros_node:
            from mission_manager.srv_helpers import call_stop_lawnmower
            call_stop_lawnmower(
                self.ros_node,
                timeout=self._c("service_timeout_s", 10.0),
            )

    def on_enter_DROPPING(self, event, state):
        print(f"[{self._elapsed()}] [DROPPING] Releasing payload")
        if self.ros_node:
            from mission_manager.srv_helpers import call_payload_release
            timeout = self._c("service_timeout_s", 10.0)
            call_payload_release(self.ros_node, timeout=timeout)
            print(f"[{self._elapsed()}]   Payload released")
        else:
            print(f"[{self._elapsed()}]   (demo mode) Simulating payload drop sequence")

        self.drop_confirmed = True
   

    def on_enter_CORRIDOR_2_TRANSIT(self, event, state):
        print(f"[{self._elapsed()}] [CORRIDOR_2_TRANSIT] Return corridor — static obstacles. OAK-D active.")
        if self.ros_node:
            from mission_manager.srv_helpers import call_fly_corridor
            speed = self._c("corridor_obs_speed_ms", 1.0)
            call_fly_corridor(
                self.ros_node, speed=speed, obstacles=True,
                timeout=self._c("service_timeout_s", 10.0),
            )
         

    def on_enter_HOMING(self, event, state):
        print(f"[{self._elapsed()}] [HOMING] Returning to launch (RTL)")
        if self.ros_node:
            from mission_manager.mavros_utils import set_mode
            set_mode(self.ros_node, "RTL",
                     timeout=self._c("service_timeout_s", 10.0))

    def on_enter_LANDED(self, event, state):
        print(f"[{self._elapsed()}] [LANDED] Mission complete. Disarming")
        if self.ros_node:
            from mission_manager.mavros_utils import disarm
            disarm(self.ros_node, timeout=self._c("service_timeout_s", 10.0))
            print(f"[{self._elapsed()}]   Vehicle disarmed")



if __name__ == "__main__":
    print(" " * 60)
    print("  Mission 2: state machine demo ")
    print(" " * 60)

    sm = MissionSM()

    # sim full auto seq
    sm.auto_activate()
    sm.qr_scanned()
    sm.airborne()
    sm.c1_complete()

    #simulate QR match mid-search
    sm.mission_target_qr = "QR_TARGET_DEMO_ID"
    sm.qr_matched()

    sm.drop_position_reached()
    sm.drop_complete()
    sm.c2_complete()
    sm.touchdown()

    print()
    print(f"Final state : {sm.current_state.id}")
    print(f"Drop confirmed : {sm.drop_confirmed}")
    print(f"Target QR  : {sm.mission_target_qr}")