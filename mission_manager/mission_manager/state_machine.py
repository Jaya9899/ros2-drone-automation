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
    NAVIGATE_TO_DROP  = State()  
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
    nav_to_drop       = TARGET_FOUND.to(NAVIGATE_TO_DROP)
    drop_position_reached = NAVIGATE_TO_DROP.to(DROPPING)
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
        NAVIGATE_TO_DROP.to(RC_MODE) |
        DROPPING.to(RC_MODE) |
        CORRIDOR_2_TRANSIT.to(RC_MODE) |
        HOMING.to(RC_MODE)
    )

    # state data
    def __init__(self, ros_node=None, cfg=None):
        self.ros_node           = ros_node  
        self.cfg                = cfg
        self.mission_target_qr  = None 
        self.target_gps         = None   
        self.home_position      = None      # stored on takeoff for RTL
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
                set_mode, arm, takeoff, wait_for_altitude, get_current_gps,
            )
            svc_timeout = self._c("service_timeout_s", 10.0)
            # Record home position before takeoff so HOMING can return here
            home = get_current_gps(self.ros_node, timeout=svc_timeout)
            if home:
                self.home_position = home
                print(f"[{self._elapsed()}]   Home position saved: {self.home_position}")
            else:
                print(f"[{self._elapsed()}]   WARNING: Could not read home GPS, "
                      f"will use MAVROS RTL as fallback")

            set_mode(self.ros_node, "GUIDED", timeout=svc_timeout)
            arm(self.ros_node, timeout=svc_timeout)
            takeoff(self.ros_node, altitude_m=cruise_alt, timeout=svc_timeout)
            wait_for_altitude(
                self.ros_node, target_alt=cruise_alt,
                tolerance=self._c("altitude_tolerance_m", 0.5),
                timeout=self._c("altitude_timeout_s", 30.0),
            )
        else:
            # demo mode – set a placeholder home position
            self.home_position = (
                self._c("origin_lat", 12.9716),
                self._c("origin_lon", 77.5946),
            )

    def on_enter_CORRIDOR_1_TRANSIT(self, event, state):
        speed = self._c("corridor_speed_ms", 2.0)
        height = self._c("corridor_height_m", 3.048)
        print(f"[{self._elapsed()}] [CORRIDOR_1_TRANSIT] "
              f"Entering clean corridor, speed {speed} m/s, "
              f"max height {height} m")
        if self.ros_node:
            from mission_manager.srv_helpers import call_fly_corridor
            call_fly_corridor(
                self.ros_node, speed=speed, obstacles=False,
                timeout=self._c("service_timeout_s", 10.0),
            )

    def on_enter_ARENA_SEARCH(self, event, state):
        arena_w = self._c("arena_width_m", 40.0)
        arena_h = self._c("arena_height_m", 30.0)
        speed = self._c("arena_speed_ms", 1.5)
        print(f"[{self._elapsed()}] [ARENA_SEARCH] "
              f"Starting BCD lawnmower over {arena_w}×{arena_h} m arena "
              f"at {speed} m/s")
        if self.ros_node:
            from mission_manager.srv_helpers import call_run_lawnmower, call_start_qr_scan
            timeout = self._c("service_timeout_s", 10.0)
            call_run_lawnmower(self.ros_node, speed=speed, timeout=timeout)
            call_start_qr_scan(self.ros_node, target_qr_id=self.mission_target_qr,
                               timeout=timeout)

    def on_enter_TARGET_FOUND(self, event, state):
        print(f"[{self._elapsed()}] [TARGET_FOUND] QR matched "
              f"Target GPS: {self.target_gps}, pausing lawnmower.")
        if self.ros_node:
            from mission_manager.srv_helpers import call_stop_lawnmower
            call_stop_lawnmower(
                self.ros_node,
                timeout=self._c("service_timeout_s", 10.0),
            )

    def on_enter_NAVIGATE_TO_DROP(self, event, state):
        if self.target_gps is None:
            print("[ERROR] target_gps not set, cannot navigate to drop!")
            self.abort()
            return
        drop_alt = self._c("drop_altitude_m", 5.0)
        tolerance = self._c("arrival_tolerance_m", 1.5)
        svc_timeout = self._c("service_timeout_s", 10.0)
        print(f"[{self._elapsed()}] [NAVIGATE_TO_DROP] "
              f"Flying to {self.target_gps}, maintaining {drop_alt} m AGL")
        if self.ros_node:
            from mission_manager.mavros_utils import goto, wait_for_arrival
            lat, lon = self.target_gps
            goto(self.ros_node, lat=lat, lon=lon, altitude_m=drop_alt,
                 publish_count=self._c("goto_publish_count", 5))
            wait_for_arrival(self.ros_node, lat=lat, lon=lon,
                             tolerance_m=tolerance,
                             timeout=self._c("arrival_timeout_s", 60.0))
         
            print(f"[{self._elapsed()}]   Arrived at drop coordinates")

    def on_enter_DROPPING(self, event, state):
        drop_alt = self._c("drop_altitude_m", 5.0)
        cruise_alt = self._c("cruise_altitude_m", 5.0)
        print(f"[{self._elapsed()}] [DROPPING] "
              f"At {drop_alt} m lowering payload")
        if self.ros_node:
            from mission_manager.srv_helpers import (
                call_payload_lower, call_payload_wait_contact, call_payload_release,
            )
            timeout = self._c("service_timeout_s", 10.0)
            # Step 1: Lower the payload via pulley/winch mechanism
            call_payload_lower(self.ros_node, timeout=timeout)
            print(f"[{self._elapsed()}]   Payload lowering...")

            # Step 2: Wait until the payload touches the ground
            contact = call_payload_wait_contact(self.ros_node, timeout=timeout)
            if contact:
                print(f"[{self._elapsed()}]   Ground contact confirmed")
            else:
                print(f"[{self._elapsed()}]   WARNING: Ground contact not confirmed, "
                      f"releasing anyway")

            # Step 3: Release the payload from the mechanism
            call_payload_release(self.ros_node, timeout=timeout)
            print(f"[{self._elapsed()}]   Payload released")
        else:
            print(f"[{self._elapsed()}]   (demo mode) Simulating payload drop sequence")

        self.drop_confirmed = True
        print(f"[{self._elapsed()}]   Payload released. Climbing back to {cruise_alt} m.")

        # Climb back to cruise altitude after drop
        if self.ros_node:
            from mission_manager.mavros_utils import goto, wait_for_altitude
            lat, lon = self.target_gps
            goto(self.ros_node, lat=lat, lon=lon, altitude_m=cruise_alt,
                 publish_count=self._c("goto_publish_count", 5))
            wait_for_altitude(
                self.ros_node, target_alt=cruise_alt,
                tolerance=self._c("altitude_tolerance_m", 0.5),
                timeout=self._c("altitude_timeout_s", 30.0),
            )
   

    def on_enter_CORRIDOR_2_TRANSIT(self, event, state):
        speed = self._c("corridor_obs_speed_ms", 1.0)
        warn_dist = self._c("oak_warning_dist_m", 1.5)
        print(f"[{self._elapsed()}] [CORRIDOR_2_TRANSIT] "
              f"Return corridor — static obstacles — speed {speed} m/s. "
              f"OAK-D active, warning at {warn_dist} m.")
        if self.ros_node:
            from mission_manager.srv_helpers import call_fly_corridor
            call_fly_corridor(
                self.ros_node, speed=speed, obstacles=True,
                timeout=self._c("service_timeout_s", 10.0),
            )
         

    def on_enter_HOMING(self, event, state):
        cruise_alt = self._c("cruise_altitude_m", 5.0)
        tolerance = self._c("arrival_tolerance_m", 1.5)
        print(f"[{self._elapsed()}] [HOMING] Returning to takeoff/landing zone")
        if self.ros_node:
            from mission_manager.mavros_utils import goto, wait_for_arrival, land
            if self.home_position:
                lat, lon = self.home_position
                print(f"[{self._elapsed()}]   Flying to home: ({lat}, {lon})")
                goto(self.ros_node, lat=lat, lon=lon, altitude_m=cruise_alt,
                     publish_count=self._c("goto_publish_count", 5))
                wait_for_arrival(self.ros_node, lat=lat, lon=lon,
                                 tolerance_m=tolerance,
                                 timeout=self._c("arrival_timeout_s", 60.0))
            else:
                from mission_manager.mavros_utils import set_mode
                print(f"[{self._elapsed()}]   No home position stored, using RTL mode")
                set_mode(self.ros_node, "RTL",
                         timeout=self._c("service_timeout_s", 10.0))
                return
            print(f"[{self._elapsed()}]   Over home, initiating landing...")
            land(self.ros_node, timeout=self._c("service_timeout_s", 10.0))

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
    sm.target_gps = (12.9716, 77.5946) #demo
    sm.qr_matched()

    sm.nav_to_drop()
    sm.drop_position_reached()
    sm.drop_complete()
    sm.c2_complete()
    sm.touchdown()

    print()
    print(f"Final state : {sm.current_state.id}")
    print(f"Drop confirmed : {sm.drop_confirmed}")
    print(f"Target QR  : {sm.mission_target_qr}")
    print(f"Target GPS : {sm.target_gps}")