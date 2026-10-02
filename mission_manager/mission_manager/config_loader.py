# config_loader.py — Central parameter declaration for all mission nodes
#
# Instead of scattering constants across source files, every node calls
#   cfg = load_mission_config(node)
# and then reads cfg.arena_width_m, cfg.cruise_altitude_m, etc.
#
# Default values are compiled here so nodes work even when launched
# without a YAML parameter file (e.g. in unit tests or demos).
#
# Usage:
#   from mission_manager.config_loader import load_mission_config
#   cfg = load_mission_config(self)   # inside a Node.__init__
#   print(cfg.cruise_altitude_m)


class MissionConfig:
    """Simple attribute-access namespace for mission parameters."""

    def __repr__(self):
        attrs = ", ".join(f"{k}={v!r}" for k, v in self.__dict__.items())
        return f"MissionConfig({attrs})"


def _declare(node, name: str, default):
    """Declare a ROS2 parameter (skip if already declared) and return its value."""
    try:
        node.declare_parameter(name, default)
    except Exception:
        # Parameter already declared — just read it
        pass
    return node.get_parameter(name).value


def load_mission_config(node) -> MissionConfig:
    """
    Declare all mission parameters on *node* and return a populated
    MissionConfig object.

    Parameters come from the YAML loaded via the launch file.  If a
    parameter is not in the YAML, the compiled default is used.
    """
    cfg = MissionConfig()

    # ── Arena & Course Geometry ──────────────────────────────────
    cfg.corridor_length_m     = _declare(node, "corridor_length_m", 10.0)
    cfg.corridor_width_m      = _declare(node, "corridor_width_m", 3.5)
    cfg.corridor_height_m     = _declare(node, "corridor_height_m", 3.048)
    cfg.arena_width_m         = _declare(node, "arena_width_m", 40.0)
    cfg.arena_height_m        = _declare(node, "arena_height_m", 30.0)

    # ── Altitudes ────────────────────────────────────────────────
    cfg.drop_altitude_m       = _declare(node, "drop_altitude_m", 5.0)
    cfg.cruise_altitude_m     = _declare(node, "cruise_altitude_m", 5.0)

    # ── Speeds ───────────────────────────────────────────────────
    cfg.corridor_speed_ms     = _declare(node, "corridor_speed_ms", 2.0)
    cfg.corridor_obs_speed_ms = _declare(node, "corridor_obs_speed_ms", 1.0)
    cfg.arena_speed_ms        = _declare(node, "arena_speed_ms", 3.0)

    # ── GPS Origin ───────────────────────────────────────────────
    cfg.origin_lat            = _declare(node, "origin_lat", 12.9716)
    cfg.origin_lon            = _declare(node, "origin_lon", 77.5946)

    # ── Camera / OAK-D ──────────────────────────────────────────
    cfg.oak_hfov_deg          = _declare(node, "oak_hfov_deg", 62.2)
    cfg.oak_warning_dist_m    = _declare(node, "oak_warning_dist_m", 1.5)
    cfg.strip_overlap         = _declare(node, "strip_overlap", 0.80)

    # ── Battery ──────────────────────────────────────────────────
    cfg.battery_critical_v    = _declare(node, "battery_critical_v", 10.0)
    cfg.battery_warning_v     = _declare(node, "battery_warning_v", 14.0)
    cfg.battery_capacity_mah  = _declare(node, "battery_capacity_mah", 5200.0)
    cfg.battery_cells         = _declare(node, "battery_cells", 4)
    cfg.nominal_voltage_v     = _declare(node, "nominal_voltage_v", 14.8)
    cfg.auw_grams             = _declare(node, "auw_grams", 1950.0)

    # ── Timeouts & Tolerances ────────────────────────────────────
    cfg.service_timeout_s     = _declare(node, "service_timeout_s", 10.0)
    cfg.altitude_timeout_s    = _declare(node, "altitude_timeout_s", 30.0)
    cfg.altitude_tolerance_m  = _declare(node, "altitude_tolerance_m", 0.5)
    cfg.arrival_tolerance_m   = _declare(node, "arrival_tolerance_m", 1.5)
    cfg.arrival_timeout_s     = _declare(node, "arrival_timeout_s", 60.0)
    cfg.goto_publish_count    = _declare(node, "goto_publish_count", 5)

    # ── Mission Node Specific ────────────────────────────────────
    cfg.tick_rate_hz          = _declare(node, "tick_rate_hz", 2.0)
    cfg.auto_start            = _declare(node, "auto_start", True)
    cfg.rc_auto_channel       = _declare(node, "rc_auto_channel", 5)
    cfg.rc_auto_pwm_thresh    = _declare(node, "rc_auto_pwm_thresh", 1500)
    cfg.qr_fallback_id        = _declare(node, "qr_fallback_id", "QR_TARGET_DEMO_ID")

    # ── Safety Monitor Specific ──────────────────────────────────
    cfg.safety_check_rate_hz     = _declare(node, "safety_check_rate_hz", 5.0)
    cfg.pointcloud_noise_min_m   = _declare(node, "pointcloud_noise_min_m", 0.3)
    cfg.obstacle_warn_throttle_s = _declare(node, "obstacle_warn_throttle_s", 1.0)

    # ── Path Planner Specific ────────────────────────────────────
    cfg.corridor_timeout_s          = _declare(node, "corridor_timeout_s", 120.0)
    cfg.lawnmower_timeout_s         = _declare(node, "lawnmower_timeout_s", 600.0)
    cfg.mavros_arrival_tolerance_m  = _declare(node, "mavros_arrival_tolerance_m", 2.0)
    cfg.corridor_entry_x            = _declare(node, "corridor_entry_x", 0.0)
    cfg.corridor_entry_y            = _declare(node, "corridor_entry_y", 15.0)
    cfg.red_zone_buffer_m           = _declare(node, "red_zone_buffer_m", 0.5)

    # ── RL Obstacle Avoidance (placeholder) ──────────────────────
    cfg.use_rl_avoidance  = _declare(node, "use_rl_avoidance", False)
    cfg.rl_model_path     = _declare(node, "rl_model_path", "")
    cfg.rl_sensor_topic   = _declare(node, "rl_sensor_topic", "/oak/points")

    # ── Payload Specific ─────────────────────────────────────────
    cfg.winch_channel       = _declare(node, "winch_channel", 9)
    cfg.release_channel     = _declare(node, "release_channel", 10)
    cfg.winch_pwm_lower     = _declare(node, "winch_pwm_lower", 1100)
    cfg.winch_pwm_stop      = _declare(node, "winch_pwm_stop", 1500)
    cfg.release_pwm_open    = _declare(node, "release_pwm_open", 1100)
    cfg.release_pwm_lock    = _declare(node, "release_pwm_lock", 1900)
    cfg.lower_duration_s    = _declare(node, "lower_duration_s", 5.0)
    cfg.contact_timeout_s   = _declare(node, "contact_timeout_s", 15.0)
    cfg.release_pause_s     = _declare(node, "release_pause_s", 1.0)

    return cfg
