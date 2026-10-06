# ArduPilot parameter sets (spec §5)

One `.param` file per stage. Load the set that matches the stage under test.
**Reboot the FC after loading a set that touches `PRX1_TYPE` or `OA_TYPE`**
(they take effect on reboot); `stage0_bench` needs no reboot. Commit gate
results with the exact `.param` file that produced them; a gate result without
its parameters is meaningless.

| File | Stage(s) | Adds |
|---|---|---|
| `stage0_bench.param` | 0 + GPS survey | `LOG_DISARMED=1` only (powered, never armed) |
| `stage1_proximity.param` | 1 | MAVLink proximity input only (display, not action) |
| `stage2_simple_avoidance.param` | 2 | + simple avoidance (Loiter/AltHold test instrument) |
| `stage5_6_bendyruler.param` | 5, 6 | + BendyRuler path planning + GCS failsafe = Land |

## Loading

MAVProxy:
```
param load params/stage5_6_bendyruler.param
reboot
```

MAVROS:
```
ros2 run mavros mavparam load params/stage5_6_bendyruler.param
```

## The values that are deliberately not 2.0

`AVOID_MARGIN` and `OA_MARGIN_MAX` are both **0.6 m**. BendyRuler only passes
between a wall and an obstacle when the gap is at least 2 × margin. The corridor
is 3.5 m wide: with 1.2 m a centred 0.8 m obstacle (1.35 m clear either side)
blocks the leg; with 0.6 m it passes. The cost is flying ~0.6 m from walls and
obstacles. Obstacle sizes are unknown until the venue — revisit this number
(and only this number) when they are known.

## BendyRuler in GUIDED needs GUID_OPTIONS bit 6

ArduPilot 4.7.1 GUIDED flies position targets with the plain position
controller unless `GUID_OPTIONS` bit 6 (64) is set; only the waypoint navigator
runs BendyRuler. `path_planner` sets bit 6 for corridor 2 and clears it after,
so leave `GUID_OPTIONS` at 0 on the FC (or put extra bits in the planner's
`guided_options_base`).

## WP_SPD is staged (it replaced WPNAV_SPEED)

4.7 renamed `WPNAV_SPEED` (cm/s) to `WP_SPD` (**m/s**); loading an old file with
`WPNAV_SPEED` fails on that line. With bit 6 set, `WP_SPD` is the corridor-2
speed — `DO_CHANGE_SPEED` is overwritten on every new target — so the planner
writes `corridor_obs_speed_ms` into it at the start of the leg. Stage it on
hardware through that parameter: 0.3 → 0.6 → 1.0 m/s, re-running the box test
after each change; the **first corridor flight is 0.3**.

## Two subsystems, one trap

`AVOID_*` (simple avoidance) only acts horizontally in AltHold/Loiter. `OA_*`
(BendyRuler) is the AUTO/GUIDED path planner. They read the same proximity data
but are independent — confirm which margin is moving before tuning it.
