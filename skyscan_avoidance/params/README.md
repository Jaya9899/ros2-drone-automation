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

`AVOID_MARGIN` and `OA_MARGIN_MAX` are both **1.2 m**, not 2.0. The corridor is
3.5 m wide, so a 2.0 m margin fits nowhere in it; BendyRuler would weave down
the centreline or refuse the leg. 1.2 is the only place corridor width enters
the design — change this (and only this) if the venue corridor is narrower.

## WPNAV_SPEED is staged

It is centimetres/second and it changes between sub-steps — do not treat the
committed value as fixed. Sim nominal is 100 (1.0 m/s). On hardware, ramp
30 → 60 → 100 → 150 per Stage 8, re-running the box test after each change; the
**first corridor flight is 30**, never the open-field speed carried straight in.

## Two subsystems, one trap

`AVOID_*` (simple avoidance) only acts horizontally in AltHold/Loiter. `OA_*`
(BendyRuler) is the AUTO/GUIDED path planner. They read the same proximity data
but are independent — confirm which margin is moving before tuning it.
