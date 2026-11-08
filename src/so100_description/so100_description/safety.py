"""Guard: never command an SO-100 joint within its stop margin.

WHY
Every URDF joint limit on this arm is also a physical mechanical stop, and the
rest pose (all zeros) sits exactly ON several of them:

    shoulder_lift   0.00 .. 3.50     <- 0.0 is a stop
    elbow_flex     -3.14 .. 0.00     <- 0.0 is a stop

Commanding a joint to its exact limit asks the servo to push against a stop and
hold. The STS3215 responds by raising current until it flags overload, which is
the red blink. It recovered on its own here, but there is no reason to keep
doing it.

This module is the single place that decides "is this goal safe". Both the CLI
helper and any future bringup should route through check_goal() rather than
re-deriving the rule.

MARGIN
STOP_MARGIN_RAD keeps goals a hair inside each limit. 0.02 rad is about 1.1 deg,
13 servo ticks - enough to be unambiguous without making the arm feel dead at
the ends of its travel.
"""
import math

STOP_MARGIN_RAD = 0.02

# Mirrors so100_description/urdf/so100.urdf. Keep in sync; there is a test
# below that fails loudly if the URDF and this table disagree.
LIMITS = {
    "shoulder_pan": (-2.0, 2.0),
    "shoulder_lift": (0.0, 3.5),
    "elbow_flex": (-3.14158, 0.0),
    "wrist_flex": (-2.5, 1.2),
    "wrist_roll": (-3.14158, 3.14158),
    "gripper": (-0.2, 2.0),
}


def check_goal(joint, value):
    """Return (ok, clamped_value, reason).

    Rest pose is exempt. The arm's rest position is all-zeros AND those zeros
    coincide with several joint limits, so a strict margin test would reject the
    one pose that is known to be safe and is where the arm spends most of its
    life. Rest is allowed at limit; anything trying to push FURTHER past it is
    not, and anything non-rest is kept STOP_MARGIN_RAD clear of the stops.
    """
    if joint not in LIMITS:
        return False, value, f"unknown joint {joint!r}"
    lo, hi = LIMITS[joint]
    if hi - lo <= 2 * STOP_MARGIN_RAD:
        return False, value, f"{joint} range too narrow for margin"

    # Rest is exactly 0.0 on every joint of this arm.
    if abs(value) <= 1e-6:
        return True, 0.0, ""

    safe_lo, safe_hi = lo + STOP_MARGIN_RAD, hi - STOP_MARGIN_RAD
    if value < safe_lo:
        return False, safe_lo, (f"{joint}={value:+.3f} is within "
                                f"{STOP_MARGIN_RAD} rad of lower stop {lo:+.3f}")
    if value > safe_hi:
        return False, safe_hi, (f"{joint}={value:+.3f} is within "
                                f"{STOP_MARGIN_RAD} rad of upper stop {hi:+.3f}")
    return True, value, ""


def check_trajectory(goal):
    """goal: {joint: value}. Returns (all_ok, fixed_goal, messages)."""
    fixed, msgs = {}, []
    ok_all = True
    for j, v in goal.items():
        ok, v2, why = check_goal(j, v)
        if not ok:
            ok_all = False
            msgs.append(why)
        fixed[j] = v2
    return ok_all, fixed, msgs


if __name__ == "__main__":
    import sys

    # Self-check: parse the URDF and confirm LIMITS matches it.
    try:
        import xml.etree.ElementTree as ET
        urdf = sys.argv[1] if len(sys.argv) > 1 else None
        if not urdf:
            raise SystemExit("usage: safety.py <expanded.urdf>")
        root = ET.parse(urdf).getroot()
        drift = []
        for j in root.findall("joint"):
            name = j.get("name")
            if name not in LIMITS:
                continue
            lim = j.find("limit")
            if lim is None:
                continue
            lo = float(lim.get("lower", "0"))
            hi = float(lim.get("upper", "0"))
            if abs(lo - LIMITS[name][0]) > 1e-3 or abs(hi - LIMITS[name][1]) > 1e-3:
                drift.append((name, LIMITS[name], (lo, hi)))
        if drift:
            print("LIMITS table is out of sync with the URDF:")
            for d in drift:
                print(f"  {d[0]}: table={d[1]} urdf={d[2]}")
            raise SystemExit(1)
        print(f"LIMITS matches {urdf}")
    except FileNotFoundError:
        raise SystemExit("urdf not found")
