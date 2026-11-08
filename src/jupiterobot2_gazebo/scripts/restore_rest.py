#!/usr/bin/env python3
"""Park the SO-100 arm at rest, or report how far it has drifted.

    ros2 run jupiterobot2_gazebo restore_rest.py --dry-run   # report only
    ros2 run jupiterobot2_gazebo restore_rest.py             # drive to rest

WHY THIS EXISTS
preflight_check.py refuses to start ros2_control when the arm is not at rest,
because the driver asserts the current pose on activate and will hold a joint
against its mechanical stop under torque. When preflight fails it tells you to
park the arm; this is the tool that does it without you hand-bending a servo.

SAFETY
  * One process at a time: the arm bus is exclusive. Do not run this while
    hardware.launch.py is up. It will fail to open the port, which is correct.
  * Slow: acceleration 20, speed 600 ticks/s, then torque released. Nothing
    is left straining.
  * Only moves joints that are off rest by more than --tolerance ticks.
  * --dry-run touches nothing and is always safe.

REST TICKS
From the driver's own first-activate log:
    servo 1..6 := 2042 2046 2052 2034 2050 2042
Note these are NOT all 2048. Each servo's rest tick is where it physically
sits with the arm folded on its stand, which differs from the electrical
centre by a few ticks per servo. Do not "round" them to 2048.
"""
import math
import sys
import time

import serial

PORT_DEFAULT = "/dev/jupiter/arm"
BAUD = 1000000
TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)
CENTRE = 2048.0

REST = {1: 2042, 2: 2046, 3: 2052, 4: 2034, 5: 2050, 6: 2042}
NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
         4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

R_TORQUE, R_ACC, R_GOAL, R_PRESENT = 0x28, 0x29, 0x2A, 0x38
R_SPEED = 0x2E


def cs(vals):
    return (~(sum(vals))) & 0xFF


def wr(s, sid, addr, vals):
    body = [sid, 0x03 + len(vals), 0x03, addr] + list(vals)
    s.reset_input_buffer()
    s.write(bytes([0xFF, 0xFF] + body + [cs(body)]))
    time.sleep(0.02)
    return s.read(16)


def rd(s, sid, addr, n):
    body = [sid, 0x04, 0x02, addr, n]
    s.reset_input_buffer()
    s.write(bytes([0xFF, 0xFF] + body + [cs(body)]))
    time.sleep(0.02)
    r = s.read(64)
    if len(r) < 6 or r[0] != 0xFF or r[2] != sid:
        return None, None
    return r[5:5 + r[3] - 2], r[4]


def snap(s, sid):
    p, err = rd(s, sid, R_PRESENT, 10)
    if not p or len(p) < 10:
        return None
    pos = p[0] | (p[1] << 8)
    return pos, (p[4] | (p[5] << 8)) & 0x3FF, p[6] / 10.0, p[7], err


def rad(t):
    return (t - CENTRE) / TICKS_PER_RAD


def main(argv):
    dry = "--dry-run" in argv
    tol = 25
    port = PORT_DEFAULT
    for a in argv:
        if a.startswith("--tolerance="):
            tol = int(a.split("=", 1)[1])
        if a.startswith("--port="):
            port = a.split("=", 1)[1]

    try:
        s = serial.Serial(port, BAUD, timeout=0.3)
    except Exception as e:                                   # noqa: BLE001
        print(f"ERROR: cannot open {port}: {e}")
        print("If hardware.launch.py is running, stop it first - the arm bus "
              "allows one process at a time.")
        return 2

    print(f"{'id':>2} {'joint':<14} {'pos':>5} {'rest':>5} {'delta':>6} "
          f"{'rad':>7} {'load':>6}")
    drift = {}
    for sid in range(1, 7):
        r = snap(s, sid)
        if not r:
            print(f"{sid:>2} {'NO RESPONSE':<14}")
            s.close()
            return 3
        pos, load, volts, temp, err = r
        d = pos - REST[sid]
        if abs(d) > tol:
            drift[sid] = d
        print(f"{sid:>2} {NAMES[sid]:<14} {pos:>5} {REST[sid]:>5} {d:>+6} "
              f"{rad(pos):>+7.3f} {load / 10:>5.1f}%")

    if not drift:
        print(f"\nAll servos within {tol} ticks of rest. Nothing to do.")
        s.close()
        return 0

    print(f"\n{len(drift)} joint(s) off rest: "
          f"{', '.join(NAMES[i] for i in drift)}")
    if dry:
        print("--dry-run: not moving anything.")
        s.close()
        return 0

    print("driving to rest (accel 20, speed 600 ticks/s) ...")
    for sid in drift:
        wr(s, sid, R_TORQUE, [1])
        time.sleep(0.05)
        wr(s, sid, R_ACC, [20])
        wr(s, sid, R_SPEED, [600 & 0xFF, (600 >> 8) & 0xFF])
        goal = REST[sid]
        wr(s, sid, R_GOAL, [goal & 0xFF, (goal >> 8) & 0xFF])
        for _ in range(60):
            time.sleep(0.1)
            r = snap(s, sid)
            if r and abs(r[0] - goal) <= 4:
                break
        r = snap(s, sid)
        if r is None:
            print(f"  servo {sid} {NAMES[sid]:<14} READ FAILED after move")
            wr(s, sid, R_TORQUE, [0])
            continue
        print(f"  servo {sid} {NAMES[sid]:<14} now {r[0]:>5} "
              f"(target {goal}) load {r[1] / 10:5.1f}%")
        wr(s, sid, R_TORQUE, [0])

    print("\nfinal state:")
    worst = 0
    for sid in range(1, 7):
        r = snap(s, sid)
        if r:
            pos, load, volts, temp, err = r
            worst = max(worst, abs(pos - REST[sid]))
            print(f"  servo {sid} {NAMES[sid]:<14} {pos:>5} "
                  f"d={pos - REST[sid]:+4} load {load / 10:4.1f}% "
                  f"{volts:.1f}V {temp}C err=0x{err:02x}")
    s.close()
    print(f"\nmax deviation from rest: {worst} ticks")
    return 0 if worst <= tol else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
