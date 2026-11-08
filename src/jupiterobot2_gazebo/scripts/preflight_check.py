#!/usr/bin/env python3
"""Preflight: is the SO-100 arm parked at rest before we hand it to ros2_control?

WHY THIS EXISTS
ros2_control's rbc driver asserts the pose it reads on activate and holds it
under torque. If the arm is sitting on a mechanical stop, that becomes "push
into the stop and keep pushing" - the servo raises current until it flags
overload, which is the red status blink. That happened once already.

This node reads the six servos once, compares against the known rest ticks,
and exits non-zero with a readable message if the arm is not parked. It never
commands motion.

Rest ticks are what the driver itself logged on first activate:
    servo 1..6 := 2042 2046 2052 2034 2050 2042

Set tolerance_ticks to widen the check; 25 ticks is ~2.2 deg, enough for
mechanical slop and servo deadband without accepting a genuinely moved joint.
"""
import sys

import rclpy
import serial
from rclpy.node import Node

REST = {1: 2042, 2: 2046, 3: 2052, 4: 2034, 5: 2050, 6: 2042}
NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
         4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}


def cs(vals):
    return (~(sum(vals))) & 0xFF


def read_all(port, baud=1000000):
    """Return {id: (pos, load_pct, volts, temp, err)} or raise on bus failure."""
    out = {}
    s = serial.Serial(port, baud, timeout=0.3)
    try:
        for sid in range(1, 7):
            body = [sid, 0x04, 0x02, 0x38, 10]
            s.reset_input_buffer()
            s.write(bytes([0xFF, 0xFF] + body + [cs(body)]))
            import time
            time.sleep(0.03)
            r = s.read(64)
            if len(r) < 6 or r[0] != 0xFF or r[2] != sid:
                raise IOError(f"no response from servo {sid}")
            p = r[5:5 + r[3] - 2]
            pos = p[0] | (p[1] << 8)
            raw = p[4] | (p[5] << 8)
            out[sid] = (pos, (raw & 0x3FF) / 10.0, p[6] / 10.0, p[7], r[4])
    finally:
        s.close()
    return out


class Preflight(Node):
    def __init__(self):
        super().__init__("preflight_check")
        self.declare_parameter("port", "/dev/jupiter/arm")
        self.declare_parameter("tolerance_ticks", 25)
        self.declare_parameter("baudrate", 1000000)

    def run(self):
        port = self.get_parameter("port").value
        tol = int(self.get_parameter("tolerance_ticks").value)
        baud = int(self.get_parameter("baudrate").value)

        try:
            state = read_all(port, baud)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().error(
                f"PREFLIGHT FAILED: cannot read the arm bus on {port}: {e}")
            return 1

        moved = []
        for sid, (pos, load, volts, temp, err) in sorted(state.items()):
            delta = pos - REST[sid]
            flag = ""
            if abs(delta) > tol:
                flag = "  <== NOT AT REST"
                moved.append((sid, NAMES[sid], pos, REST[sid], delta))
            if err:
                flag += f"  ERROR=0x{err:02x}"
            if load >= 50:
                flag += f"  LOAD {load:.0f}%"
            self.get_logger().info(
                f"  servo {sid} {NAMES[sid]:<14} pos={pos:>4} "
                f"rest={REST[sid]:>4} d={delta:+5} load={load:4.1f}% "
                f"{volts:.1f}V {temp}C{flag}")

        if moved:
            self.get_logger().error(
                "PREFLIGHT FAILED: arm is not parked at rest pose. "
                "Refusing to hand it to ros2_control, because the driver would "
                "hold the current pose under torque and could overload a servo.")
            for sid, name, pos, rest, d in moved:
                self.get_logger().error(
                    f"  servo {sid} ({name}) at {pos}, expected {rest} "
                    f"({d:+d} ticks)")
            self.get_logger().error(
                "Fix: park the arm by hand, or run "
                "`ros2 run jupiterobot2_gazebo restore_rest.py` to drive it "
                "back to rest slowly, then relaunch.")
            return 1

        self.get_logger().info("PREFLIGHT OK: all 6 servos at rest pose.")
        return 0


def main():
    rclpy.init()
    node = Preflight()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
