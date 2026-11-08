"""AX-12A protocol 1.0 driver for the Jupiter 2 head pan joint.

Why this exists rather than ros2_control
----------------------------------------
`dynamixel_hardware_interface` (ROBOTIS) cannot drive this servo. Its source
builds its packet handler with `dynamixel::PacketHandler::getPacketHandler()`
with no argument, and that default is protocol 2.0. The AX-12A speaks protocol
1.0, and there is no ax12 entry in its `param/dxl_model/` table either, so the
plugin has no model ID to identify it by. Pointing ros2_control at /dev/ttyUSB1
would silently fail to ping the servo.

So this node owns the bus directly, the same way the Jupitron access layer does.
It is deliberately small: one joint, one port, explicit protocol-1.0 frames.

Interface
---------
    sub  /head_controller/joint_trajectory   trajectory_msgs/JointTrajectory
    sub  /head/joint_goal                    sensor_msgs/JointState
    pub  /joint_states                       sensor_msgs/JointState
    srv  /head/enable                        std_srvs/SetBool

Why BOTH trajectory and JointState input: the description package's
MoveIt/ros2_control configs publish FollowJointTrajectory via
/head_controller/joint_trajectory, while the simplest teleop just wants "go to
this angle". Supporting the trajectory topic means this node drops straight into
the existing launch files; the JointState topic means a student can test it with
one `ros2 topic pub`.

Safety
------
* auto_enable is FALSE by default. The node reads position but does not energise
  the servo until you ask. That matches the Jupitron bridge.
* Every goal is clamped to the URDF limits (-1.5 .. 1.5 rad) and then to the
  servo's own 0..1023 tick range.
* On shutdown torque is disabled, because a head left holding current heats up
  and there is no fan in the assembly.

The 300-degree trap
-------------------
An AX-12 spans 300 degrees over 1024 ticks, not 360. `RAD_PER_TICK` therefore
uses 300/1024, not 2*pi/1024. Getting this wrong makes every commanded angle
20 % too small -- which looks like "the servo undershoots" and is very hard to
spot, because it is perfectly repeatable.
"""

import math
import os
import select
import termios
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
# ------------------------------------------------------------- protocol 1.0 ---

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

TICKS_PER_REV = 1024
CENTRE_TICK = 512
# AX-12 spans 300 degrees over 1024 ticks -- NOT 360.
RAD_PER_TICK = (300.0 / 1024.0) * math.pi / 180.0
RAD_PER_SEC_PER_UNIT = 0.111 * 2.0 * math.pi / 60.0
NM_PER_LOAD_UNIT = 0.001 * 1.5


def checksum(body):
    """Protocol 1.0: sum ID + LENGTH + INSTRUCTION + params, then invert.

    This INCLUDES the id, which is the same formula Feetech uses. That near
    identity is why an AX-12 at ID 6 answers a Feetech ping frame.
    """
    return ~sum(body) & 0xFF


def build(dxl_id, instr, params=b""):
    length = 2 + len(params)
    body = bytes([dxl_id, length, instr]) + bytes(params)
    return b"\xff\xff" + body + bytes([checksum(body)])


def tick_to_rad(tick):
    return (tick - CENTRE_TICK) * RAD_PER_TICK


def rad_to_tick(rad):
    return CENTRE_TICK + int(round(rad / RAD_PER_TICK))


def decode_speed(raw):
    """Present speed is sign-magnitude in bit 10, not two's complement."""
    mag = raw & 0x03FF
    return -mag * RAD_PER_SEC_PER_UNIT if raw & 0x0400 else mag * RAD_PER_SEC_PER_UNIT


def decode_load(raw):
    mag = raw & 0x03FF
    return -mag * NM_PER_LOAD_UNIT if raw & 0x0400 else mag * NM_PER_LOAD_UNIT


class Ax12Bus:
    """Blocking protocol-1.0 transactions on one servo."""

    def __init__(self, port, dxl_id=6, baud=1_000_000, timeout=0.05):
        self.port = port
        self.dxl_id = dxl_id
        self.baud = baud
        self.timeout = timeout
        self.fd = None
        self._buf = b""

    def open(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            import fcntl
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (ImportError, OSError) as exc:
            os.close(self.fd)
            self.fd = None
            raise IOError("%s is already in use by another process (%s); "
                          "only one owner may drive a servo bus"
                          % (self.port, exc))
        try:
            attrs = termios.tcgetattr(self.fd)
            speed = getattr(termios, "B%d" % self.baud)
            attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE
                          | termios.CRTSCTS)
            attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                          | termios.ISTRIP | termios.INLCR | termios.IGNCR
                          | termios.ICRNL | termios.IXON | termios.IXOFF)
            attrs[1] &= ~termios.OPOST
            attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
                          | termios.ISIG | termios.IEXTEN)
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            self.close()
            raise
        return self

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def _send(self, instr, params=b""):
        pkt = build(self.dxl_id, instr, params)
        view = memoryview(pkt)
        deadline = time.monotonic() + 0.25
        while view:
            try:
                view = view[os.write(self.fd, view):]
            except BlockingIOError:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise IOError("serial write timed out")
                select.select([], [self.fd], [], left)

    def _recv(self, deadline):
        while time.monotonic() < deadline:
            i = self._buf.find(b"\xff\xff")
            if i >= 0 and len(self._buf) >= i + 4:
                length = self._buf[i + 3]
                total = i + 4 + length
                if len(self._buf) >= total:
                    frame = self._buf[i:total]
                    self._buf = self._buf[total:]
                    if frame[2] == self.dxl_id:
                        return frame
                    continue
            left = deadline - time.monotonic()
            if left <= 0:
                break
            r, _, _ = select.select([self.fd], [], [], left)
            if not r:
                break
            try:
                chunk = os.read(self.fd, 256)
            except BlockingIOError:
                continue
            if chunk:
                self._buf += chunk
        return None

    def _txrx(self, instr, params, expect_len):
        self._send(instr, params)
        frame = self._recv(time.monotonic() + self.timeout)
        if frame is None:
            return None, None
        err = frame[4]
        data = frame[5:5 + expect_len] if expect_len else b""
        if len(data) < expect_len:
            return None, err
        return data, err

    def ping(self):
        _, err = self._txrx(INST_PING, b"", 0)
        return err is not None

    def read1(self, addr):
        data, err = self._txrx(INST_READ, bytes([addr, 1]), 1)
        return (data[0], err) if data else (None, err)

    def read2(self, addr):
        data, err = self._txrx(INST_READ, bytes([addr, 2]), 2)
        if not data:
            return None, err
        return (data[0] | (data[1] << 8)), err

    def read_block(self, addr, length):
        """One 0x02 READ covering `length` consecutive registers.

        Registers 36..43 (position, speed, load, voltage, temperature) are
        contiguous, so ONE transaction replaces five. This matters far more
        than it looks: each transaction costs a full FTDI latency_timer tick
        (~16 ms by default), so 5 separate reads cost ~80 ms while one block
        read costs ~16 ms. The node physically cannot reach its target rate
        without this.
        """
        data, err = self._txrx(INST_READ, bytes([addr, length & 0xFF]), length)
        if not data or len(data) < length:
            return None, err
        return data, err

    def write1(self, addr, value):
        _, err = self._txrx(INST_WRITE, bytes([addr, value & 0xFF]), 0)
        return err == 0 if err is not None else False

    def write2(self, addr, value):
        _, err = self._txrx(
            INST_WRITE,
            bytes([addr, value & 0xFF, (value >> 8) & 0xFF]), 0)
        return err == 0 if err is not None else False


# --------------------------------------------------------------------- node ---

# FTDI FT232H. Matched on VID:PID because the tty number is not stable: on this
# chassis the CP2102 (RPLidar) and this FTDI swap between /dev/ttyUSB0 and
# /dev/ttyUSB1 across reboots.
FTDI_VID = "0403"
STABLE_HEAD_LINK = "/dev/jupiter/head"


def _find_ftdi_tty():
    """Return the /dev/tty* backed by an FTDI adapter, or '' when absent."""
    import glob
    for tty in sorted(glob.glob("/dev/ttyUSB*")):
        name = os.path.basename(tty)
        base = os.path.join("/sys/class/tty", name, "device")
        if not os.path.isdir(base):
            continue
        cur = os.path.realpath(base)
        for _ in range(8):
            vid_p = os.path.join(cur, "idVendor")
            if os.path.isfile(vid_p):
                try:
                    with open(vid_p) as f:
                        vid = f.read().strip()
                except OSError:
                    break
                if vid == FTDI_VID:
                    return tty
                break
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    return ""


def _default_head_port():
    """Stable symlink if installed, else the FTDI found by VID, else a guess."""
    if os.path.exists(STABLE_HEAD_LINK):
        return STABLE_HEAD_LINK
    found = _find_ftdi_tty()
    if found:
        return found
    return "/dev/ttyUSB1"


class HeadNode(Node):
    def __init__(self):
        super().__init__("head_node")

        self.declare_parameter("port", _default_head_port())
        self.declare_parameter("dxl_id", 6)
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("joint_name", "head_joint")
        self.declare_parameter("min_rad", -1.5)
        self.declare_parameter("max_rad", 1.5)
        # Safety: do not energise on start. Read-only until /head/enable is
        # called or auto_enable is explicitly set true.
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("disable_torque_on_stop", True)

        self.port = str(self.get_parameter("port").value)
        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.baud = int(self.get_parameter("baud").value)
        self.joint = str(self.get_parameter("joint_name").value)
        self.min_rad = float(self.get_parameter("min_rad").value)
        self.max_rad = float(self.get_parameter("max_rad").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.disable_on_stop = bool(
            self.get_parameter("disable_torque_on_stop").value)

        self.bus = Ax12Bus(self.port, self.dxl_id, self.baud)
        self.enabled = False
        self.goal_rad = None
        self._last_good = None
        self._torque_cached = None

        try:
            self.bus.open()
            self.get_logger().info("opened %s" % self.port)
        except OSError as exc:
            self.get_logger().error(
                "cannot open %s: %s -- node will publish nothing" % (self.port, exc))
            self.bus.fd = None

        if self.bus.fd is not None:
            if self.bus.ping():
                self.get_logger().info("AX-12 id %d answered on %s"
                                       % (self.dxl_id, self.port))
                pos, _ = self.bus.read2(ADDR_PRESENT_POSITION)
                if pos is not None:
                    self.goal_rad = tick_to_rad(pos)
                    self.get_logger().info(
                        "present position %d ticks (%.3f rad)"
                        % (pos, self.goal_rad))
            else:
                self.get_logger().warn(
                    "no AX-12 id %d on %s -- check the port and that nothing "
                    "else holds it" % (self.dxl_id, self.port))

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        # Full servo telemetry, including fields JointState cannot carry
        # (voltage, temperature, raw tick, bus error count). This exists so the
        # access panel can display the Servo tab WITHOUT opening the bus: the
        # bus is exclusive, and a second opener corrupts it (measured 45% bad
        # reads with two owners vs 0% with one).
        self.pub_telemetry = self.create_publisher(
            DiagnosticArray, "/head/telemetry", 10)
        self._bus_errors = 0
        self.create_subscription(
            JointTrajectory, "/head_controller/joint_trajectory",
            self._on_trajectory, 10)
        self.create_subscription(JointState, "/head/joint_goal",
                                 self._on_goal, 10)
        self.create_service(SetBool, "/head/enable", self._on_enable)

        self.create_timer(1.0 / max(1.0, self.rate_hz), self._publish_state)

        if self.auto_enable:
            self._set_torque(True)

        self.get_logger().info(
            "head_node ready: %s on %s, limits %.2f..%.2f rad, torque %s"
            % (self.joint, self.port, self.min_rad, self.max_rad,
               "ON" if self.enabled else "OFF (call /head/enable)"))

    # ------------------------------------------------------------- helpers
    def _clamp(self, rad):
        """Clamp to URDF limits, then to the servo's own tick range."""
        r = max(self.min_rad, min(self.max_rad, float(rad)))
        tick = rad_to_tick(r)
        if tick < 0:
            tick = 0
        elif tick > 1023:
            tick = 1023
        return tick_to_rad(tick)

    def _set_torque(self, on):
        if self.bus.fd is None:
            return False
        ok = self.bus.write1(ADDR_TORQUE_ENABLE, 1 if on else 0)
        self.enabled = bool(ok and on)
        return ok

    def _goto(self, rad, why=""):
        if self.bus.fd is None:
            return
        target = self._clamp(rad)
        tick = rad_to_tick(target)
        # Seed the goal before enabling: writing a goal while torque is off is
        # safe and prevents a jump when torque comes on.
        ok = self.bus.write2(ADDR_GOAL_POSITION, tick)
        if ok:
            self.goal_rad = target
            self.get_logger().debug("%s -> %.3f rad (%d ticks)"
                                    % (why or "goal", target, tick))
        else:
            self.get_logger().warn("write goal failed on %s" % self.port)

    # ----------------------------------------------------------- callbacks
    def _on_enable(self, req, res):
        ok = self._set_torque(req.data)
        res.success = ok
        if ok:
            res.message = "torque ON" if req.data else "torque OFF"
        else:
            res.message = "write failed on %s" % self.port
        self.get_logger().info("torque %s: %s"
                               % ("on" if req.data else "off", res.message))
        return res

    def _on_goal(self, msg):
        """Accept a JointState; use position[0] for our joint if named."""
        if not msg.position:
            return
        idx = 0
        if msg.name:
            if self.joint not in msg.name:
                return
            idx = msg.name.index(self.joint)
            if idx >= len(msg.position):
                return
        self._goto(msg.position[idx], "JointState goal")

    def _on_trajectory(self, msg):
        """Take the LAST point: this is a position-only sink, not a controller.

        A real JointTrajectoryController interpolates; doing so here would mean
        duplicating ros2_control. Taking the final waypoint is honest about what
        this node is, and matches how teleop goals are actually used.
        """
        if not msg.points:
            return
        p = msg.points[-1]
        if not p.positions:
            return
        idx = 0
        if msg.joint_names:
            if self.joint not in msg.joint_names:
                return
            idx = msg.joint_names.index(self.joint)
            if idx >= len(p.positions):
                return
        if not self.enabled:
            self.get_logger().warn(
                "trajectory received but torque is off -- call /head/enable "
                "with {data: true}")
        self._goto(p.positions[idx], "trajectory goal")

    def _publish_state(self):
        if self.bus.fd is None:
            return

        # ONE block read covers registers 36..43:
        #   36-37 present position, 38-39 present speed, 40-41 present load,
        #   42 present voltage, 43 present temperature.
        # Previously this took 5 separate transactions (plus a 6th for torque),
        # and each transaction blocks for a full FTDI latency_timer tick
        # (~16 ms), so a cycle cost ~96 ms and the node only achieved ~10 Hz
        # while advertising 20 Hz. One block read costs ~16 ms.
        raw, err = self.bus.read_block(ADDR_PRESENT_POSITION, 8)

        if raw is None:
            self._bad_reads = getattr(self, "_bad_reads", 0) + 1
            self._bus_errors += 1
            if self._last_good is None:
                return
            pos = self._last_good
            if self._bad_reads % 20 == 1:
                self.get_logger().warn(
                    "servo read invalid (no reply); holding last good tick %d. "
                    "Check that no other process owns %s."
                    % (self._last_good, self.port))
            spd = ld = volt = temp = None
            tq = None
        else:
            pos = raw[0] | (raw[1] << 8)
            spd = raw[2] | (raw[3] << 8)
            ld = raw[4] | (raw[5] << 8)
            volt = raw[6]
            temp = raw[7]

        # A corrupt reply must NEVER become a publication. Tick 0 (or >1023)
        # decodes to -2.618 rad, the servo's hard negative limit, and looked
        # like the head had slammed into a mechanical stop.
        if raw is not None and (pos < 0 or pos > 1023):
            self._bad_reads = getattr(self, "_bad_reads", 0) + 1
            self._bus_errors += 1
            if self._last_good is None:
                return
            pos = self._last_good
            spd = ld = volt = temp = None
        elif raw is not None:
            self._bad_reads = 0
            self._last_good = pos

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint]
        js.position = [tick_to_rad(pos)]
        if spd is not None:
            js.velocity = [decode_speed(spd)]
        if ld is not None:
            js.effort = [decode_load(ld)]

        try:
            self.pub.publish(js)
            self._publish_telemetry(pos, js, volt=volt, temp=temp)
        except Exception:                            # noqa: BLE001
            # Context can be torn down under us on SIGTERM; a dead publisher is
            # not an error worth crashing the shutdown path over.
            pass

    def _publish_telemetry(self, tick, js, volt=None, temp=None):
        """Publish the fields JointState cannot carry, on /head/telemetry.

        This is what lets the access panel show voltage/temperature/raw tick
        without opening the bus itself.

        volt/temp arrive already-read from the block read in _publish_state.
        Re-reading them here would add two more ~16 ms transactions per cycle
        for data we already have. Torque is only read every N cycles because it
        changes rarely and each read costs another full latency tick.
        """
        self._tel_cycle = getattr(self, "_tel_cycle", 0) + 1
        if self._tel_cycle % 10 == 1 or self._torque_cached is None:
            tq = self.bus.read1(ADDR_TORQUE_ENABLE)
            self._torque_cached = bool(tq and tq[0])
        tq_on = self._torque_cached

        kv = [
            KeyValue(key="tick", value=str(int(tick))),
            KeyValue(key="rad", value="%.5f" % tick_to_rad(tick)),
            KeyValue(key="torque", value="true" if tq_on else "false"),
            KeyValue(key="bus_errors", value=str(self._bus_errors)),
            KeyValue(key="port", value=self.port),
        ]
        if js.velocity:
            kv.append(KeyValue(key="speed", value="%.5f" % js.velocity[0]))
        if js.effort:
            kv.append(KeyValue(key="load", value="%.5f" % js.effort[0]))
        if volt is not None:
            kv.append(KeyValue(key="voltage", value="%.1f" % (volt / 10.0)))
        if temp is not None:
            kv.append(KeyValue(key="temp", value=str(temp)))

        st = DiagnosticStatus()
        st.name = "head_pan"
        st.hardware_id = "ax12_id%d" % self.dxl_id
        st.level = (DiagnosticStatus.WARN if temp and temp > 60
                    else DiagnosticStatus.OK)
        st.message = "ok" if st.level == DiagnosticStatus.OK else "hot"
        st.values = kv

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_telemetry.publish(arr)

    def destroy_node(self):
        if self.disable_on_stop and self.bus.fd is not None:
            try:
                self.bus.write1(ADDR_TORQUE_ENABLE, 0)
                self.get_logger().info("torque disabled on shutdown")
            except Exception:                        # noqa: BLE001
                pass
        self.bus.close()
        super().destroy_node()


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = HeadNode()
    from rclpy._rclpy_pybind11 import RCLError
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM/SIGINT is the normal way to stop this.
        pass
    except RCLError as exc:
        # If the context is torn down while the wait set is being rebuilt,
        # rclpy.spin() raises RCLError("...context is not valid...") instead of
        # ExternalShutdownException. That is a shutdown, not a fault, so it
        # must not print a traceback -- an operator reading the log would think
        # the driver crashed.
        if rclpy.ok():
            node.get_logger().error("spin aborted unexpectedly: %s" % exc)
    finally:
        try:
            node.destroy_node()
        except Exception:                            # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
