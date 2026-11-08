"""Hardware backends for the Jupiter access panel.

Three ways into the robot, each with a different constraint:

  HeadBus   AX-12A servo on /dev/ttyUSB1 (FTDI). EXCLUSIVE access -- only one
            process can own the tty. If head_node is running the panel must not
            open it, or frames from two writers corrupt each other. resolve()
            detects that and says so instead of fighting for the port.

  ArmBus    6x Feetech ST3215 on /dev/ttyACM1 (CH340). Same exclusivity rule.
            Reports current (register 69) -- the AX-12A does NOT, which is why
            the two servo panels show different fields.

  RosBridge Thin rclpy wrapper living on a background thread. Only the base
            teleop, /odom, /scan and TF come through here.

Nothing in this module is imported by the TUI unless the corresponding hardware
exists, so the panel still starts on a laptop with nothing plugged in.
"""

import glob
import os
import select
import termios
import threading
import time

# --------------------------------------------------------------- AX-12A ---

AX12_INST_PING = 0x01
AX12_INST_READ = 0x02
AX12_INST_WRITE = 0x03

AX12_TORQUE_ENABLE = 24
AX12_GOAL_POSITION = 30
AX12_PRESENT_POSITION = 36
AX12_PRESENT_SPEED = 38
AX12_PRESENT_LOAD = 40
AX12_PRESENT_VOLTAGE = 42
AX12_PRESENT_TEMPERATURE = 43

CENTRE_TICK = 512
# AX-12 spans 300 degrees over 1024 ticks, NOT 360. Using 2*pi/1024 makes every
# commanded angle 20 percent too small.
AX12_RAD_PER_TICK = (300.0 / 1024.0) * 3.141592653589793 / 180.0
AX12_RAD_PER_SEC_UNIT = 0.111 * 2.0 * 3.141592653589793 / 60.0
AX12_NM_PER_LOAD_UNIT = 0.001 * 1.5

# --------------------------------------------------------------- Feetech ---

FT_INST_PING = 0x01
FT_INST_READ = 0x02
FT_INST_WRITE = 0x03
FT_ADDR_TORQUE = 40
FT_ADDR_GOAL_POS = 42
FT_ADDR_GOAL_SPEED = 46
FT_ADDR_PRESENT = 56
FT_TICKS_PER_REV = 4096
FT_TICKS_PER_RAD = FT_TICKS_PER_REV / (2.0 * 3.141592653589793)
FT_CENTRE = 2048
FT_IDS = (1, 2, 3, 4, 5, 6)
FT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_pitch", "wrist_roll", "gripper")
FT_LIMITS = ((-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57),
             (-1.57, 1.57), (-1.57, 1.57), (-1.57, 0.0))


def _checksum(body):
    """Both families: ~(id + length + instruction + params) & 0xFF."""
    return ~sum(body) & 0xFF


def _frame(sid, instr, params=b""):
    body = bytes([sid, 2 + len(params), instr]) + bytes(params)
    return b"\xff\xff" + body + bytes([_checksum(body)])


class _Bus:
    """Raw non-blocking tty with protocol frames. Shared by both servos."""

    def __init__(self, port, baud=1_000_000, timeout=0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.fd = None
        self._buf = b""
        self._lock = threading.Lock()

    def open(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        # Opening a tty does not lock it. Without an explicit exclusive lock the
        # access panel and head_node/arm driver can both write to the same
        # half-duplex bus, corrupting replies (measured: 45% bad reads vs 0%
        # with a single owner). Fail fast instead of silently interfering.
        try:
            import fcntl
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (ImportError, OSError) as exc:
            os.close(self.fd)
            self.fd = None
            raise IOError("%s is already owned by another process (%s)"
                          % (self.port, exc))
        try:
            a = termios.tcgetattr(self.fd)
            speed = getattr(termios, "B%d" % self.baud)
            a[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.CRTSCTS)
            a[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
            a[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                      | termios.ISTRIP | termios.INLCR | termios.IGNCR
                      | termios.ICRNL | termios.IXON | termios.IXOFF)
            a[1] &= ~termios.OPOST
            a[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
                      | termios.ISIG | termios.IEXTEN)
            a[4] = speed
            a[5] = speed
            a[6][termios.VMIN] = 0
            a[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
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

    def _send(self, sid, instr, params=b""):
        pkt = _frame(sid, instr, params)
        view = memoryview(pkt)
        deadline = time.monotonic() + 0.25
        while view:
            try:
                view = view[os.write(self.fd, view):]
            except BlockingIOError:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise IOError("write timeout")
                select.select([], [self.fd], [], left)

    def _recv(self, sid, deadline):
        while time.monotonic() < deadline:
            i = self._buf.find(b"\xff\xff")
            if i >= 0 and len(self._buf) >= i + 4:
                length = self._buf[i + 3]
                total = i + 4 + length
                if len(self._buf) >= total:
                    frame = self._buf[i:total]
                    self._buf = self._buf[total:]
                    if frame[2] == sid:
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

    def txrx(self, sid, instr, params, expect):
        with self._lock:
            if self.fd is None:
                return None
            try:
                self._send(sid, instr, params)
            except (OSError, IOError):
                return None
            frame = self._recv(sid, time.monotonic() + self.timeout)
        if frame is None:
            return None
        return frame[5:5 + expect] if expect else b""

    def read(self, sid, addr, length):
        return self.txrx(sid, FT_INST_READ, bytes([addr, length]), length)

    def write(self, sid, addr, data):
        return self.txrx(sid, FT_INST_WRITE, bytes([addr]) + bytes(data), 0)

    def ping(self, sid):
        return self.txrx(sid, FT_INST_PING, b"", 0) is not None


def port_owner(path):
    """pid holding a tty, or 0. Uses fuser then a /proc scan."""
    if not os.path.exists(path):
        return 0
    try:
        import subprocess
        r = subprocess.run(["fuser", path], capture_output=True, text=True, timeout=3)
        for tok in (r.stderr + r.stdout).replace(":", " ").split():
            if tok.isdigit():
                return int(tok)
    except Exception:                                # noqa: BLE001
        pass
    target = os.path.realpath(path)
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = fd_dir.split("/")[2]
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.readlink(os.path.join(fd_dir, fd)) == target:
                        return int(pid)
                except OSError:
                    continue
        except OSError:
            continue
    return 0


# ------------------------------------------------------------------ head ---

class HeadBus:
    """AX-12A head pan. No current register exists -- see notes."""

    def __init__(self, port="/dev/jupiter/head", dxl_id=6):
        self.port = port
        self.dxl_id = dxl_id
        self.bus = None
        self.error = ""

    def resolve(self):
        """Open the bus, or set self.error explaining why not."""
        self.error = ""
        if not os.path.exists(self.port):
            self.error = "%s does not exist" % self.port
            return False
        pid = port_owner(self.port)
        if pid:
            self.error = "port held by pid %d -- stop that process first" % pid
            return False
        try:
            self.bus = _Bus(self.port).open()
        except OSError as exc:
            self.error = "cannot open %s: %s" % (self.port, exc)
            return False
        if not self.bus.ping(self.dxl_id):
            self.error = "no AX-12 id %d answered" % self.dxl_id
            self.bus.close()
            self.bus = None
            return False
        return True

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def telemetry(self):
        """One snapshot. Returns dict or None. No 'current' key: unsupported.

        Uses ONE block read of registers 36..43 instead of five separate ones.
        Each transaction costs a full FTDI latency_timer tick (~16 ms), so five
        reads cost ~80 ms and this method blocked the TUI for that long on every
        refresh. One block read costs ~16 ms.
        """
        if self.bus is None:
            return None
        raw = self.bus.read(self.dxl_id, AX12_PRESENT_POSITION, 8)
        if raw is None or len(raw) < 8:
            return None

        tick = raw[0] | (raw[1] << 8)
        out = {"tick": tick, "rad": (tick - CENTRE_TICK) * AX12_RAD_PER_TICK}

        sp = raw[2] | (raw[3] << 8)
        mag = sp & 0x03FF
        out["speed"] = (-mag if sp & 0x0400 else mag) * AX12_RAD_PER_SEC_UNIT

        ld = raw[4] | (raw[5] << 8)
        mag = ld & 0x03FF
        out["load"] = (-mag if ld & 0x0400 else mag) * AX12_NM_PER_LOAD_UNIT

        out["voltage"] = raw[6] / 10.0
        out["temp"] = raw[7]

        # Torque lives at register 24, not in the 36..43 block, so it still needs
        # its own transaction -- but it changes rarely, so cache it.
        self._tq_cycle = getattr(self, "_tq_cycle", 0) + 1
        if self._tq_cycle % 10 == 1 or getattr(self, "_tq_cached", None) is None:
            tq = self.bus.read(self.dxl_id, AX12_TORQUE_ENABLE, 1)
            if tq:
                self._tq_cached = bool(tq[0])
        if getattr(self, "_tq_cached", None) is not None:
            out["torque"] = self._tq_cached
        return out

    def set_torque(self, on):
        if self.bus is None:
            return False
        return self.bus.write(self.dxl_id, AX12_TORQUE_ENABLE, [1 if on else 0]) is not None

    def goto_rad(self, rad):
        if self.bus is None:
            return False
        lo, hi = -1.5, 1.5
        rad = max(lo, min(hi, float(rad)))
        tick = CENTRE_TICK + int(round(rad / AX12_RAD_PER_TICK))
        tick = max(0, min(1023, tick))
        data = [tick & 0xFF, (tick >> 8) & 0xFF]
        return self.bus.write(self.dxl_id, AX12_GOAL_POSITION, data) is not None


# ------------------------------------------------------------------- arm ---

class ArmBus:
    """6x Feetech ST3215. Reports current, unlike the AX-12A."""

    def __init__(self, port="/dev/jupiter/arm"):
        self.port = port
        self.bus = None
        self.error = ""
        self.present = []
        # Per-servo travel limits in rad, read from the servo's own min/max
        # angle registers (9 / 11). The hardcoded FT_LIMITS tuple is WRONG for
        # this arm: measured, id3 allows -2.988..0.000 rad, not -1.57..1.57.
        # Commanding outside the real range makes the servo silently stop at
        # its limit, which looks like "the command was ignored".
        self.limits = {}

    def _read_limits(self, sid):
        """Read the servo's own min/max angle registers, in rad."""
        mn = self.bus.read(sid, 9, 2)
        mx = self.bus.read(sid, 11, 2)
        if not mn or not mx or len(mn) < 2 or len(mx) < 2:
            return None
        mn_t = mn[0] | (mn[1] << 8)
        mx_t = mx[0] | (mx[1] << 8)
        if mn_t >= mx_t:
            return None
        return ((mn_t - FT_CENTRE) / FT_TICKS_PER_RAD,
                (mx_t - FT_CENTRE) / FT_TICKS_PER_RAD)

    def limits_for(self, sid):
        """Cached (lo, hi) in rad, falling back to the table only if unreadable."""
        if sid not in self.limits:
            got = self._read_limits(sid) if self.bus is not None else None
            self.limits[sid] = got or FT_LIMITS[sid - 1]
        return self.limits[sid]

    def resolve(self):
        self.error = ""
        if not os.path.exists(self.port):
            self.error = "%s does not exist" % self.port
            return False
        pid = port_owner(self.port)
        if pid:
            self.error = "port held by pid %d -- stop that process first" % pid
            return False
        try:
            self.bus = _Bus(self.port).open()
        except OSError as exc:
            self.error = "cannot open %s: %s" % (self.port, exc)
            return False
        self.present = [i for i in FT_IDS if self.bus.ping(i)]
        if not self.present:
            self.error = "no Feetech servo answered"
            self.bus.close()
            self.bus = None
            return False
        return True

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    @staticmethod
    def _s16(data, off):
        v = data[off] | (data[off + 1] << 8)
        return v - 65536 if v > 32767 else v

    @staticmethod
    def _sign_mag(data, off):
        raw = data[off] | (data[off + 1] << 8)
        mag = raw & 0x7FFF
        return -mag if raw & 0x8000 else mag

    def telemetry(self):
        """List of per-servo dicts, in FT_IDS order."""
        if self.bus is None:
            return []
        out = []
        for sid in FT_IDS:
            raw = self.bus.read(sid, FT_ADDR_PRESENT, 15)
            if not raw or len(raw) < 15:
                out.append({"id": sid, "ok": False})
                continue
            pos = raw[0] | (raw[1] << 8)
            out.append({
                "id": sid,
                "ok": True,
                "name": FT_NAMES[sid - 1],
                "pos": pos,
                "rad": (pos - FT_CENTRE) / FT_TICKS_PER_RAD,
                "speed": self._sign_mag(raw, 2),
                "load": self._sign_mag(raw, 4),
                "volt": raw[6] / 10.0,
                "temp": raw[7],
                "current": self._s16(raw, 13),
            })
        return out

    def set_torque(self, sid, on):
        if self.bus is None:
            return False
        return self.bus.write(sid, FT_ADDR_TORQUE, [1 if on else 0]) is not None

    def goto_rad(self, sid, rad):
        if self.bus is None:
            return False
        # Use the servo's OWN limits, not the FT_LIMITS table. Commanding a
        # value outside the hardware range makes the servo stop at its limit
        # without reporting an error, which reads as "the command did nothing".
        lo, hi = self.limits_for(sid)
        rad = max(lo, min(hi, float(rad)))
        tick = int(round(FT_CENTRE + rad * FT_TICKS_PER_RAD))
        data = [tick & 0xFF, (tick >> 8) & 0xFF]
        return self.bus.write(sid, FT_ADDR_GOAL_POS, data) is not None


# ------------------------------------------------------------ rviz config ---

def rviz_config_path(share_dir):
    """Path to the ROS 2 RViz config shipped in jupiterobot2_description."""
    return os.path.join(share_dir, "rviz", "jupiter_access.rviz")
