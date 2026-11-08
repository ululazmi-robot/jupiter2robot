"""Protocol-level liveness pings.

Two servo families, two frame formats. Getting either wrong returns "no device"
against hardware that is working perfectly, so both are spelled out here with
their checksum rules stated explicitly.

    Feetech ST3215 (SO-100 arm)
        FF FF <id> <len> <instr> <params...> <checksum>
        checksum = ~(sum(id .. last param)) & 0xFF      <- sum STARTS at id

    Dynamixel AX-12A protocol 1.0 (head)
        FF FF <id> <len> <instr> <params...> <checksum>
        checksum = ~(sum(instr .. last param)) & 0xFF   <- sum starts at instr

That one-byte difference is why a hand-rolled AX-12 ping returns hardware error
0x10 (instruction error) and looks like silence.

SAFETY: these functions refuse to open a port that another process holds. Two
writers on a half-duplex servo bus corrupt frames and can move a joint.
"""

import os
import select
import termios
import time

FEETECH_INST_PING = 0x01
AX12_INST_PING = 0x01

FEETECH_BAUD = 1_000_000
AX12_BAUD = 1_000_000

FEETECH_IDS = (1, 2, 3, 4, 5, 6)
AX12_ID = 6


# ------------------------------------------------------------------ helpers ---

def _open_raw(path, baud):
    """Open a tty raw at `baud`, non-blocking. Caller must close."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        speed = getattr(termios, "B%d" % baud)
        # cflag: 8N1, no parity, no flow control, receiver on
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE
                      | termios.CRTSCTS)
        attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        # iflag: raw, no translation
        attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                      | termios.ISTRIP | termios.INLCR | termios.IGNCR
                      | termios.ICRNL | termios.IXON | termios.IXOFF)
        # oflag: raw
        attrs[1] &= ~termios.OPOST
        # lflag: no echo, no canonical, no signals
        attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
                      | termios.ISIG | termios.IEXTEN)
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def _port_is_held(path):
    """True when some other process has the tty open."""
    import glob
    target = os.path.realpath(path)
    me = str(os.getpid())
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = fd_dir.split("/")[2]
        if pid == me:
            continue
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.readlink(os.path.join(fd_dir, fd)) == target:
                        return True
                except OSError:
                    continue
        except OSError:
            continue
    return False


def usb_ids_for_tty(path):
    """Return (vid, pid) of the USB device behind a tty, or (None, None).

    Walks /sys/class/tty/<name>/device/.. up to the USB interface and reads
    idVendor/idProduct. This is how we tell a CH340 from an FTDI without
    trusting the device node name, which changes across reboots.
    """
    import glob
    name = os.path.basename(os.path.realpath(path))
    base = os.path.join("/sys/class/tty", name, "device")
    if not os.path.exists(base):
        return None, None
    cur = os.path.realpath(base)
    for _ in range(6):
        vid_p = os.path.join(cur, "idVendor")
        pid_p = os.path.join(cur, "idProduct")
        if os.path.exists(vid_p) and os.path.exists(pid_p):
            try:
                with open(vid_p) as f:
                    vid = f.read().strip()
                with open(pid_p) as f:
                    pid = f.read().strip()
                return vid, pid
            except OSError:
                return None, None
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return None, None


# Which USB adapters can legitimately host which protocol. Getting this wrong
# is how a Feetech ping "succeeds" against an AX-12: the frames are nearly
# identical and both servos answer ID 6.
ADAPTER_ALLOWED = {
    "feetech": ("1a86",),    # CH340 family (QinHeng)
    "ax12":    ("0403",),    # FTDI FT232H
}


def _adapter_matches(path, proto):
    vid, pid = usb_ids_for_tty(path)
    if vid is None:
        return True          # cannot tell (e.g. a real UART); allow
    allowed = ADAPTER_ALLOWED.get(proto)
    if not allowed:
        return True
    return vid in allowed


def _write_all(fd, data, timeout=0.25):
    view = memoryview(data)
    deadline = time.monotonic() + timeout
    while view:
        try:
            view = view[os.write(fd, view):]
        except BlockingIOError:
            left = deadline - time.monotonic()
            if left <= 0:
                return False
            select.select([], [fd], [], left)
    return True


def _scan_reply(fd, expect_id, deadline):
    """Read until a well-formed status packet for expect_id appears.

    Returns the frame bytes, or None. NOTE: a plain function, not a generator --
    an earlier version used `yield` and callers silently got a generator object,
    which is truthy and has no indices.
    """
    buf = b""
    while time.monotonic() < deadline:
        i = buf.find(b"\xff\xff")
        while i >= 0 and len(buf) >= i + 5:
            sid = buf[i + 2]
            length = buf[i + 3]
            total = i + 4 + length
            if len(buf) >= total:
                frame = buf[i:total]
                if sid == expect_id:
                    return bytes(frame)
                buf = buf[total:]
                i = buf.find(b"\xff\xff")
                continue
            break

        left = deadline - time.monotonic()
        if left <= 0:
            break
        r, _, _ = select.select([fd], [], [], left)
        if not r:
            continue
        try:
            chunk = os.read(fd, 256)
        except BlockingIOError:
            continue
        except OSError:
            return None
        if chunk:
            buf += chunk
    return None


# ----------------------------------------------------------------- feetech ---

def feetech_checksum(body):
    """body = bytes from id through last param."""
    return ~sum(body) & 0xFF


def feetech_ping_frame(servo_id):
    body = bytes([servo_id, 0x02, FEETECH_INST_PING])
    return b"\xff\xff" + body + bytes([feetech_checksum(body)])


def ping_feetech(path, servo_ids=FEETECH_IDS, timeout=0.05):
    """Return (ok, detail). Succeeds when ANY id answers.

    Refuses a port whose USB adapter is not a CH340-family device: a Feetech
    ping frame is close enough to an AX-12 ping that an FTDI-hosted Dynamixel
    will answer, and reporting the head as an arm is worse than reporting
    nothing.
    """
    if not _adapter_matches(path, "feetech"):
        vid, pid = usb_ids_for_tty(path)
        return False, ("%s is %s:%s, not a CH340 arm adapter"
                       % (path, vid or "?", pid or "?"))
    if _port_is_held(path):
        return False, "%s is held by another process (refusing to write)" % path
    fd = None
    try:
        fd = _open_raw(path, FEETECH_BAUD)
        for sid in servo_ids:
            if not _write_all(fd, feetech_ping_frame(sid)):
                return False, "serial write failed on %s" % path
            reply = _scan_reply(fd, sid, time.monotonic() + timeout)
            if reply is not None:
                err = reply[4] if len(reply) > 4 else 0
                if err == 0:
                    return True, "feetech id %d ok" % sid
                return True, "feetech id %d replied with err 0x%02x" % (sid, err)
        return False, "no feetech id answered on %s" % path
    except PermissionError:
        return False, "permission denied on %s" % path
    except OSError as exc:
        return False, "cannot open %s: %s" % (path, exc)
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass


# -------------------------------------------------------------------- AX12 ---

def ax12_checksum(body):
    """Protocol 1.0: ~(ID + LENGTH + INSTRUCTION + params) & 0xFF.

    Identical in form to Feetech's checksum -- the id IS included. Getting this
    wrong (summing only instr+params) makes every packet fail with hardware
    error 0x10, which looks exactly like a missing servo.
    """
    return ~sum(body) & 0xFF


def ax12_ping_frame(dxl_id):
    length = 2                      # instr + checksum
    body = bytes([dxl_id, length, AX12_INST_PING])
    return b"\xff\xff" + body + bytes([ax12_checksum(body)])


def build(dxl_id, instr, params=b""):
    """Generic protocol-1.0 request frame."""
    body = bytes([dxl_id, 2 + len(params), instr]) + bytes(params)
    return b"\xff\xff" + body + bytes([ax12_checksum(body)])


def ping_ax12(path, dxl_id=AX12_ID, timeout=0.05):
    """Return (ok, detail)."""
    if not _adapter_matches(path, "ax12"):
        vid, pid = usb_ids_for_tty(path)
        return False, ("%s is %s:%s, not an FTDI head adapter"
                       % (path, vid or "?", pid or "?"))
    if _port_is_held(path):
        return False, "%s is held by another process (refusing to write)" % path
    fd = None
    try:
        fd = _open_raw(path, AX12_BAUD)
        if not _write_all(fd, ax12_ping_frame(dxl_id)):
            return False, "serial write failed on %s" % path
        reply = _scan_reply(fd, dxl_id, time.monotonic() + timeout)
        if reply is None:
            return False, "no AX-12 id %d answered on %s" % (dxl_id, path)
        # status frame: FF FF id len err params...
        err = reply[4] if len(reply) > 4 else 0
        if err != 0:
            return True, "ax12 id %d replied with hw error 0x%02x" % (dxl_id, err)
        return True, "ax12 id %d ok" % dxl_id
    except PermissionError:
        return False, "permission denied on %s" % path
    except OSError as exc:
        return False, "cannot open %s: %s" % (path, exc)
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass


def ax12_read_telemetry(path, dxl_id=AX12_ID):
    """Best-effort position/voltage/temperature. Returns dict or None.

    Registers (protocol 1.0): 36 present position (2), 42 voltage (1),
    43 temperature (1). Read as two transactions to keep frames short.
    """
    if _port_is_held(path):
        return None
    fd = None
    try:
        fd = _open_raw(path, AX12_BAUD)

        def read(addr, length):
            params = bytes([addr, length])
            frame = build(dxl_id, 0x02, params)
            if not _write_all(fd, frame):
                return None
            return _scan_reply(fd, dxl_id, time.monotonic() + 0.05)

        out = {}
        pos = read(36, 2)
        if pos is not None and len(pos) >= 7:
            out["position_tick"] = pos[5] | (pos[6] << 8)
        volt = read(42, 1)
        if volt is not None and len(volt) >= 6:
            out["voltage"] = volt[5] / 10.0
        temp = read(43, 1)
        if temp is not None and len(temp) >= 6:
            out["temperature"] = temp[5]
        return out or None
    except OSError:
        return None
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
