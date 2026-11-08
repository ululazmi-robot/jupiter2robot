"""Manifest loading, validation and the probe primitives.

Ported from the Jupitron access layer. This module is deliberately
dependency-free apart from PyYAML and the stdlib so the TUI can import it
without pulling rclpy in, and so `manifest_check` can run on a machine with no
ROS installed.

Difference from the original: there is no `bridge` backend here. The Jupiter 2
repo runs its own drivers and the monitor only observes native topics, so the
only backends are `native` (an existing ROS 2 driver) and `external` (a tool we
merely watch). The `_is_bridge_process` helper is retained but now recognises
the repo's own driver processes, because a port held by the driver we want is
the healthy case, not an obstruction.
"""

import glob
import os
import re
import shutil
import subprocess
import termios

CATEGORIES = ("body", "arm", "head", "vision", "audio", "tools")
BACKENDS = ("native", "external")

# These mirror the constants in DriverStatus.msg. Duplicated as plain strings so
# this module stays importable without a built message package.
UNKNOWN = "unknown"
ONLINE = "online"
BUSY = "busy"
OFFLINE = "offline"
NO_HW = "no_hw"
ERROR = "error"
SIM = "sim"


# ---------------------------------------------------------------- loading ---

def manifest_path():
    """Locate config/drivers.yaml.

    Prefers the installed share dir so a built workspace is self-consistent,
    then falls back to the source tree for development before the first build.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # installed: <prefix>/lib/python3.10/site-packages/jupiter_monitor_hub/../../share/...
        os.path.join(here, "..", "..", "..", "share", "jupiter_monitor_hub",
                     "config", "drivers.yaml"),
        os.path.join(here, "..", "config", "drivers.yaml"),
        # source tree
        os.path.join(here, "..", "..", "config", "drivers.yaml"),
        os.path.expanduser("~/Jupiter2Robot/src/jupiter_monitor_hub/config/drivers.yaml"),
    ]
    for c in candidates:
        c = os.path.normpath(c)
        if os.path.isfile(c):
            return c
    # last resort: walk up from here looking for config/drivers.yaml
    d = here
    for _ in range(6):
        c = os.path.join(d, "config", "drivers.yaml")
        if os.path.isfile(c):
            return c
        d = os.path.dirname(d)
    raise FileNotFoundError("config/drivers.yaml not found near " + here)


def load(path=None):
    """Return (meta, drivers) with light validation applied."""
    import yaml
    path = path or manifest_path()
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    drivers = doc.get("drivers") or []
    problems = validate(drivers)
    if problems:
        raise ValueError("manifest %s invalid:\n  %s" % (path, "\n  ".join(problems)))
    return {"version": doc.get("version", 0), "path": path}, drivers


def validate(drivers):
    problems = []
    seen = set()
    for i, d in enumerate(drivers):
        key = d.get("key")
        where = "drivers[%d]" % i
        if not key:
            problems.append("%s: missing key" % where)
            continue
        if key in seen:
            problems.append("%s: duplicate key %r" % (where, key))
        seen.add(key)

        cat = d.get("category")
        if cat not in CATEGORIES:
            problems.append("%s (%s): category %r not in %s" % (where, key, cat, CATEGORIES))
        elif not key.startswith(cat + "."):
            problems.append("%s: key %r must start with %r" % (where, key, cat + "."))

        backend = d.get("backend")
        if backend not in BACKENDS:
            problems.append("%s (%s): backend %r not in %s" % (where, key, backend, BACKENDS))

        probe = d.get("probe", "none")
        if not re.match(r"^(port:|usb:|topic:|serial:|alsa:|input:|proc:|none$)", probe):
            problems.append("%s (%s): unknown probe %r" % (where, key, probe))

        for t in d.get("topics") or []:
            if not isinstance(t, list) or len(t) != 3:
                problems.append("%s (%s): topic entry %r must be [name, type, role]"
                                % (where, key, t))

        if backend in ("native",) and d.get("controllable") and not d.get("launch"):
            problems.append("%s (%s): controllable but no launch spec" % (where, key))

        for req in ("real_hardware", "controllable", "requires_confirm"):
            if req not in d:
                problems.append("%s (%s): missing %s" % (where, key, req))
    return problems


def by_key(drivers):
    return {d["key"]: d for d in drivers}


def grouped(drivers):
    """Ordered dict category -> [drivers], in CATEGORIES order."""
    out = {c: [] for c in CATEGORIES}
    for d in drivers:
        out.setdefault(d["category"], []).append(d)
    return out


# ---------------------------------------------------------------- probing ---

def _port_owner(path):
    """Return pid holding a tty, or 0.

    Scans /proc directly instead of shelling out to `fuser`. Measured on this
    machine: the subprocess costs ~50 ms per call while the /proc walk costs
    ~7 ms. The Overview tab probes every driver each refresh, so that 43 ms per
    port was the single largest cost in the TUI.
    """
    if not os.path.exists(path):
        return 0
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


def _usb_present(vid, pid):
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(dev, "idVendor")) as f:
                if f.read().strip() != vid:
                    continue
            with open(os.path.join(dev, "idProduct")) as f:
                if f.read().strip() == pid:
                    return True
        except OSError:
            continue
    return False


def _serial_by_id(path):
    """Resolve a /dev/ttyACM* to its stable by-id name, if any."""
    real = os.path.realpath(path)
    for link in glob.glob("/dev/serial/by-id/*"):
        if os.path.realpath(link) == real:
            return os.path.basename(link)
    return ""


def _input_present(name):
    for p in glob.glob("/sys/class/input/*/device/name"):
        try:
            with open(p) as f:
                if name.lower() in f.read().strip().lower():
                    return True
        except OSError:
            continue
    return False


def _alsa_present(kind):
    """kind: 'cap' or 'play'. Returns True when an ALSA device of that kind exists."""
    if not shutil.which("arecord" if kind == "cap" else "aplay"):
        return False
    cmd = ["arecord", "-l"] if kind == "cap" else ["aplay", "-l"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return "card " in r.stdout
    except (OSError, subprocess.SubprocessError):
        return False


def _proc_matches(pattern):
    """True when any process cmdline matches the regex. Returns (bool, pid)."""
    rx = re.compile(pattern)
    me = str(os.getpid())
    for p in glob.glob("/proc/[0-9]*/cmdline"):
        pid = p.split("/")[2]
        if pid == me:
            continue
        try:
            with open(p, "rb") as f:
                cmd = f.read().replace(b"\x00", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        if rx.search(cmd):
            return True, int(pid)
    return False, 0


def probe_port(path):
    if not os.path.exists(path):
        return OFFLINE, "no %s (device absent)" % path, 0
    pid = _port_owner(path)
    if pid:
        return BUSY, "held by pid %d" % pid, pid
    return OFFLINE, "present, no process owns it", 0


def probe_usb(vid_pid):
    vid, _, pid = vid_pid.partition(":")
    if _usb_present(vid, pid):
        return ONLINE, "usb %s:%s enumerates" % (vid, pid), 0
    return NO_HW, "usb %s:%s not present" % (vid, pid), 0


def probe_input(name):
    if _input_present(name):
        return ONLINE, "input device present", 0
    return NO_HW, "no input device matching %r" % name, 0


def probe_alsa(kind):
    if _alsa_present(kind):
        return ONLINE, "ALSA %s device present" % ("capture" if kind == "cap" else "playback"), 0
    return NO_HW, "no ALSA %s device" % kind, 0


def probe_proc(pattern):
    hit, pid = _proc_matches(pattern)
    if hit:
        return ONLINE, "process pid %d" % pid, pid
    return OFFLINE, "no matching process", 0


def probe_topic(name, known_topics=None):
    """Topic liveness needs the ROS graph, which a plain probe cannot see.

    The hub overrides this with a graph-aware check; standalone callers get a
    conservative answer so nothing claims ONLINE without evidence.
    """
    if known_topics is not None and name in known_topics:
        return ONLINE, "topic advertised", 0
    return UNKNOWN, "needs ROS graph (hub supplies this)", 0


def probe_serial(proto, candidates=None):
    """Protocol-level ping. Returns (state, detail, pid).

    Candidate pools are separated by ADAPTER TYPE, not just tried in order.
    This matters: the Feetech and AX-12 ping frames are nearly identical
    (both FF FF <id> <len> 01 <cksum>), so an AX-12 at ID 6 will happily answer
    a Feetech ping. Probing ttyUSB* for Feetech therefore reports the head as
    an arm.

      Feetech ST3215 arm  -> CH340, CDC-ACM  -> /dev/ttyACM*
      Dynamixel AX-12A    -> FTDI FT232H     -> /dev/ttyUSB*

    Both are then confirmed by reading the model-number register, so a wrong
    answer cannot be mistaken for a right one.
    """
    from .serial_probe import ping_feetech, ping_ax12

    if candidates is None:
        # NOTE: on this chassis the FTDI FT232H (AX-12 head) is ttyUSB1 and the
        # CP2102 (RPLidar) is ttyUSB0. They are NOT fixed across reboots; the
        # probe therefore tries both and the adapter VID check rejects the wrong
        # one, so a pass on either is trustworthy.
        candidates = (("/dev/ttyACM1", "/dev/ttyACM0") if proto == "feetech"
                      else ("/dev/ttyUSB1", "/dev/ttyUSB0"))

    # Remember why each candidate failed. "No servo answered" is misleading when
    # the real reason is that another process holds the port -- and so is
    # "close that process" when the holder IS the driver we want running.
    held_pid = 0
    held_detail = ""
    for path in candidates:
        if not os.path.exists(path):
            continue
        pid = _port_owner(path)
        if pid:
            held_pid = pid
            if _is_bridge_process(pid):
                held_detail = ("%s is held by pid %d (a Jupiter driver) -- "
                               "driver running, check its topics" % (path, pid))
            else:
                held_detail = ("%s is held by pid %d -- close that process to "
                               "probe the %s bus" % (path, pid, proto))
        try:
            if proto == "feetech":
                ok, detail = ping_feetech(path)
            elif proto == "ax12":
                ok, detail = ping_ax12(path)
            else:
                return UNKNOWN, "unknown serial proto %r" % proto, 0
        except Exception as exc:                    # noqa: BLE001 - probe must never raise
            ok, detail = False, "%s: %s" % (type(exc).__name__, exc)
        if ok:
            return ONLINE, "%s %s" % (path, detail), pid

    if held_detail:
        # Return the holder's pid: a caller can then tell "our driver" from
        # "something else" without re-walking /proc.
        return BUSY, held_detail, held_pid
    return OFFLINE, "no %s servo answered" % proto, 0


# Processes that legitimately hold a serial/USB device in THIS repo. A held port
# is the intended state when the driver we WANT is the holder -- reporting "busy"
# for a perfectly healthy running driver is the single most misleading thing this
# probe can do. Keep the list explicit; an empty string would match everything.
DRIVER_EXES = (
    # servos
    "dynamixel_hardware_interface",
    "dynamixel_manager",
    "controller_manager.py",
    "controller_spawner.py",
    "ros2_control_node",
    "joint_state_broadcaster",
    "head_node",
    "feetech_bridge",
    "ax12_bridge",
    # sensors and base
    "rplidar_node",
    "sllidar_node",
    "astra_camera_node",
    "turtlebot3_ros",
)


def _is_bridge_process(pid):
    """True when `pid` is one of the robot's own drivers.

    Without this, a running driver makes the probe report the port as an
    obstruction, when it is in fact the intended occupier.
    """
    try:
        with open("/proc/%d/cmdline" % int(pid), "rb") as f:
            cmd = f.read().replace(b"\x00", b" ").decode("utf-8", "replace")
    except (OSError, ValueError):
        return False
    return any(exe in cmd for exe in DRIVER_EXES)


def run_probe(spec, known_topics=None):
    """Dispatch a manifest probe string. Never raises."""
    try:
        if spec == "none":
            return UNKNOWN, "no probe defined", 0
        kind, _, arg = spec.partition(":")
        if kind == "port":
            return probe_port(arg)
        if kind == "usb":
            return probe_usb(arg)
        if kind == "input":
            return probe_input(arg)
        if kind == "alsa":
            return probe_alsa(arg)
        if kind == "proc":
            return probe_proc(arg)
        if kind == "topic":
            return probe_topic(arg, known_topics)
        if kind == "serial":
            return probe_serial(arg)
        return UNKNOWN, "unhandled probe %r" % spec, 0
    except Exception as exc:                        # noqa: BLE001
        return ERROR, "probe failed: %s" % exc, 0


# ------------------------------------------------------------------- misc ---

def tty_baud_supported(baud):
    return hasattr(termios, "B%d" % baud)
