"""Managed `ros2 launch` children, so the panel replaces typing commands.

Each module is a spec: what to launch, what it needs first, and what it holds
exclusively. Two rules matter:

1. EXCLUSIVE PORTS. head_node and the panel's own servo view both want
   /dev/ttyUSB1. Starting head_node while the panel holds the bus would corrupt
   frames from two writers, so launch() refuses if a sibling already owns the
   port, and says which one.

2. DEPENDENCIES. Starting vision without the camera, or nav2 without the lidar,
   is not an error but it is useless. `requires` is advisory -- the panel warms
   rather than blocks, because you may have started the dependency by hand.

Processes are killed by process GROUP, not by pid: `ros2 launch` spawns children
of its own, and killing only the parent orphans gzserver/rviz2 -- which then keep
holding ports and topics, and the next start fails for no visible reason.
"""

import os
import signal
import subprocess
import threading
import time


class Module:
    """One launchable subsystem."""

    def __init__(self, key, label, package, launch_file, args=None,
                 holds=(), requires=(), note=""):
        self.key = key
        self.label = label
        self.package = package
        self.launch_file = launch_file
        self.args = list(args or [])
        self.holds = tuple(holds)
        self.requires = tuple(requires)
        self.note = note

        self.proc = None
        self.started_at = 0.0
        self.log = []                     # rolling lines
        self.state = "stopped"            # stopped | running | failed
        self.exit_code = None
        self._reader = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------- command
    def argv(self):
        return ["ros2", "launch", self.package, self.launch_file] + self.args

    def command_text(self):
        return " ".join(self.argv())

    # -------------------------------------------------------------- status
    def is_running(self):
        p = self.proc
        return p is not None and p.poll() is None

    def uptime(self):
        return time.time() - self.started_at if self.started_at else 0.0

    def append(self, line):
        with self._lock:
            self.log.append(line.rstrip("\n"))
            if len(self.log) > 400:
                self.log = self.log[-400:]

    def log_tail(self, n=200):
        with self._lock:
            return list(self.log[-n:])


# The catalogue. Order is launch order in the overview.
MODULES = [
    Module("base", "Base (OpenCR)", "jupiterobot2_bringup",
           "jupiterobot2_core.launch.py",
           holds=("/dev/ttyACM0",),
           note="Wheel PID lives in firmware. Needs /dev/ttyACM0."),

    Module("lidar", "RPLidar", "jupiterobot2_bringup",
           "jupiterobot2_lidar.launch.py",
           holds=("/dev/ttyUSB0",),
           note="Port auto-resolved by CP2102 VID:PID."),

    Module("camera", "Astra camera", "astra_camera",
           "astra.launch.xml", args=["product_id:=0x065b"],
           holds=("usb 2bc5:065b",),
           note="Depth + colour. IR disabled while colour is on."),

    Module("head", "Head pan (AX-12A)", "jupiterobot2_head_bringup",
           "head.launch.py",
           holds=("/dev/ttyUSB1",),
           note="Torque starts OFF. Enable from the Servo tab."),

    Module("vision", "Vision pipeline", "jupiterobot2_vision",
           "", requires=("camera",),
           note="No launch file in this repo -- run the node directly."),

    Module("monitor_hub", "Monitor hub", "jupiter_monitor_hub",
           "monitor.launch.py",
           note="Publishes /robot/health and /robot/state."),

    Module("nav2", "Nav2 navigation", "jupiterobot2_navigation",
           "navigation.launch.py", requires=("lidar", "base"),
           note="Check use_sim_time before running on hardware."),

    Module("slam", "SLAM (slam_toolbox)", "jupiterobot2_gazebo",
           "slam.launch.py", requires=("lidar",),
           note="Use the no-odom mapper params when wheels drift."),

    Module("gazebo", "Gazebo simulation", "jupiterobot2_gazebo",
           "gazebo.launch.py",
           note="Simulation replaces hardware -- do not run together."),
]

BY_KEY = {m.key: m for m in MODULES}


class Launcher:
    """Owns the child processes. One instance, shared by the TUI."""

    def __init__(self):
        self.modules = MODULES
        self._lock = threading.Lock()

    # ------------------------------------------------------------- queries
    def port_conflict(self, module):
        """Return (other_module, port) when a sibling already holds a device."""
        for port in module.holds:
            for other in self.modules:
                if other is module or not other.is_running():
                    continue
                if port in other.holds:
                    return other, port
        return None, None

    def missing_requirements(self, module):
        return [k for k in module.requires
                if k in BY_KEY and not BY_KEY[k].is_running()]

    # -------------------------------------------------------------- control
    def start(self, key, extra_args=None):
        """Launch one module. Returns (ok, message)."""
        m = BY_KEY.get(key)
        if m is None:
            return False, "unknown module %r" % key
        if m.is_running():
            return False, "%s already running (pid %d)" % (m.label, m.proc.pid)
        if not m.launch_file:
            return False, ("%s has no launch file in this repo; run it with "
                           "'ros2 run %s <node>'" % (m.label, m.package))

        other, port = self.port_conflict(m)
        if other is not None:
            return False, ("%s already holds %s -- stop it first"
                           % (other.label, port))

        argv = m.argv() + list(extra_args or [])

        with self._lock:
            m.log = []
            m.exit_code = None
            try:
                # start_new_session=True gives the child its own process group,
                # which is what makes stop() able to kill its whole tree.
                m.proc = subprocess.Popen(
                    argv,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                    cwd=os.path.expanduser("~"),
                )
            except FileNotFoundError:
                m.state = "failed"
                return False, "ros2 not found on PATH"
            except OSError as exc:
                m.state = "failed"
                return False, "cannot start: %s" % exc

            m.started_at = time.time()
            m.state = "running"
            m._reader = threading.Thread(
                target=self._pump, args=(m,), daemon=True)
            m._reader.start()

        warn = [k for k in self.missing_requirements(m)]
        msg = "started %s (pid %d)" % (m.label, m.proc.pid)
        if warn:
            msg += "; warning: %s not running" % ", ".join(warn)
        return True, msg

    def _pump(self, module):
        """Read child stdout into the rolling log until it exits."""
        proc = module.proc
        try:
            for line in proc.stdout:
                module.append(line)
        except (ValueError, OSError):
            pass
        finally:
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
            code = proc.poll()
            module.exit_code = code
            module.state = "stopped" if code == 0 else "failed"
            module.append("[panel] process exited with code %s" % code)

    def stop(self, key, timeout=6.0):
        """Stop a module and its whole process tree."""
        m = BY_KEY.get(key)
        if m is None:
            return False, "unknown module %r" % key
        if not m.is_running():
            m.state = "stopped"
            return False, "%s is not running" % m.label

        try:
            os.killpg(os.getpgid(m.proc.pid), signal.SIGINT)
        except (ProcessLookupError, OSError):
            pass

        deadline = time.time() + timeout
        while time.time() < deadline and m.proc.poll() is None:
            time.sleep(0.1)

        if m.proc.poll() is None:
            # Still alive: `ros2 launch` sometimes ignores SIGINT while a
            # gzserver/rviz2 child is wedged.
            try:
                os.killpg(os.getpgid(m.proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
            m.append("[panel] SIGKILL after %.0fs" % timeout)
        m.state = "stopped"
        return True, "stopped %s" % m.label

    def restart(self, key):
        m = BY_KEY.get(key)
        if m is None:
            return False, "unknown module %r" % key
        if m.is_running():
            self.stop(key)
            time.sleep(0.5)
        return self.start(key)

    def stop_all(self):
        for m in self.modules:
            if m.is_running():
                try:
                    self.stop(m.key)
                except Exception:                    # noqa: BLE001
                    pass
