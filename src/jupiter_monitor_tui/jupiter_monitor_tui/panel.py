"""Jupiter 2 access panel: monitor + per-subsystem control.

Seven tabs, each owning one subsystem end to end. The design rule throughout:
the panel must start and stay usable on a laptop with nothing plugged in, so
every tab degrades to "what should be here and why it isn't" rather than
crashing or showing an empty box.

    Overview  driver health, all 14 entries      (reads the hub or probes)
    Servo     head AX-12A: pos/speed/load/V/temp + jog + torque
    Arm       6x Feetech: pos/load/current/temp grid + jog
    Base      drive with w/a/s/d, live odom, speed limits
    Lidar     scan stats + V to open RViz
    Vision    camera/depth/cloud stats + V to open RViz
    Process   start/stop/restart each ros2 launch with captured logs

Two hardware facts this panel is built around, both verified on the robot:

* The AX-12A has NO current register. It reports load, voltage and temperature.
  Any "current" field on the Servo tab would be fabricated, so there isn't one.
* The Feetech ST3215 DOES report current (register 69), so the Arm tab shows it.
  The two tabs deliberately show different fields.

Servo buses are exclusive: if head_node is running, the Servo tab cannot open
/dev/ttyUSB1. It says which pid holds the port instead of fighting for it.
"""

import argparse
import os
import subprocess
import sys
import time

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import (DataTable, Footer, Header, Input, RichLog, Static,
                             TabbedContent, TabPane)

try:
    import jupiter_monitor_hub.manifest as mf
except ImportError:                                  # pragma: no cover
    mf = None

from jupiter_monitor_tui import hw
from jupiter_monitor_tui.launcher import Launcher, BY_KEY
from jupiter_monitor_tui.roskey import (RosBridge, clamp_lin, clamp_ang,
                                        MAX_LIN_VEL, MAX_ANG_VEL,
                                        LIN_STEP, ANG_STEP)

STATE_STYLE = {
    "online":  ("bold green",   "●"),
    "busy":    ("bold yellow",  "◐"),
    "offline": ("dim",          "○"),
    "no_hw":   ("dim red",      "✕"),
    "error":   ("bold red",     "!"),
    "sim":     ("bold cyan",    "◆"),
    "unknown": ("dim",          "?"),
}

CATEGORY_TITLE = {
    "body":   "BODY   -- drive, odometry, lidar",
    "arm":    "ARM    -- SO-100 6-servo manipulator",
    "head":   "HEAD   -- camera pan (AX-12A)",
    "vision": "VISION -- depth camera, perception pipeline",
    "audio":  "AUDIO  -- microphone, speaker, voice nodes",
    "tools":  "TOOLS  -- teleop, nav, SLAM, simulation",
}

CATEGORY_ORDER = ("body", "arm", "head", "vision", "audio", "tools")


def _fmt(v, unit="", nd=2):
    if v is None:
        return "-"
    return ("%." + str(nd) + "f%s") % (v, unit)


def _bar(value, lo, hi, width=16):
    """Text progress bar, for a value inside a known range."""
    if value is None or hi <= lo:
        return Text("-" * width, style="dim")
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    n = int(round(frac * width))
    t = Text()
    t.append("█" * n, style="cyan")
    t.append("░" * (width - n), style="dim")
    return t


class JupiterPanel(App):
    CSS = """
    Screen { background: $surface; }
    #summary { height: 3; padding: 0 1; background: $boost; }
    DataTable { height: 1fr; }
    .pane { padding: 0 1; }
    .big { text-style: bold; color: $text; }
    .dim { color: $text-muted; }
    #detail { height: 1fr; }
    """

    BINDINGS = [
        Binding("q", "quit", "quit"),
        Binding("j", "cursor_down", "down", show=False),
        Binding("k", "cursor_up", "up", show=False),
        Binding("down", "cursor_down", "down", show=False, priority=True),
        Binding("up", "cursor_up", "up", show=False, priority=True),
        Binding("p", "probe", "probe"),
        Binding("v", "open_rviz", "rviz"),
        Binding("t", "toggle_torque", "torque"),
        Binding("r", "refresh_now", "refresh"),
        Binding("s", "toggle_selected", "start/stop"),
        Binding("c", "centre", "centre"),
        # Angle control is alphabet keys, not punctuation or arrows:
        #   f / b  jog the selected actuator forward / back
        #   F / B  same, larger step
        # Arrows stay reserved for list navigation so they never fight with jog.
        Binding("f", "jog_fwd", "jog +"),
        Binding("b", "jog_back", "jog -"),
        Binding("F", "jog_fwd_big", "+ big", show=False),
        Binding("B", "jog_back_big", "- big", show=False),
        Binding("z", "zero", "zero", show=False),
        Binding("w", "drive_fwd", "fwd", show=False),
        Binding("x", "drive_back", "back", show=False),
        Binding("a", "drive_left", "left", show=False),
        Binding("d", "drive_right", "right", show=False),
        Binding("space", "drive_stop", "stop", show=False),
        Binding("tab", "next_tab", "next tab"),
        Binding("shift+tab", "prev_tab", "prev tab"),
    ]

    def __init__(self, interval=1.0, start_tab="overview"):
        super().__init__()
        self.interval = interval
        self._start_tab = start_tab

        self.rows = []
        self.selected_key = ""
        self._drivers = []
        self._meta = {}
        self._status = ""

        self.launcher = Launcher()
        self.head = hw.HeadBus()
        self.arm = hw.ArmBus()
        self.ros = None

        self._lin = 0.0
        self._ang = 0.0
        self._heading = 0.0          # jog target for head
        self._arm_sel = 1            # selected arm servo id
        self._rviz = None

        self._load_manifest()

    # ------------------------------------------------------------- manifest
    def _load_manifest(self):
        if mf is None:
            self._status = "jupiter_monitor_hub not importable"
            return
        try:
            self._meta, self._drivers = mf.load()
            self.rows = list(self._drivers)
        except Exception as exc:                     # noqa: BLE001
            self._status = "manifest error: %s" % exc

    # ------------------------------------------------------------- compose
    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Static("", id="summary")
        with TabbedContent(initial=self._start_tab):
            with TabPane("Overview", id="overview"):
                yield DataTable(id="t_overview", cursor_type="row", zebra_stripes=True)
            with TabPane("Servo", id="servo"):
                with VerticalScroll(classes="pane"):
                    yield Static("", id="servo_info")
            with TabPane("Arm", id="arm"):
                yield DataTable(id="t_arm", cursor_type="row", zebra_stripes=True)
            with TabPane("Base", id="base"):
                with VerticalScroll(classes="pane"):
                    yield Static("", id="base_info")
            with TabPane("Lidar", id="lidar"):
                with VerticalScroll(classes="pane"):
                    yield Static("", id="lidar_info")
            with TabPane("Vision", id="vision"):
                with VerticalScroll(classes="pane"):
                    yield Static("", id="vision_info")
            with TabPane("Process", id="process"):
                yield DataTable(id="t_process", cursor_type="row", zebra_stripes=True)
            with TabPane("Log", id="log"):
                yield RichLog(id="logs", wrap=True, highlight=False)
        yield Footer()

    def on_mount(self):
        self.title = "Jupiter 2 access panel"
        self.sub_title = "monitor + control"

        ov = self.query_one("#t_overview", DataTable)
        ov.add_column("", width=2)
        ov.add_column("state", width=8)
        ov.add_column("driver", width=18)
        ov.add_column("device", width=22)
        ov.add_column("rate", width=7)
        ov.add_column("primary topic", width=30)

        am = self.query_one("#t_arm", DataTable)
        am.add_column("id", width=3)
        am.add_column("joint", width=16)
        am.add_column("position", width=12)
        am.add_column("load", width=8)
        am.add_column("current", width=9)
        am.add_column("volt", width=7)
        am.add_column("temp", width=6)

        pr = self.query_one("#t_process", DataTable)
        pr.add_column("", width=2)
        pr.add_column("state", width=9)
        pr.add_column("module", width=22)
        pr.add_column("uptime", width=8)
        pr.add_column("ports", width=18)

        # The ROS bridge is only needed for the Base tab, and starting rclpy
        # can block for seconds. Do it off the UI thread.
        import threading
        threading.Thread(target=self._start_ros, daemon=True).start()
        # Open both servo buses in the background so the Servo and Arm tabs have
        # data immediately. Without this the Arm tab showed only "closed" until
        # you happened to press a jog key, which looks like broken hardware.
        threading.Thread(target=self._open_buses, daemon=True).start()

        self._refresh()
        self.set_interval(self.interval, self._refresh)

    def _open_buses(self):
        """Best-effort open of both servo buses, off the UI thread."""
        try:
            if not self.head.resolve():
                self._head_err = self.head.error
        except Exception as exc:                     # noqa: BLE001
            self._head_err = str(exc)
        try:
            if not self.arm.resolve():
                self._arm_err = self.arm.error
        except Exception as exc:                     # noqa: BLE001
            self._arm_err = str(exc)

    def _start_ros(self):
        self.ros = RosBridge()
        self.ros.start()

    def on_unmount(self):
        try:
            if self.ros is not None:
                self.ros.stop_base()
                self.ros.stop()
        except Exception:                            # noqa: BLE001
            pass
        try:
            self.head.close()
            self.arm.close()
        except Exception:                            # noqa: BLE001
            pass
        try:
            if self._rviz is not None and self._rviz.poll() is None:
                self._rviz.terminate()
        except Exception:                            # noqa: BLE001
            pass

    # ------------------------------------------------------------ refresh
    def _refresh(self):
        try:
            self._render_summary()
            self._render_overview()
            self._render_process()
            self._render_servo()
            self._render_arm()
            self._render_base()
            self._render_lidar()
            self._render_vision()
        except Exception as exc:                     # noqa: BLE001
            self._status = "render error: %s" % exc

    def _render_summary(self):
        running = sum(1 for m in self.launcher.modules if m.is_running())
        total = len(self.launcher.modules)
        parts = [("jupiter2 ", "bold cyan"),
                 ("%d/%d modules up" % (running, total),
                  "green" if running else "dim")]
        if self.head.bus is not None:
            parts.append(("  head:open", "green"))
        elif self.head.error:
            parts.append(("  head:closed", "dim"))
        if self.arm.bus is not None:
            parts.append(("  arm:open", "green"))
        src = "hub" if self._hub_up() else "local"
        parts.append(("   [%s]" % src, "dim"))

        t = Text()
        for s, st in parts:
            t.append(s, style=st)
        t.append("\n")
        t.append(self._status or "p probe · v rviz · t torque · s start/stop · "
                 "tab switch · q quit", style="dim" if not self._status else "yellow")
        self.query_one("#summary", Static).update(t)

    def _hub_up(self):
        try:
            return BY_KEY["monitor_hub"].is_running()
        except KeyError:
            return False

    PROBE_TTL = 5.0     # seconds a cached probe result stays valid

    def _local_state(self, d):
        """Probe one manifest entry without the hub, with a short TTL cache.

        A full probe of all 14 drivers costs ~378 ms, dominated by serial pings
        and /proc scans. Running that on every 1 s refresh burned a third of a
        core for information that changes on the order of seconds. Results are
        now cached for PROBE_TTL; press 'p' to force a fresh probe.
        """
        probe = d.get("probe", "none")
        if probe.startswith("serial:"):
            # A serial ping is the slowest thing here and must not run while
            # another process owns the bus.
            if d["category"] == "head" and self.head.bus is not None:
                return "online", "panel owns the bus"
            if d["category"] == "arm" and self.arm.bus is not None:
                return "online", "panel owns the bus"
            return "unknown", "serial probe deferred (press p)"

        now = time.time()
        if not hasattr(self, "_probe_cache"):
            self._probe_cache = {}
        hit = self._probe_cache.get(probe)
        if hit and (now - hit[0]) < self.PROBE_TTL and not getattr(self, "_force_probe", False):
            return hit[1], hit[2]

        try:
            state, detail, _pid = mf.run_probe(probe)
        except Exception as exc:                     # noqa: BLE001
            state, detail = "error", str(exc)
        self._probe_cache[probe] = (now, state, detail)
        return state, detail

    def _render_overview(self):
        table = self.query_one("#t_overview", DataTable)
        rows = []
        cat = None
        for d in self.drivers_or_rows():
            state, detail = (self._local_state(d) if mf is not None
                             else ("unknown", "no manifest"))
            rows.append((d, state, detail))
            if d["category"] != cat:
                cat = d["category"]

        # Rebuild ONLY when the data actually changed. Rebuilding every refresh
        # resets cursor_row to 0, which is what made the highlight jump to the
        # top once a second. The signature deliberately excludes the detail
        # string, which changes constantly and is not visible in this table.
        sig = tuple((r[0]["key"], r[1]) for r in rows)
        if sig == getattr(self, "_ov_sig", None) and table.row_count:
            return
        self._ov_sig = sig

        saved = self._saved_cursor(table)
        table.clear()
        cat = None
        for d, state, detail in rows:
            if d["category"] != cat:
                cat = d["category"]
                table.add_row(Text(""),
                              Text(CATEGORY_TITLE.get(cat, cat.upper()),
                                   style="bold reverse"),
                              Text(""), Text(""), Text(""), Text(""),
                              key="__hdr__" + cat)
            style, glyph = STATE_STYLE.get(state, ("dim", "?"))
            primary = d.get("primary_topic") or "-"
            table.add_row(
                Text(glyph, style=style),
                Text(state, style=style),
                Text(d["key"], style="bold"),
                Text(str(d.get("device", "-")), style="dim"),
                Text("-", style="dim"),
                Text(primary, style="cyan" if primary != "-" else "dim"),
                key=d["key"],
            )
        self._restore_cursor(table, saved)

    def drivers_or_rows(self):
        return self._drivers

    # ------------------------------------------------- cursor preservation
    def _saved_cursor(self, table):
        """Row key under the cursor, so it survives a table rebuild.

        DataTable.clear() resets cursor_row to 0. Tracking the KEY (not the
        index) means the highlight stays on the same logical row even when
        rows above it are added or removed.
        """
        try:
            if table.row_count == 0:
                return None
            row, _col = table.cursor_coordinate
            key = table.coordinate_to_cell_key((row, 0)).row_key
            return key.value if key else None
        except Exception:                            # noqa: BLE001
            return None

    def _restore_cursor(self, table, key):
        if not key:
            return
        try:
            for i in range(table.row_count):
                k = table.coordinate_to_cell_key((i, 0)).row_key
                if k and k.value == key:
                    table.move_cursor(row=i, scroll=False)
                    return
        except Exception:                            # noqa: BLE001
            pass

    def _render_process(self):
        table = self.query_one("#t_process", DataTable)
        rows = []
        for m in self.launcher.modules:
            if m.is_running():
                rows.append((m, "●", "bold green", "running",
                             "%ds" % int(m.uptime())))
            elif m.state == "failed":
                rows.append((m, "!", "bold red", "failed", "-"))
            else:
                rows.append((m, "○", "dim", "stopped", "-"))

        # Signature includes state but NOT uptime: uptime ticks every second and
        # would force a rebuild, which is exactly the cursor-reset bug.
        sig = tuple((r[0].key, r[3]) for r in rows)
        if sig == getattr(self, "_pr_sig", None) and table.row_count:
            return
        self._pr_sig = sig

        saved = self._saved_cursor(table)
        table.clear()
        for m, glyph, style, state, up in rows:
            table.add_row(
                Text(glyph, style=style),
                Text(state, style=style),
                Text(m.key, style="bold"),
                Text(up, style="dim"),
                Text(",".join(m.holds) or "-", style="dim"),
                key=m.key,
            )
        self._restore_cursor(table, saved)

    def _head_source(self):
        """Pick where Servo tab data comes from, preferring the passive path.

        Priority:
          1. /head/telemetry  -- head_node owns the bus and reports everything.
             Zero contention, so this is preferred even if the panel *could*
             open the bus.
          2. Direct bus       -- only when nobody else owns it and head_node is
             not publishing.
        Opening the bus while head_node runs corrupts it (45% bad reads), so the
        panel deliberately does not compete.
        """
        tel = self.ros.head_telemetry() if self.ros is not None else None
        if tel:
            return "telemetry", tel
        if self.head.bus is not None:
            d = self.head.telemetry()
            if d is not None:
                d = dict(d)
                d.setdefault("bus_errors", 0)
                return "direct", d
        return None, None

    def _update_pane(self, widget, text, key=""):
        """Repaint a Static only when its content actually changed.

        Every tab used to call .update() on every refresh tick, which forces a
        full re-measure and re-render of that widget even when the text is
        byte-identical. The Servo tab changes constantly so it always repaints,
        but Base/Lidar/Vision mostly do not -- skipping them measurably reduces
        CPU per tick and stops the terminal flickering.
        """
        if not hasattr(self, "_pane_cache"):
            self._pane_cache = {}
        plain = text.plain if hasattr(text, "plain") else str(text)
        if self._pane_cache.get(key) == plain:
            return
        self._pane_cache[key] = plain
        widget.update(text)

    def _render_servo(self):
        w = self.query_one("#servo_info", Static)
        t = Text()

        src, d = self._head_source()

        if d is None:
            t.append("HEAD AX-12A  --  no data\n\n", style="bold")
            if self.head.error:
                t.append(self.head.error + "\n\n", style="yellow")
            t.append("Two ways to get data:\n", style="dim")
            t.append("  * start the head from the Process tab -- head_node owns "
                     "the bus and publishes /head/telemetry (recommended)\n",
                     style="dim")
            t.append("  * press 'c' to open the bus directly, if nothing else "
                     "holds it\n", style="dim")
            t.append("\nThe bus is exclusive: two writers corrupt it (measured "
                     "45% bad reads).\n", style="dim")
            w.update(t)
            return

        label = ("via /head/telemetry (head_node owns the bus)"
                 if src == "telemetry" else "direct bus access")
        t.append("HEAD PAN (AX-12A id %d)\n" % self.head.dxl_id, style="bold")
        t.append("source: %s\n\n" % label, style="dim")

        rad = d.get("rad")
        tick = d.get("tick")
        t.append("position   ")
        t.append("%.3f rad" % rad if rad is not None else "-", style="bold cyan")
        if tick is not None:
            t.append("   tick %d" % tick, style="dim")
        t.append("\n           ")
        if rad is not None:
            t.append(_bar(rad, -1.5, 1.5, 30))
        t.append("  -1.5 .. 1.5\n\n")

        t.append("speed      ")
        t.append(_fmt(d.get("speed"), " rad/s", 3), style="cyan")
        t.append("\n")
        t.append("load       ")
        t.append(_fmt(d.get("load"), " N*m", 3), style="cyan")
        t.append("   (torque proxy)\n")
        t.append("voltage    ")
        t.append(_fmt(d.get("voltage"), " V", 1), style="cyan")
        t.append("\n")
        t.append("temp       ")
        temp = d.get("temp")
        tstyle = "bold red" if temp and temp > 60 else "cyan"
        t.append(("%d C" % temp) if temp is not None else "-", style=tstyle)
        t.append("\n")
        t.append("torque     ")
        t.append("ON" if d.get("torque") else "OFF",
                 style="bold green" if d.get("torque") else "dim")
        if src == "direct":
            t.append("     (press t to toggle)", style="dim")
        else:
            t.append("     (head_node owns it: press t on the Process tab, or "
                     "call /head/enable)", style="dim")
        t.append("\n")

        errs = d.get("bus_errors")
        if errs is not None:
            t.append("bus errors ")
            t.append(str(errs), style="bold red" if errs else "dim")
            t.append("   (increases when another process fights for the port)\n",
                     style="dim")

        t.append("\nNO CURRENT SENSOR. ", style="bold yellow")
        t.append("The AX-12A has no present-current register;\naddresses 68-71 "
                 "all read 0. Use load as the torque proxy.\n\n", style="dim")

        if src == "direct":
            t.append("jog: f / b  12 deg   F / B  30 deg   z zero   t torque\n",
                     style="dim")
        else:
            t.append("Read-only while head_node runs. To jog, either stop "
                     "head_node and press 'c',\n", style="dim")
            t.append("or drive this joint over ROS:\n", style="dim")
            t.append("  ros2 topic pub --once /head/joint_goal "
                     "sensor_msgs/msg/JointState \"{name: [head_joint], "
                     "position: [0.5]}\"\n", style="cyan")
        self._update_pane(w, t, "servo")

    def _render_arm(self):
        table = self.query_one("#t_arm", DataTable)
        if self.arm.bus is None:
            if getattr(self, "_arm_sig", None) != "closed":
                self._arm_sig = "closed"
                table.clear()
                table.add_row(Text("!"), Text("closed", style="dim"),
                              Text(self.arm.error or "not open", style="yellow"),
                              Text(""), Text(""), Text(""), Text(""))
            return

        states = self.arm.telemetry()
        if not states:
            if getattr(self, "_arm_sig", None) != "noreply":
                self._arm_sig = "noreply"
                table.clear()
                table.add_row(Text("?"), Text("no reply", style="dim"),
                              Text(""), Text(""), Text(""), Text(""), Text(""))
            return

        # Position/load/current change continuously, so a signature check would
        # never match and the table would be rebuilt every tick -- the very bug
        # being fixed. Instead UPDATE cells in place: no clear(), so cursor_row
        # is never touched. Rows are added once.
        ids = tuple(s["id"] for s in states)
        need_rows = (getattr(self, "_arm_sig", None) != ids) or not table.row_count
        if need_rows:
            saved = self._saved_cursor(table)
            self._arm_sig = ids
            table.clear()
            for s in states:
                table.add_row(Text(""), Text(""), Text(""), Text(""),
                              Text(""), Text(""), Text(""),
                              key="arm%d" % s["id"])
            self._restore_cursor(table, saved)

        for i, s in enumerate(states):
            if not s.get("ok"):
                values = (Text("✕", style="red"), Text("id %d" % s["id"]),
                          Text("no reply", style="dim"),
                          Text(""), Text(""), Text(""), Text(""))
            else:
                tstyle = "bold red" if s["temp"] > 60 else "cyan"
                values = (
                    Text(str(s["id"]), style="bold"),
                    Text(s["name"]),
                    Text("%.3f" % s["rad"], style="bold cyan"),
                    Text(str(s["load"]), style="dim"),
                    Text(str(s["current"]), style="cyan"),
                    Text("%.1f" % s["volt"], style="dim"),
                    Text("%dC" % s["temp"], style=tstyle),
                )
            try:
                for col, v in enumerate(values):
                    table.update_cell_at((i, col), v)
            except Exception:                        # noqa: BLE001
                pass

    def _render_base(self):
        w = self.query_one("#base_info", Static)
        t = Text()
        odom = self.ros.odom() if self.ros is not None else None
        scan = self.ros.scan() if self.ros is not None else None

        t.append("BASE DRIVE   limits %.2f m/s  %.2f rad/s\n\n"
                 % (MAX_LIN_VEL, MAX_ANG_VEL), style="bold")

        t.append("linear     ")
        t.append("%+.2f m/s" % self._lin, style="bold cyan")
        t.append("   ")
        t.append(_bar(self._lin, -MAX_LIN_VEL, MAX_LIN_VEL, 20))
        t.append("\n")
        t.append("angular    ")
        t.append("%+.2f rad/s" % self._ang, style="bold cyan")
        t.append("   ")
        t.append(_bar(self._ang, -MAX_ANG_VEL, MAX_ANG_VEL, 20))
        t.append("\n\n")

        t.append("drive: w forward  x back  a left  d right  space STOP\n",
                 style="yellow")
        t.append("step:  %.2f m/s, %.2f rad/s\n\n" % (LIN_STEP, ANG_STEP), style="dim")

        t.append("ODOMETRY\n", style="bold")
        if odom is None:
            t.append("  no /odom -- is the base running?\n", style="dim")
        else:
            t.append("  x %.3f m   y %.3f m   yaw %.3f rad\n"
                     % (odom["x"], odom["y"], odom["yaw"]), style="cyan")
            t.append("  vx %.3f m/s   wz %.3f rad/s\n"
                     % (odom["vx"], odom["wz"]), style="dim")
            age = time.time() - odom["stamp"]
            t.append("  age %.1f s%s\n" % (age, "  STALE" if age > 2 else ""),
                     style="red" if age > 2 else "dim")

        t.append("\nSCAN\n", style="bold")
        if scan is None:
            t.append("  no /scan\n", style="dim")
        else:
            t.append("  %d points   0.15 .. %.1f m\n"
                     % (scan["count"], scan["range_max"]), style="cyan")
        self._update_pane(w, t, "base")

    def _render_lidar(self):
        w = self.query_one("#lidar_info", Static)
        t = Text()
        scan = self.ros.scan() if self.ros is not None else None
        t.append("RPLIDAR\n\n", style="bold")

        mod = BY_KEY["lidar"]
        t.append("launcher   ")
        t.append("running" if mod.is_running() else "stopped",
                 style="green" if mod.is_running() else "dim")
        t.append("\nsource     /dev/jupiter/lidar (CP2102 10c4:ea60)\n\n", style="dim")

        if scan is None:
            t.append("no /scan yet -- start the lidar from the Process tab\n",
                     style="yellow")
        else:
            t.append("points     %d\n" % scan["count"], style="cyan")
            t.append("range      0.15 .. %.1f m\n" % scan["range_max"], style="cyan")
            age = time.time() - scan["stamp"]
            t.append("age        %.1f s%s\n"
                     % (age, "  STALE" if age > 2 else ""),
                     style="red" if age > 2 else "dim")

        t.append("\n3D VIEW\n", style="bold")
        t.append("  press v to open RViz2 with LaserScan + TF + RobotModel\n",
                 style="yellow")
        t.append("  (RViz runs as a separate window; a terminal cannot render "
                 "3D)\n", style="dim")
        self._update_pane(w, t, "lidar")

    def _render_vision(self):
        w = self.query_one("#vision_info", Static)
        t = Text()
        t.append("VISION\n\n", style="bold")

        cam = BY_KEY["camera"]
        t.append("camera     ")
        t.append("running" if cam.is_running() else "stopped",
                 style="green" if cam.is_running() else "dim")
        t.append("\nusb       2bc5:065b Orbbec Astra\n", style="dim")
        t.append("streams   depth + colour (IR disabled while colour is on)\n\n",
                 style="dim")

        t.append("topics\n", style="bold")
        for name, typ in (("/camera/color/image_raw", "rgb8 640x480 @25 Hz"),
                          ("/camera/depth/image_raw", "16UC1 640x480 @28 Hz"),
                          ("/camera/depth_registered/points", "PointCloud2")):
            t.append("  %-34s %s\n" % (name, typ), style="cyan")

        t.append("\n3D VIEW\n", style="bold")
        t.append("  press v to open RViz2 with PointCloud2 + colour Image\n",
                 style="yellow")
        t.append("  point clouds cannot be drawn as text -- that is what RViz "
                 "is for\n", style="dim")
        self._update_pane(w, t, "vision")

    # ------------------------------------------------------------ actions
    def _current_tab(self):
        try:
            tabs = self.query_one(TabbedContent)
            return tabs.active
        except Exception:                            # noqa: BLE001
            return "overview"

    def action_next_tab(self):
        self._cycle_tab(1)

    def action_prev_tab(self):
        self._cycle_tab(-1)

    def _cycle_tab(self, delta):
        order = ["overview", "servo", "arm", "base", "lidar", "vision",
                 "process", "log"]
        cur = self._current_tab()
        try:
            i = order.index(cur)
        except ValueError:
            i = 0
        try:
            self.query_one(TabbedContent).active = order[(i + delta) % len(order)]
        except Exception:                            # noqa: BLE001
            pass

    def action_probe(self):
        # Drop the TTL cache and bypass it for this pass, so 'p' really re-probes.
        self._probe_cache = {}
        self._force_probe = True
        self._status = "probing (serial pings included)..."
        try:
            self._refresh()
        finally:
            self._force_probe = False
        self._status = "probe complete"

    def action_refresh_now(self):
        self._refresh()

    # --------------------------------------------------------------- servo
    JOG_SMALL = 0.21          # 12 degrees
    JOG_BIG = 0.52            # 30 degrees

    def _actuator_context(self):
        """Which tab the jog keys act on: 'head', 'arm', or None.

        f/b mean different things on different tabs, so the tab decides the
        target. This keeps one pair of keys instead of two, and prevents a jog
        while looking at the Overview tab.
        """
        tab = self._current_tab()
        if tab == "servo":
            return "head"
        if tab == "arm":
            return "arm"
        return None

    def action_jog_fwd(self):
        self._jog(self.JOG_SMALL)

    def action_jog_back(self):
        self._jog(-self.JOG_SMALL)

    def action_jog_fwd_big(self):
        self._jog(self.JOG_BIG)

    def action_jog_back_big(self):
        self._jog(-self.JOG_BIG)

    def action_zero(self):
        """Go to 0 rad on whichever actuator group the tab is showing."""
        self._goto_absolute(0.0)

    def _ros_head_goal(self, target):
        """Send a head goal over ROS instead of touching the serial bus."""
        if self.ros is None:
            return False
        return self.ros.head_goal(target)

    def _open_if_needed(self, which):
        bus = self.head if which == "head" else self.arm
        if bus.bus is not None:
            return True
        if bus.resolve():
            self._status = "%s bus opened on %s" % (which, bus.port)
            return True
        self._status = "%s: %s" % (which, bus.error)
        return False

    def _jog(self, delta):
        which = self._actuator_context()
        if which is None:
            self._status = "jog works on the Servo and Arm tabs"
            return

        # If head_node owns the bus (telemetry mode), drive the joint over ROS
        # instead of stealing the port. That is the whole point of the
        # telemetry path: full read AND write without bus contention.
        if which == "head":
            tel = self.ros.head_telemetry() if self.ros is not None else None
            if tel and tel.get("rad") is not None:
                target = max(-1.5, min(1.5, tel["rad"] + delta))
                if self._ros_head_goal(target):
                    self._status = "head -> %.3f rad (via ROS)" % target
                else:
                    self._status = "head: ROS goal failed"
                return

        if not self._open_if_needed(which):
            return

        if which == "head":
            d = self.head.telemetry()
            if d is None:
                self._status = "head: no telemetry"
                return
            target = max(-1.5, min(1.5, d["rad"] + delta))
            self._write(which, None, target)
        else:
            states = self.arm.telemetry()
            sel = self._selected_arm_id()
            row = next((s for s in states if s["id"] == sel), None)
            if row is None or not row.get("ok"):
                self._status = "arm id %d not answering" % sel
                return
            lo, hi = self.arm.limits_for(sel)
            target = max(lo, min(hi, row["rad"] + delta))
            self._write(which, sel, target)

    def _goto_absolute(self, rad):
        which = self._actuator_context()
        if which is None:
            self._status = "zero works on the Servo and Arm tabs"
            return
        # Same rule as _jog: prefer ROS when head_node owns the bus.
        if which == "head":
            tel = self.ros.head_telemetry() if self.ros is not None else None
            if tel:
                target = max(-1.5, min(1.5, float(rad)))
                if self._ros_head_goal(target):
                    self._status = "head -> %.3f rad (via ROS)" % target
                else:
                    self._status = "head: ROS goal failed"
                return
        if not self._open_if_needed(which):
            return
        self._write(which, None if which == "head" else self._selected_arm_id(),
                    rad)

    def _write(self, which, sid, target):
        if which == "head":
            ok = self.head.goto_rad(target)
            label = "head_joint"
        else:
            ok = self.arm.goto_rad(sid, target)
            label = "arm id %d" % sid
        if ok:
            self._status = "%s -> %.3f rad" % (label, target)
        else:
            self._status = "%s: write failed" % label

    def _selected_arm_id(self):
        """Servo id under the cursor on the Arm tab, else the stored default."""
        try:
            table = self.query_one("#t_arm", DataTable)
            row, _ = table.cursor_coordinate
            key = table.coordinate_to_cell_key((row, 0)).row_key
            if key and key.value and key.value.startswith("arm"):
                return int(key.value[3:])
        except Exception:                            # noqa: BLE001
            pass
        return getattr(self, "_arm_sel", 1)

    def action_centre(self):
        """Open the bus if needed, else centre the selected actuator."""
        which = self._actuator_context() or ("head" if self.head.bus is not None
                                             else "arm")
        self._goto_absolute(0.0)

    def action_toggle_torque(self):
        which = self._actuator_context()
        if which == "head":
            if not self._open_if_needed("head"):
                return
            d = self.head.telemetry()
            cur = bool(d.get("torque")) if d else False
            if self.head.set_torque(not cur):
                self._status = "head torque -> %s" % ("OFF" if cur else "ON")
            else:
                self._status = "head torque write failed"
            return

        # Arm: torque is per-servo, so toggle just the selected one.
        if not self._open_if_needed("arm"):
            return
        sid = self._selected_arm_id()
        row = next((s for s in self.arm.telemetry() if s["id"] == sid), None)
        # Read the enable register directly; telemetry has no torque field.
        raw = self.arm.bus.read(sid, hw.FT_ADDR_TORQUE, 1)
        cur = bool(raw and raw[0])
        if self.arm.set_torque(sid, not cur):
            self._status = "arm id %d torque -> %s" % (sid, "OFF" if cur else "ON")
        else:
            self._status = "arm id %d torque write failed" % sid

    # ---------------------------------------------------------------- base
    def _base_pub(self):
        if self.ros is None:
            self._status = "ROS bridge not ready"
            return
        self.ros.publish_cmd(self._lin, self._ang)

    def action_drive_fwd(self):
        self._lin = clamp_lin(self._lin + LIN_STEP)
        self._base_pub()

    def action_drive_back(self):
        self._lin = clamp_lin(self._lin - LIN_STEP)
        self._base_pub()

    def action_drive_left(self):
        self._ang = clamp_ang(self._ang + ANG_STEP)
        self._base_pub()

    def action_drive_right(self):
        self._ang = clamp_ang(self._ang - ANG_STEP)
        self._base_pub()

    def action_drive_stop(self):
        self._lin = self._ang = 0.0
        if self.ros is not None:
            self.ros.stop_base()
        self._status = "STOP"

    # --------------------------------------------------------------- rviz
    def action_open_rviz(self):
        if self._rviz is not None and self._rviz.poll() is None:
            self._status = "RViz already open"
            return
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("jupiterobot2_description")
            cfg = hw.rviz_config_path(share)
        except Exception as exc:                     # noqa: BLE001
            self._status = "cannot find rviz config: %s" % exc
            return
        if not os.path.exists(cfg):
            self._status = "rviz config missing: %s" % cfg
            return
        try:
            self._rviz = subprocess.Popen(
                ["rviz2", "-d", cfg],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self._status = "RViz2 opened (%s)" % cfg
        except FileNotFoundError:
            self._status = "rviz2 not installed"

    # ------------------------------------------------------------ process
    def _selected_process_key(self):
        table = self.query_one("#t_process", DataTable)
        try:
            if table.cursor_row is not None and table.cursor_row >= 0:
                rk = table.coordinate_to_cell_key(
                    (table.cursor_row, 0)).row_key
                if rk and rk.value:
                    return rk.value
        except Exception:                            # noqa: BLE001
            pass
        return None

    def action_toggle_selected(self):
        tab = self._current_tab()
        if tab != "process":
            self._status = "start/stop applies on the Process tab"
            return
        key = self._selected_process_key()
        if not key:
            self._status = "no module selected"
            return
        mod = BY_KEY.get(key)
        if mod is None:
            return
        if mod.is_running():
            ok, msg = self.launcher.stop(key)
        else:
            ok, msg = self.launcher.start(key)
        self._status = msg
        self._write_log("[panel] " + msg)
        self._refresh()

    def action_toggle_tab(self):
        pass

    def _write_log(self, line):
        try:
            self.query_one("#logs", RichLog).write(line)
        except Exception:                            # noqa: BLE001
            pass

    def on_data_table_row_selected(self, event):
        key = event.row_key.value if event.row_key else None
        if self._current_tab() == "process" and key:
            mod = BY_KEY.get(key)
            if mod is None:
                return
            if mod.is_running():
                ok, msg = self.launcher.stop(key)
            else:
                ok, msg = self.launcher.start(key)
            self._status = msg
            self._write_log("[panel] " + msg)
            self._refresh()

    def action_cursor_down(self):
        self._move_cursor(1)

    def action_cursor_up(self):
        self._move_cursor(-1)

    def _move_cursor(self, delta):
        tab = self._current_tab()
        tid = {"overview": "#t_overview", "process": "#t_process",
               "arm": "#t_arm"}.get(tab)
        if not tid:
            return
        try:
            table = self.query_one(tid, DataTable)
            table.action_cursor_down() if delta > 0 else table.action_cursor_up()
        except Exception:                            # noqa: BLE001
            pass


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="jupiter-panel",
        description="Jupiter 2 access panel: monitor and control per subsystem")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="refresh seconds (default 1.0)")
    ap.add_argument("--tab", default="overview",
                    help="tab to open first")
    ap.add_argument("--once", action="store_true",
                    help="print a status snapshot and exit")
    args = ap.parse_args(argv)

    if args.once:
        return _print_once()

    JupiterPanel(interval=args.interval, start_tab=args.tab).run()
    return 0


def _print_once():
    """Headless snapshot: launcher state plus live hardware, no UI."""
    lines = []
    launcher = Launcher()
    lines.append("modules")
    for m in launcher.modules:
        lines.append("  %-12s %-9s %s" % (m.key, m.state, m.command_text()))

    head = hw.HeadBus()
    lines.append("\nhead (%s)" % head.port)
    if head.resolve():
        d = head.telemetry()
        lines.append("  position %.3f rad  tick %s" % (d["rad"], d["tick"]))
        lines.append("  speed %s  load %s  volt %s  temp %s"
                     % (d.get("speed"), d.get("load"), d.get("voltage"),
                        d.get("temp")))
        lines.append("  NOTE: no current register on AX-12A")
        head.close()
    else:
        lines.append("  " + head.error)

    arm = hw.ArmBus()
    lines.append("\narm (%s)" % arm.port)
    if arm.resolve():
        for s in arm.telemetry():
            if s.get("ok"):
                lines.append("  id%d %-14s %.3f rad  load %s  current %s  %dC"
                             % (s["id"], s["name"], s["rad"], s["load"],
                                s["current"], s["temp"]))
        arm.close()
    else:
        lines.append("  " + arm.error)

    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
