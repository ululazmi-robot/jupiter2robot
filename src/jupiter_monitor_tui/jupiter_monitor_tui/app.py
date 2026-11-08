"""The Jupiter 2 monitor dashboard.

Design decisions worth stating up front, because they are what make this
different from a generic ROS monitor:

1. It reads the MANIFEST when /robot/health is silent. So it is useful before
   anything is running -- it shows what *should* exist and probes it live. A
   dashboard that shows an empty screen because a node is down is useless
   exactly when you need it.

2. It groups by category (body / arm / head / vision / audio / tools), because
   "which subsystem is broken" is the question people actually ask.

3. It shows the TOPIC for each driver, not just a green dot. Topic names are
   NATIVE ROS 2 names (/cmd_vel, /scan, /odom), so what you read here is what
   you type into `ros2 topic echo`.

4. It is read-only. There is no start/stop here: the repo's own launches own the
   drivers, and a monitor that can kill them is a monitor you cannot trust.

Keys
    up/down or j/k   move selection
    tab / shift-tab  cycle category filter
    d                toggle detail pane (full topic list + device + notes)
    p                probe now (re-read everything, including serial pings)
    /                filter by substring
    q                quit
"""

import argparse
import sys
import threading
import time

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.widgets import DataTable, Footer, Header, Input, Static

# Import the manifest helpers. jupiter_monitor_hub is a sibling package
# installed in the same prefix; if it is missing the TUI degrades to
# manifest-less mode rather than refusing to start.
try:
    import jupiter_monitor_hub.manifest as mf
except ImportError:                                  # pragma: no cover
    mf = None


STATE_STYLE = {
    "online":  ("bold green",   "●"),
    "busy":    ("bold yellow",  "◐"),
    "offline": ("dim",          "○"),
    "no_hw":   ("dim red",      "✕"),
    "error":   ("bold red",     "!"),
    "sim":     ("bold cyan",    "◆"),
    "unknown": ("dim",          "?"),
}

CATEGORY_ORDER = ("body", "arm", "head", "vision", "audio", "tools")
CATEGORY_TITLE = {
    "body":   "BODY   -- drive, odometry, lidar",
    "arm":    "ARM    -- SO-100 6-servo manipulator",
    "head":   "HEAD   -- camera pan (AX-12A)",
    "vision": "VISION -- depth camera, perception pipeline",
    "audio":  "AUDIO  -- microphone, speaker, voice nodes",
    "tools":  "TOOLS  -- teleop, nav, SLAM, simulation",
}


def _fmt_rate(hz):
    if hz <= 0:
        return "-"
    if hz >= 10:
        return "%.0f Hz" % hz
    return "%.1f Hz" % hz


class DriverRow:
    """One rendered row: manifest entry + live state."""

    __slots__ = ("key", "category", "label", "device", "driver", "backend",
                 "state", "detail", "rate", "pid", "topics", "primary",
                 "real_hw", "controllable", "requires_confirm", "notes")

    def __init__(self, entry, state="unknown", detail="not probed", rate=0.0, pid=0):
        self.key = entry["key"]
        self.category = entry["category"]
        self.label = entry.get("label", self.key)
        self.device = str(entry.get("device", "-"))
        self.driver = entry.get("driver", "")
        self.backend = entry.get("backend", "")
        self.state = state
        self.detail = detail
        self.rate = rate
        self.pid = pid
        self.topics = entry.get("topics") or []
        self.primary = entry.get("primary_topic") or ""
        self.real_hw = bool(entry.get("real_hardware"))
        self.controllable = bool(entry.get("controllable"))
        self.requires_confirm = bool(entry.get("requires_confirm"))
        self.notes = (entry.get("notes") or "").strip()


class JupiterMonitor(App):
    CSS = """
    Screen { background: $surface; }
    #layout { height: 1fr; }
    #main { width: 3fr; }
    #side { width: 2fr; border-left: solid $primary-darken-2; padding: 0 1; }
    #summary { height: 3; padding: 0 1; background: $boost; }
    #detail { height: 1fr; }
    DataTable { height: 1fr; }
    #filter { display: none; height: 3; }
    #filter.visible { display: block; }
    """

    BINDINGS = [
        Binding("q", "quit", "quit"),
        # priority=True so the DataTable never consumes these itself. Without
        # it, Down/Up moved the widget's cursor while self.selected stayed put,
        # and the next 1 Hz refresh snapped the highlight back to the top --
        # which looked like the selection resetting itself over and over.
        Binding("j", "cursor_down", "down", show=False),
        Binding("k", "cursor_up", "up", show=False),
        Binding("down", "cursor_down", "down", show=False, priority=True),
        Binding("up", "cursor_up", "up", show=False, priority=True),
        Binding("tab", "next_category", "next cat"),
        Binding("shift+tab", "prev_category", "prev cat"),
        Binding("d", "toggle_detail", "detail"),
        Binding("p", "probe", "probe"),
        Binding("/", "filter", "filter"),
        Binding("escape", "cancel", "cancel"),
    ]

    def __init__(self, interval=1.0, filter_text="", category=None):
        super().__init__()
        self.interval = interval
        self.filter_text = filter_text.lower()
        self.category_filter = category
        self.rows = []
        self.selected = 0
        # Selection is tracked by key so it survives a table rebuild; the index
        # is derived from it, never the other way round.
        self.selected_key = ""
        self._meta = {}
        self._drivers = []
        self._lock = threading.Lock()
        self._health = None          # latest DriverStatusArray, if a hub is up
        self._ros_thread = None
        self._ros_stop = threading.Event()
        self._ros_done = threading.Event()
        self._node = None
        self._show_detail = True
        self._status_line = ""
        self._force_serial = False

        self._load_manifest()

    # ------------------------------------------------------------- manifest
    def _load_manifest(self):
        if mf is None:
            self._status_line = "jupiter_monitor_hub not importable: manifest-less mode"
            return
        try:
            self._meta, self._drivers = mf.load()
            self.rows = [DriverRow(d) for d in self._drivers]
        except Exception as exc:                     # noqa: BLE001
            self._status_line = "manifest error: %s" % exc

    # ------------------------------------------------------------------ ros
    def _start_ros(self):
        """Subscribe to /robot/health in a background thread.

        Deliberately optional: if the hub is not running, probing still works
        and the TUI stays useful.
        """
        def run():
            try:
                import rclpy
                from jupiter_monitor_msgs.msg import DriverStatusArray
            except ImportError:
                return
            try:
                rclpy.init()
            except Exception:                        # noqa: BLE001
                return
            from rclpy.node import Node
            node = Node("jupiter_monitor_tui_listener")
            self._node = node

            def cb(msg):
                with self._lock:
                    self._health = msg

            node.create_subscription(DriverStatusArray, "/robot/health", cb, 10)
            while not self._ros_stop.is_set():
                try:
                    rclpy.spin_once(node, timeout_sec=0.2)
                except Exception:                    # noqa: BLE001
                    break
            # Tear down in this thread, then clear the flag the main thread
            # waits on. Without the join in on_unmount, interpreter shutdown
            # races this thread and aborts with
            # "terminate called without an active exception" (SIGABRT).
            try:
                node.destroy_node()
            except Exception:                        # noqa: BLE001
                pass
            try:
                rclpy.shutdown()
            except Exception:                        # noqa: BLE001
                pass
            self._ros_done.set()

        self._ros_done = threading.Event()
        self._ros_thread = threading.Thread(target=run, daemon=True)
        self._ros_thread.start()

    # ------------------------------------------------------------- compose
    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="layout"):
            with Vertical(id="main"):
                yield Static("", id="summary")
                yield Input(placeholder="filter by substring...", id="filter")
                yield DataTable(id="table", cursor_type="row", zebra_stripes=True)
            with Vertical(id="side"):
                with VerticalScroll(id="detail"):
                    yield Static("", id="detail-text")
        yield Footer()

    def on_mount(self):
        self.title = "Jupiter 2 monitor"
        self.sub_title = "native topics"

        table = self.query_one("#table", DataTable)
        table.add_column("", width=2)
        table.add_column("state", width=8)
        table.add_column("driver", width=18)
        table.add_column("device", width=18)
        table.add_column("rate", width=7)
        table.add_column("primary topic", width=30)
        table.add_column("backend", width=8)

        self._start_ros()
        # Give the health subscription a moment to receive its first message
        # before the first render. Without this the opening frame always showed
        # local-probe results even when a hub was publishing, which is confusing
        # (and briefly wrong) on every start.
        deadline = time.time() + 2.5
        while time.time() < deadline:
            with self._lock:
                if self._health is not None:
                    break
            time.sleep(0.1)

        self._refresh()
        self.set_interval(self.interval, self._refresh)

    def on_unmount(self):
        self._ros_stop.set()
        # Wait for the listener thread to finish its own rclpy shutdown. Skipping
        # this join lets interpreter shutdown race the spin thread and abort with
        # "terminate called without an active exception".
        done = getattr(self, "_ros_done", None)
        thread = getattr(self, "_ros_thread", None)
        if done is not None and thread is not None:
            done.wait(timeout=3.0)
            thread.join(timeout=1.0)

    # ------------------------------------------------------------ refresh
    def _live_map(self):
        """key -> (state, detail, rate, pid) from the hub, if published."""
        out = {}
        with self._lock:
            health = self._health
        if health is None:
            return out
        for d in health.drivers:
            out[d.key] = (d.state, d.detail, float(d.rate_hz), int(d.pid))
        return out

    def _refresh(self):
        live = self._live_map()

        if live:
            for row in self.rows:
                got = live.get(row.key)
                if got:
                    row.state, row.detail, row.rate, row.pid = got
        else:
            self._probe_locally()

        self._render_table()
        self._render_summary(live)
        self._render_detail()

    def _probe_locally(self):
        """No hub: probe every driver in-process so the TUI still tells the truth."""
        if mf is None:
            return
        for row in self.rows:
            entry = next((d for d in self._drivers if d["key"] == row.key), None)
            if entry is None:
                continue
            # serial probes are slow; skip them on the fast path unless asked
            probe = entry.get("probe", "none")
            if probe.startswith("serial:") and not self._force_serial:
                row.state, row.detail, row.pid = mf.UNKNOWN, "serial probe deferred (press p)", 0
                continue
            state, detail, pid = mf.run_probe(probe)
            # A port held by the robot's own driver is the intended state, not
            # an obstruction. Without this the dashboard showed 'busy' for a
            # perfectly healthy driver whenever the hub was not running.
            if state == mf.BUSY and pid and mf._is_bridge_process(pid):
                state = mf.ONLINE
                detail = "driver running (pid %d)" % pid
            row.state, row.detail, row.pid = state, detail, pid
            row.rate = 0.0
        self._force_serial = False

    def _visible(self):
        out = []
        for row in self.rows:
            if self.category_filter and row.category != self.category_filter:
                continue
            if self.filter_text:
                hay = " ".join((row.key, row.label, row.device, row.driver,
                                row.primary, row.state)).lower()
                if self.filter_text not in hay:
                    continue
            out.append(row)
        return out

    def _render_table(self):
        table = self.query_one("#table", DataTable)
        visible = self._visible()

        # Remember the selection by KEY, not by index.
        #
        # The old code restored the cursor with move_cursor(row=...), which does
        # not scroll the view, so after a few rows the cursor was off-screen and
        # the table appeared stuck. It also rebuilt the whole table once a
        # second, which reset the widget's own cursor each time.
        previous_key = self.selected_key

        signature = tuple(
            (r.key, r.category, r.state, r.rate, r.device) for r in visible)
        if signature == getattr(self, "_last_signature", None) and table.row_count:
            return
        self._last_signature = signature

        # Preserve scroll position across the rebuild: clearing a DataTable
        # resets its scroll offset, which is the other half of "it jumps to the
        # top".
        try:
            saved_scroll_y = table.scroll_offset.y
        except Exception:                            # noqa: BLE001
            saved_scroll_y = 0

        table.clear()
        current_cat = None
        for row in visible:
            if row.category != current_cat:
                current_cat = row.category
                title = CATEGORY_TITLE.get(current_cat, current_cat.upper())
                table.add_row(Text(""), Text(title, style="bold reverse"),
                              Text(""), Text(""), Text(""), Text(""), Text(""),
                              key="__hdr__" + current_cat)

            style, glyph = STATE_STYLE.get(row.state, ("dim", "?"))
            dot = Text(glyph, style=style)
            st = Text(row.state, style=style)
            rate = Text(_fmt_rate(row.rate),
                        style="green" if row.rate > 0 else "dim")
            primary = Text(row.primary or "-", style="cyan" if row.primary else "dim")
            table.add_row(
                dot, st,
                Text(row.key, style="bold"),
                Text(row.device, style="dim"),
                rate, primary,
                Text(row.backend, style="dim"),
                key=row.key,
            )

        if not visible:
            return

        # Resolve the key to a row: keep the previous one if still present,
        # otherwise clamp to the nearest valid index.
        keys = [r.key for r in visible]
        if previous_key in keys:
            self.selected_key = previous_key
            self.selected = keys.index(previous_key)
        else:
            self.selected = max(0, min(self.selected, len(keys) - 1))
            self.selected_key = keys[self.selected]

        try:
            table.move_cursor(row=self._row_index_of(self.selected_key),
                              scroll=False)
            table.scroll_to(y=saved_scroll_y, animate=False)
        except Exception:                            # noqa: BLE001
            pass

    def _row_index_of(self, key):
        """DataTable row index for a driver key, accounting for header rows."""
        table = self.query_one("#table", DataTable)
        try:
            return table.get_row_index(key)
        except Exception:                            # noqa: BLE001
            return 0

    def _render_summary(self, live):
        total = len(self.rows)
        online = sum(1 for r in self.rows if r.state in ("online", "sim"))
        real = sum(1 for r in self.rows if r.real_hw and r.state == "online")
        fault = sum(1 for r in self.rows if r.state in ("error", "busy"))
        src = "hub" if live else "local probe"

        parts = [
            ("jupiter2", "bold cyan"),
            ("  %d/%d online" % (online, total), "green" if online else "dim"),
            ("  %d real hw" % real, "cyan" if real else "dim"),
        ]
        if fault:
            parts.append(("  %d fault" % fault, "bold yellow"))
        parts.append(("   source: %s" % src, "dim"))
        if self.category_filter:
            parts.append(("   filter: %s" % self.category_filter, "magenta"))

        text = Text()
        for s, style in parts:
            text.append(s, style=style)
        text.append("\n")
        if self._status_line:
            text.append(self._status_line, style="yellow")
        else:
            text.append("p probe - tab category - d detail - / filter - q quit",
                        style="dim")
        self.query_one("#summary", Static).update(text)

    def _render_detail(self):
        visible = self._visible()
        widget = self.query_one("#detail-text", Static)
        if not visible:
            widget.update(Text("no drivers match", style="dim"))
            return
        row = None
        for r in visible:
            if r.key == self.selected_key:
                row = r
                break
        if row is None:
            row = visible[min(self.selected, len(visible) - 1)]

        t = Text()
        t.append(row.label + "\n", style="bold")
        style, glyph = STATE_STYLE.get(row.state, ("dim", "?"))
        t.append("%s %s" % (glyph, row.state), style=style)
        t.append("  " + row.detail + "\n\n", style="dim")

        for label, value in (("key", row.key), ("driver", row.driver),
                             ("device", row.device), ("backend", row.backend),
                             ("pid", str(row.pid) if row.pid else "-"),
                             ("rate", _fmt_rate(row.rate))):
            t.append("%-9s" % label, style="dim")
            t.append("%s\n" % value, style="white")

        flags = []
        if row.real_hw:
            flags.append("real-hardware")
        if row.controllable:
            flags.append("controllable")
        if row.requires_confirm:
            flags.append("confirm-to-stop")
        if flags:
            t.append("flags    ", style="dim")
            t.append(", ".join(flags) + "\n", style="magenta")

        if row.topics:
            t.append("\ntopics\n", style="bold")
            for name, typ, role in row.topics:
                mark = "->" if role == "command" else "  "
                style2 = "yellow" if role == "command" else "cyan"
                t.append("  %s " % mark, style=style2)
                t.append(name + "\n", style=style2)
                t.append("     %s  %s\n" % (typ, role), style="dim")

        if row.notes:
            t.append("\nnotes\n", style="bold")
            t.append(_wrap(row.notes, 44) + "\n", style="italic")

        widget.update(t)

    # ------------------------------------------------------------ actions
    def action_toggle_detail(self):
        side = self.query_one("#side", Vertical)
        side.display = not side.display

    def action_probe(self):
        self._force_serial = True
        self._status_line = "probing (serial pings included)..."
        self._refresh()
        self._status_line = "probe complete"

    def action_filter(self):
        box = self.query_one("#filter", Input)
        box.add_class("visible")
        box.focus()

    def action_cancel(self):
        box = self.query_one("#filter", Input)
        if box.has_class("visible"):
            box.remove_class("visible")
            self.set_focus(self.query_one("#table", DataTable))

    def on_input_submitted(self, event):
        self.filter_text = (event.value or "").lower()
        self._reset_selection()
        self.action_cancel()
        self._refresh()

    def action_next_category(self):
        cats = list(CATEGORY_ORDER)
        if self.category_filter is None:
            self.category_filter = cats[0]
        else:
            i = cats.index(self.category_filter)
            self.category_filter = cats[(i + 1) % len(cats)]
        self._reset_selection()
        self._refresh()

    def action_prev_category(self):
        cats = list(CATEGORY_ORDER)
        if self.category_filter is None:
            self.category_filter = cats[-1]
        else:
            i = cats.index(self.category_filter)
            self.category_filter = cats[(i - 1) % len(cats)]
        self._reset_selection()
        self._refresh()

    def _reset_selection(self):
        """A filter change invalidates the key AND the render cache."""
        self.selected = 0
        self.selected_key = ""
        self._last_signature = None

    def action_cursor_down(self):
        self._move(1)

    def action_cursor_up(self):
        self._move(-1)

    def _move(self, delta):
        visible = self._visible()
        if not visible:
            return
        keys = [r.key for r in visible]
        # Prefer the tracked key: self.selected can be stale after a filter or
        # a table rebuild, the key cannot.
        if self.selected_key in keys:
            cur = keys.index(self.selected_key)
        else:
            cur = max(0, min(self.selected, len(keys) - 1))
        cur = max(0, min(len(keys) - 1, cur + delta))
        self.selected = cur
        self.selected_key = keys[cur]
        try:
            table = self.query_one("#table", DataTable)
            # scroll=True so the highlight stays on screen, which the old
            # move_cursor call did not do.
            table.move_cursor(row=self._row_index_of(self.selected_key),
                              scroll=True)
        except Exception:                            # noqa: BLE001
            pass
        self._render_detail()

    def on_data_table_row_highlighted(self, event):
        """Keep self.selected in sync when the widget's own cursor moves.

        Mouse clicks and any DataTable-internal movement land here. Without this
        the tracked key and the visible highlight can drift apart.
        """
        key = event.row_key.value if event.row_key else None
        if not key or key.startswith("__hdr__"):
            return
        visible = self._visible()
        for i, row in enumerate(visible):
            if row.key == key:
                self.selected = i
                self.selected_key = key
                self._render_detail()
                return


def _wrap(text, width):
    words = text.split()
    lines, cur = [], ""
    for w in words:
        if len(cur) + len(w) + 1 > width:
            lines.append(cur)
            cur = w
        else:
            cur = (cur + " " + w).strip()
    if cur:
        lines.append(cur)
    return "\n".join("  " + ln for ln in lines)


# ------------------------------------------------------------------- cli ---

def _hub_snapshot(timeout_s=3.0):
    """Try to read one /robot/health message. Returns {key: (state,detail,rate,pid)}.

    `--once` should report the same thing the interactive view does, and the
    interactive view prefers the hub. Consulting the raw prober instead made
    `--once` disagree with the dashboard: a running driver showed as `busy`
    because its own port was held, which is the intended state, not a fault.
    """
    try:
        import rclpy
        from jupiter_monitor_msgs.msg import DriverStatusArray
    except ImportError:
        return {}
    try:
        rclpy.init()
    except Exception:                                # noqa: BLE001
        pass
    try:
        from rclpy.node import Node
    except ImportError:
        return {}
    node = Node("jupiter_monitor_tui_once")
    out = {}
    got = []

    def cb(msg):
        got.append(msg)

    node.create_subscription(DriverStatusArray, "/robot/health", cb, 10)
    deadline = time.time() + timeout_s
    while time.time() < deadline and not got:
        try:
            rclpy.spin_once(node, timeout_sec=0.2)
        except Exception:                            # noqa: BLE001
            break
    try:
        node.destroy_node()
    except Exception:                                # noqa: BLE001
        pass
    if got:
        for d in got[0].drivers:
            out[d.key] = (d.state, d.detail, float(d.rate_hz), int(d.pid))
    try:
        rclpy.shutdown()
    except Exception:                                # noqa: BLE001
        pass
    return out


def _print_once(args):
    """Headless snapshot, for scripts and for CI."""
    if mf is None:
        print("jupiter_monitor_hub not importable", file=sys.stderr)
        return 2
    meta, drivers = mf.load()

    live = {} if args.no_probe else _hub_snapshot()
    src = "hub" if live else "local probe"
    print("jupiter2  manifest v%s  %d drivers   [source: %s]"
          % (meta.get("version"), len(drivers), src))
    print()
    for cat in CATEGORY_ORDER:
        group = [d for d in drivers if d["category"] == cat]
        if not group:
            continue
        print(CATEGORY_TITLE.get(cat, cat.upper()))
        for d in group:
            rate = 0.0
            if args.no_probe:
                state, detail = "-", d.get("probe", "")
            elif d["key"] in live:
                state, detail, rate, _pid = live[d["key"]]
            else:
                state, detail, pid = mf.run_probe(d.get("probe", "none"))
                # a port held by our own driver is healthy, not obstinate
                if state == mf.BUSY and pid and mf._is_bridge_process(pid):
                    state, detail = mf.ONLINE, "driver running (pid %d)" % pid
            glyph = STATE_STYLE.get(state, ("", "?"))[1]
            hz = _fmt_rate(rate)
            print("  %s %-9s %-18s %-18s %-7s %s"
                  % (glyph, state, d["key"], str(d.get("device", "-")), hz, detail))
            if args.verbose and d.get("topics"):
                for name, typ, role in d["topics"]:
                    print("        %-34s %-40s %s" % (name, typ, role))
        print()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="jupiter-monitor",
        description="Jupiter 2 driver dashboard (body/arm/head/vision/audio/tools)")
    ap.add_argument("--once", action="store_true",
                    help="print one snapshot and exit (no interactive UI)")
    ap.add_argument("--json", action="store_true", help="with --once: emit JSON")
    ap.add_argument("--no-probe", action="store_true",
                    help="with --once: list the manifest without probing")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="with --once: also list every topic")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="UI refresh seconds (default 1.0)")
    ap.add_argument("--category", default=None, choices=CATEGORY_ORDER,
                    help="start filtered to one category")
    ap.add_argument("--filter", default="", help="start filtered by substring")
    args = ap.parse_args(argv)

    if args.once:
        if mf is None:
            print("jupiter_monitor_hub not importable", file=sys.stderr)
            return 2
        if args.json:
            import json
            meta, drivers = mf.load()
            rows = []
            for d in drivers:
                state, detail, pid = mf.run_probe(d.get("probe", "none"))
                rows.append({
                    "key": d["key"], "category": d["category"],
                    "label": d.get("label"), "device": str(d.get("device")),
                    "backend": d["backend"], "state": state, "detail": detail,
                    "pid": pid, "rate_hz": 0.0,
                    "primary_topic": d.get("primary_topic", ""),
                    "topics": d.get("topics") or [],
                    "real_hardware": d.get("real_hardware"),
                    "controllable": d.get("controllable"),
                    "requires_confirm": d.get("requires_confirm"),
                })
            print(json.dumps({"manifest": meta, "drivers": rows}, indent=2))
            return 0
        return _print_once(args)

    app = JupiterMonitor(interval=args.interval, filter_text=args.filter,
                         category=args.category)
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
