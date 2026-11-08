"""Thin rclpy bridge for the base tab: /cmd_vel out, /odom and /scan in.

Kept separate from the panel's own rclpy usage because this one runs on a
dedicated background thread and must never block the UI. Everything here is
defensive: the TUI has to stay usable when no ROS graph is up at all.

The teleop limits match jupiterobot2_teleop/teleop_keyboard.py exactly
(0.26 m/s, 1.82 rad/s) so the panel and the keyboard node cannot disagree about
what "full speed" means.
"""

import math
import threading
import time

MAX_LIN_VEL = 0.26
MAX_ANG_VEL = 1.82
LIN_STEP = 0.01
ANG_STEP = 0.1


class RosBridge:
    """Owns a node on a background thread; exposes plain dicts to the UI."""

    def __init__(self):
        self.node = None
        self.pub_cmd = None
        self._odom = None
        self._scan = None
        self._head_tel = None
        self.pub_head = None
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self.error = ""

    # ------------------------------------------------------------- lifecycle
    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self._ready.wait(timeout=5.0)

    def _run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import LaserScan
        except ImportError as exc:
            self.error = "rclpy unavailable: %s" % exc
            self._ready.set()
            return
        try:
            rclpy.init()
        except Exception:                            # noqa: BLE001
            pass
        try:
            node = Node("jupiter_access_panel")
        except Exception as exc:                     # noqa: BLE001
            self.error = "cannot create node: %s" % exc
            self._ready.set()
            return

        self.node = node
        self.pub_cmd = node.create_publisher(Twist, "/cmd_vel", 10)

        node.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # /scan and /head/telemetry are live sensor streams: only the newest
        # sample is useful, and a RELIABLE depth-10 queue means a panel stall
        # makes it work through a backlog of stale data afterwards. BEST_EFFORT
        # with depth 1 keeps the panel on the current sample instead. The
        # publishers (rplidar_node uses BEST_EFFORT by default, head_node uses
        # default RELIABLE) remain compatible: a BEST_EFFORT subscriber accepts
        # both.
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)
        # Full head servo telemetry from head_node. Subscribing here is what
        # lets the Servo tab show voltage/temperature/load WITHOUT opening the
        # serial bus, so head_node can own it exclusively.
        try:
            from diagnostic_msgs.msg import DiagnosticArray
            node.create_subscription(DiagnosticArray, "/head/telemetry",
                                     self._on_head_telemetry, sensor_qos)
        except ImportError:
            pass

        self._ready.set()
        while not self._stop.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as exc:                 # noqa: BLE001
                # Do not swallow this silently: a dead spin loop means every
                # subscription stops updating and the UI shows stale/None data
                # with no visible reason. Record it and stop.
                self.error = "spin stopped: %s: %s" % (type(exc).__name__, exc)
                break
        try:
            node.destroy_node()
        except Exception:                            # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:                            # noqa: BLE001
            pass

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ------------------------------------------------------------ callbacks
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._lock:
            self._odom = {
                "x": p.x, "y": p.y,
                "yaw": math.atan2(siny, cosy),
                "vx": msg.twist.twist.linear.x,
                "wz": msg.twist.twist.angular.z,
                "stamp": time.time(),
            }

    def _on_scan(self, msg):
        with self._lock:
            self._scan = {
                "count": len(msg.ranges),
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "stamp": time.time(),
            }

    def _on_head_telemetry(self, msg):
        """Flatten DiagnosticArray into a plain dict for the UI.

        Every conversion is defensive: a malformed KeyValue must never raise
        inside a subscription callback, because an exception here kills the
        spin loop and silently freezes EVERY other subscription too (odom,
        scan). That exact failure happened: bus_errors arrived as b'\\x00' and
        int() raised, taking the whole bridge down.
        """
        if not msg.status:
            return
        st = msg.status[0]
        vals = {kv.key: kv.value for kv in st.values}

        def as_float(key):
            try:
                v = vals.get(key)
                return float(v) if v not in (None, "") else None
            except (TypeError, ValueError):
                return None

        def as_int(key):
            f = as_float(key)
            return int(f) if f is not None else None

        with self._lock:
            self._head_tel = {
                "name": st.name,
                "message": st.message,
                "tick": as_int("tick"),
                "rad": as_float("rad"),
                "speed": as_float("speed"),
                "load": as_float("load"),
                "voltage": as_float("voltage"),
                "temp": as_int("temp"),
                "torque": str(vals.get("torque", "")).lower() == "true",
                "bus_errors": as_int("bus_errors") or 0,
                "port": vals.get("port", ""),
                "stamp": time.time(),
            }

    # -------------------------------------------------------------- getters
    def odom(self):
        with self._lock:
            return dict(self._odom) if self._odom else None

    def scan(self):
        with self._lock:
            return dict(self._scan) if self._scan else None

    def head_telemetry(self):
        """Latest /head/telemetry, or None when head_node is not publishing."""
        with self._lock:
            return dict(self._head_tel) if self._head_tel else None

    def has_cmd_publisher(self):
        try:
            return self.pub_cmd is not None and self.pub_cmd.get_subscription_count() > 0
        except Exception:                            # noqa: BLE001
            return False

    # --------------------------------------------------------------- control
    def publish_cmd(self, lin, ang):
        """Send one Twist. Returns False when nothing is subscribed (no base)."""
        if self.pub_cmd is None:
            return False
        try:
            from geometry_msgs.msg import Twist
            t = Twist()
            t.linear.x = float(lin)
            t.angular.z = float(ang)
            self.pub_cmd.publish(t)
            return True
        except Exception:                            # noqa: BLE001
            return False

    def stop_base(self):
        return self.publish_cmd(0.0, 0.0)

    def head_goal(self, rad, joint="head_joint"):
        """Publish a head goal over ROS, for when head_node owns the bus."""
        if self.node is None:
            return False
        try:
            if self.pub_head is None:
                from sensor_msgs.msg import JointState
                self.pub_head = self.node.create_publisher(
                    JointState, "/head/joint_goal", 10)
            from sensor_msgs.msg import JointState
            m = JointState()
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.name = [joint]
            m.position = [float(rad)]
            self.pub_head.publish(m)
            return True
        except Exception:                            # noqa: BLE001
            return False


def clamp_lin(v):
    return max(-MAX_LIN_VEL, min(MAX_LIN_VEL, v))


def clamp_ang(v):
    return max(-MAX_ANG_VEL, min(MAX_ANG_VEL, v))
