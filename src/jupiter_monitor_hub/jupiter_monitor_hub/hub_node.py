"""The Jupiter 2 monitor hub: one node that knows the state of every driver.

Responsibilities, in order of importance:

1. Load the driver manifest and probe every entry.
2. Publish /robot/health (DriverStatusArray) at a steady rate.
3. Measure the actual rate of each driver's primary topic, so "online" means
   data is flowing rather than "a process exists".
4. Publish /robot/state -- one message with pose, joints, camera geometry and
   audio availability, so consumers never fan out to five drivers.

The hub never opens a serial port for writing. Probes are read-only pings, and
they refuse to touch a port another process holds.

Every topic this node READS is a native ROS 2 name (/odom, /scan, /joint_states,
...), so it works against the repo's stock launches with no remapping. The only
new names are this node's two outputs, /robot/health and /robot/state.

Design note on rate measurement: subscribing to a topic to measure it adds a
subscriber, which is not free. Heavy topics are declared in `skip_rates` so the
hub never subscribes to a 54 MB/s point cloud just to print a number.
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Time as TimeMsg

from jupiter_monitor_msgs.msg import DriverStatus, DriverStatusArray, RobotState

import jupiter_monitor_hub.manifest as mf


# Topics we must never subscribe to for rate measurement: the act of measuring
# would cost more than the information is worth. Matched by substring.
SKIP_RATE_PATTERNS = (
    "/points",
    "/camera/depth/points",
    "/depth_registered/points",
    "/image_rect",
    "/image_raw",
)

# How long to wait for the first message before declaring a topic silent.
RATE_WARMUP_S = 2.0
RATE_WINDOW_S = 3.0


def _qos_for(topic_type):
    """Sensor streams want BEST_EFFORT; everything else is fine reliable."""
    sensor_types = ("sensor_msgs/msg/Image", "sensor_msgs/msg/PointCloud2",
                    "sensor_msgs/msg/LaserScan", "sensor_msgs/msg/Imu")
    if topic_type in sensor_types:
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class HealthHub(Node):
    def __init__(self):
        super().__init__("jupiter_monitor_hub")

        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("measure_rates", True)
        self.declare_parameter("skip_rates", "/points,/depth_registered/points,/image_raw")
        self.declare_parameter("probe_timeout_s", 4.0)
        # Native topic names this node reads. Defaults match the repo's stock
        # launches; override per-robot if a namespace is in use.
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.measure_rates = bool(self.get_parameter("measure_rates").value)
        skip = str(self.get_parameter("skip_rates").value or "")
        self.skip_rates = tuple(p for p in skip.split(",") if p) or SKIP_RATE_PATTERNS

        self._meta, self.drivers = mf.load()
        self.get_logger().info(
            "manifest v%s: %s (%d drivers)"
            % (self._meta.get("version"), self._meta["path"], len(self.drivers))
        )

        # state cache: key -> dict(state, detail, pid, rate_hz)
        self._cache = {}
        for d in self.drivers:
            self._cache[d["key"]] = {
                "state": mf.UNKNOWN, "detail": "not probed yet",
                "pid": 0, "rate": 0.0, "stamp": 0.0,
            }

        # rate measurement bookkeeping
        self._counts = {}       # topic -> [count, first_seen, last_seen]
        self._subs = {}         # topic -> subscription

        # outputs -- the only two names this layer introduces
        self.pub_health = self.create_publisher(
            DriverStatusArray, "/robot/health", 10)
        self.pub_state = self.create_publisher(
            RobotState, "/robot/state", 10)

        # latest robot-level values, filled by lightweight subscriptions
        self._odom = None
        self._battery = None
        self._joints = None
        self._caminfo = None

        self._wire_state_subs()

        # timers
        self.create_timer(1.0 / max(0.5, self.rate_hz), self._tick_publish)
        self.create_timer(1.0, self._tick_probe)

        self._last_state_log = 0.0
        self.get_logger().info(
            "hub ready: /robot/health and /robot/state "
            "(watching native topics)")

    # ------------------------------------------------------------- state subs
    def _wire_state_subs(self):
        """Subscribe to the handful of NATIVE topics RobotState is built from."""
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import BatteryState, JointState, CameraInfo

        odom = str(self.get_parameter("odom_topic").value)
        battery = str(self.get_parameter("battery_topic").value)
        joints = str(self.get_parameter("joint_states_topic").value)
        caminfo = str(self.get_parameter("camera_info_topic").value)

        self.create_subscription(Odometry, odom, self._on_odom, 10)
        self.create_subscription(BatteryState, battery, self._on_battery, 10)
        # One topic covers body, arm and head: joint_state_broadcaster merges
        # them, so there is nothing to join here.
        self.create_subscription(JointState, joints, self._on_joints, 10)
        self.create_subscription(CameraInfo, caminfo, self._on_caminfo, 10)

    def _on_odom(self, msg):
        self._odom = msg

    def _on_battery(self, msg):
        self._battery = msg

    def _on_caminfo(self, msg):
        self._caminfo = msg

    def _on_joints(self, msg):
        if self._joints is None:
            self._joints = {}
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self._joints[name] = (pos, vel)

    # ------------------------------------------------------------- probing
    def _graph_topics(self):
        """Set of topic names currently advertised, from the live graph."""
        try:
            return {t for t, types in self.get_topic_names_and_types()}
        except Exception:                            # noqa: BLE001
            return set()

    def _tick_probe(self):
        """Re-probe every driver. Cheap probes each second; serial pings less often."""
        graph = self._graph_topics()
        now = time.monotonic()
        for d in self.drivers:
            entry = self._cache[d["key"]]
            probe = d.get("probe", "none")

            # Serial pings are expensive and can disturb a running driver, so
            # only do them at most every 5 s.
            if probe.startswith("serial:"):
                if (now - entry.get("stamp", 0)) < 5.0:
                    continue

            # A driver whose primary topic is live is ONLINE regardless of the
            # port: a topic proves function where a port only proves presence.
            topic_alive = False
            primary = d.get("primary_topic") or ""
            if primary and primary in graph:
                rate = entry.get("rate", 0.0)
                # A topic with a publisher but zero measured rate is BUSY-style
                # silence; only treat it live once a message has actually arrived.
                topic_alive = rate > 0.0 or not self.measure_rates

            state, detail, pid = mf.run_probe(probe, known_topics=graph)
            entry["stamp"] = now

            if topic_alive and state in (mf.OFFLINE, mf.BUSY, mf.UNKNOWN):
                state = mf.ONLINE
                detail = "primary topic live (%.1f Hz)" % entry.get("rate", 0.0)

            # Refine BUSY: a held port with a live topic is working correctly.
            if state == mf.BUSY and topic_alive:
                state = mf.ONLINE
                detail = "port held and topic live (%.1f Hz)" % entry.get("rate", 0.0)

            if not d.get("real_hardware") and state == mf.OFFLINE:
                # pure-software entries report "not running", not a fault
                detail = detail or "not running"

            entry["state"] = state
            entry["detail"] = detail
            entry["pid"] = pid

            self._ensure_rate_sub(d, graph)

    def _ensure_rate_sub(self, d, graph):
        if not self.measure_rates:
            return
        topic = d.get("primary_topic") or ""
        if not topic:
            return
        if any(pat and pat in topic for pat in self.skip_rates):
            return
        if topic not in graph:
            self._counts.pop(topic, None)
            # drop a stale subscription so we do not accumulate them
            sub = self._subs.pop(topic, None)
            if sub is not None:
                try:
                    self.destroy_subscription(sub)
                except Exception:                    # noqa: BLE001
                    pass
            return
        if topic in self._subs:
            return

        typ = None
        for entry in d.get("topics") or []:
            if entry[0] == topic:
                typ = entry[1]
                break
        qos = _qos_for(typ or "")
        try:
            sub = self.create_subscription(
                _msg_class(typ), topic,
                lambda msg, t=topic: self._count_msg(t), qos)
        except Exception as exc:                     # noqa: BLE001
            self.get_logger().debug("cannot measure %s: %s" % (topic, exc))
            return
        self._subs[topic] = sub
        self._counts[topic] = [0, time.monotonic(), time.monotonic()]

    def _count_msg(self, topic):
        rec = self._counts.get(topic)
        now = time.monotonic()
        if rec is None:
            self._counts[topic] = [1, now, now]
        else:
            rec[0] += 1
            rec[2] = now

    def _compute_rates(self):
        now = time.monotonic()
        for d in self.drivers:
            topic = d.get("primary_topic") or ""
            entry = self._cache[d["key"]]
            rec = self._counts.get(topic)
            if not topic or rec is None:
                entry["rate"] = 0.0
                continue
            count, first, last = rec
            span = last - first
            if count < 2 or span <= 0:
                entry["rate"] = 0.0
                continue
            # decay the window so a stopped topic falls to zero
            if now - last > RATE_WINDOW_S:
                entry["rate"] = 0.0
                rec[0], rec[1], rec[2] = 0, now, now
                continue
            entry["rate"] = (count - 1) / span

    # ------------------------------------------------------------- publishing
    def _stamp(self):
        t = TimeMsg()
        n = self.get_clock().now().nanoseconds
        t.sec = int(n // 1_000_000_000)
        t.nanosec = int(n % 1_000_000_000)
        return t

    def _driver_msg(self, d):
        entry = self._cache[d["key"]]
        msg = DriverStatus()
        msg.category = d["category"]
        msg.key = d["key"]
        msg.label = d.get("label", d["key"])
        msg.driver = d.get("driver", "")
        msg.device = str(d.get("device", "-"))
        msg.backend = d.get("backend", "")
        msg.state = entry["state"]
        msg.detail = entry["detail"]
        msg.stamp = self._stamp()
        msg.real_hardware = bool(d.get("real_hardware"))
        msg.controllable = bool(d.get("controllable"))
        msg.requires_confirm = bool(d.get("requires_confirm"))
        names, types, roles = [], [], []
        for t in d.get("topics") or []:
            names.append(t[0])
            types.append(t[1])
            roles.append(t[2])
        msg.topic_names = names
        msg.topic_types = types
        msg.topic_roles = roles
        msg.rate_hz = float(entry.get("rate", 0.0))
        msg.pid = int(entry.get("pid", 0))
        return msg

    def _tick_publish(self):
        self._compute_rates()

        arr = DriverStatusArray()
        arr.header.stamp = self._stamp()
        arr.header.frame_id = ""
        out = []
        online = 0
        real = 0
        for d in self.drivers:
            msg = self._driver_msg(d)
            out.append(msg)
            if msg.state in (mf.ONLINE, mf.SIM):
                online += 1
            if msg.real_hardware and msg.state == mf.ONLINE:
                real += 1
        arr.drivers = out
        arr.online_count = online
        arr.total_count = len(out)
        arr.real_hardware_count = real
        self.pub_health.publish(arr)

        self.pub_state.publish(self._robot_state(arr))

    def _robot_state(self, health):
        st = RobotState()
        st.header.stamp = health.header.stamp
        st.header.frame_id = "base_footprint"

        if self._odom is not None:
            p = self._odom.pose.pose
            st.odom_x = p.position.x
            st.odom_y = p.position.y
            st.odom_yaw = _yaw_from_quat(p.orientation)
            st.odom_vx = self._odom.twist.twist.linear.x
            st.odom_wz = self._odom.twist.twist.angular.z

        if self._battery is not None:
            st.battery_voltage = float(self._battery.voltage)
            # turtlebot3_node always reports 0.0; expose -1.0 to mean unknown
            pct = float(self._battery.percentage)
            st.battery_percentage = pct if pct > 0.0 else -1.0

        if self._joints:
            names = sorted(self._joints)
            st.joint_names = names
            st.joint_positions = [self._joints[n][0] for n in names]
            st.joint_velocities = [self._joints[n][1] for n in names]

        ci = self._caminfo
        if ci is not None:
            st.depth_width = int(ci.width)
            st.depth_height = int(ci.height)
            st.depth_frame_id = ci.header.frame_id
            if len(ci.k) >= 6:
                st.depth_fx = float(ci.k[0])
                st.depth_cx = float(ci.k[2])
                st.depth_fy = float(ci.k[4])
                st.depth_cy = float(ci.k[5])

        for d in self.drivers:
            if d["key"] == "audio.input":
                e = self._cache[d["key"]]
                st.audio_input_available = e["state"] == mf.ONLINE
                st.audio_detail = e["detail"]
            elif d["key"] == "audio.output":
                st.audio_output_available = self._cache[d["key"]]["state"] == mf.ONLINE

        st.health = health
        st.hardware_active = health.real_hardware_count > 0
        sim_entry = self._cache.get("tools.sim")
        st.simulation_active = bool(sim_entry and sim_entry["state"] == mf.ONLINE)
        return st


def _yaw_from_quat(q):
    import math
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


_MSG_CLASS_CACHE = {}


def _msg_class(type_str):
    """Resolve 'pkg/msg/Name' to a class, cached."""
    if not type_str:
        return _Fallback
    if type_str in _MSG_CLASS_CACHE:
        return _MSG_CLASS_CACHE[type_str]
    pkg, _, rest = type_str.partition("/")
    kind, _, name = rest.partition("/")
    if kind != "msg":
        raise ValueError("not a message type: %s" % type_str)
    mod = __import__("%s.msg" % pkg, fromlist=[name])
    cls = getattr(mod, name)
    _MSG_CLASS_CACHE[type_str] = cls
    return cls


class _Fallback:
    """Stand-in when a type cannot be resolved: counts any message."""


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = HealthHub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM/SIGINT is the normal way to stop this: exit quietly instead of
        # dumping a traceback that looks like a crash.
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:                            # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
