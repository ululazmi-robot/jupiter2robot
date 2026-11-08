# One driver or software tool as seen by the Jupiter 2 monitor.
#
# This is the atom the TUI renders. Everything a student needs to know about a
# subsystem -- what owns it, which port it holds, whether it is alive, which
# topics it exposes -- lives in one of these.
#
# Topic names inside this message are NATIVE ROS 2 names (/cmd_vel, /scan,
# /odom), not a private namespace. The monitor describes the robot as it is.

# --- state constants (string, so `ros2 topic echo` is readable) -------------
string UNKNOWN = "unknown"      # never probed
string ONLINE  = "online"       # driver running and its primary topic is live
string BUSY    = "busy"         # a process holds the device but output is silent
string OFFLINE = "offline"      # device present, nothing driving it
string NO_HW   = "no_hw"        # the hardware does not exist on this chassis
string ERROR   = "error"        # driver running but reporting failure
string SIM     = "sim"          # simulation instead of hardware

# --- identity ---------------------------------------------------------------
string category        # body | arm | head | vision | audio | tools
string key             # stable id: base, lidar, camera, arm, head, mic, ...
string label           # human-readable name
string driver          # package/executable that owns the device
string device          # /dev node, USB vid:pid, or ALSA name
string backend         # native | external  (see config/drivers.yaml)

# --- state ------------------------------------------------------------------
string state           # one of the constants above
string detail          # single-line explanation, safe to display
builtin_interfaces/Time stamp

# --- capability flags -------------------------------------------------------
bool real_hardware     # true when this drives real actuators/sensors
bool controllable      # true when the monitor may start/stop it
bool requires_confirm  # true when a stop would leave hardware energised

# --- ROS interface ----------------------------------------------------------
# Parallel arrays; index i of each describes the same topic.
string[] topic_names   # fully qualified native name, e.g. /cmd_vel
string[] topic_types   # e.g. geometry_msgs/msg/Twist
string[] topic_roles   # primary | command | state | debug | service
float64  rate_hz       # measured on the primary topic, 0.0 when not measured
int32    pid           # owning process, 0 when none
