# One snapshot of everything the robot knows about itself, on /robot/state.
#
# This is the "read everything with one subscription" topic. A vision node, a
# dashboard or a logging job subscribes here instead of fanning out to five
# drivers with five different message types.
#
# Every field is filled from a NATIVE topic; nothing here requires remapping.

std_msgs/Header header

# --- body -------------------------------------------------------------------
# Same semantics as nav_msgs/Odometry but flattened so this message does not
# force a dependency on nav_msgs for consumers that only want a pose.
float64 odom_x
float64 odom_y
float64 odom_yaw
float64 odom_vx
float64 odom_wz

# Battery. `percentage` is -1.0 when the driver does not compute it, which is
# the case for turtlebot3_node (it always reports 0.0 => "unknown").
float64 battery_voltage
float64 battery_percentage

# --- head / arm -------------------------------------------------------------
string[] joint_names
float64[] joint_positions
float64[] joint_velocities

# --- vision -----------------------------------------------------------------
# Depth image geometry, enough to deproject without a camera_info fetch.
int32  depth_width
int32  depth_height
float32 depth_fx
float32 depth_fy
float32 depth_cx
float32 depth_cy
float32 depth_min_range
float32 depth_max_range
string depth_frame_id

# --- audio ------------------------------------------------------------------
bool audio_input_available
bool audio_output_available
string audio_detail

# --- health -----------------------------------------------------------------
DriverStatusArray health

# True when any driver reports real_hardware and is online. A consumer can use
# this to refuse to move things when only simulation is up.
bool hardware_active
bool simulation_active
