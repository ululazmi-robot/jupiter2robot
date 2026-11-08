# All drivers, one message. Published at 2 Hz on /robot/health.
#
# Deliberately flat and small so the TUI can render without extra lookups and
# `ros2 topic echo` stays readable.

std_msgs/Header header
DriverStatus[] drivers

# Convenience roll-ups so a consumer does not have to count states itself.
int32 online_count
int32 total_count
int32 real_hardware_count
