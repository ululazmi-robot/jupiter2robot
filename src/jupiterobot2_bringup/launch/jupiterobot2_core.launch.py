"""Base driver: ROBOTIS OpenCR over USB CDC (turtlebot3_node).

PORT RESOLUTION
This used to hardcode /dev/ttyACM0, which is assigned by USB enumeration order.
On this chassis the OpenCR (base) and the CH340 (SO-100 arm) are both CDC
devices, so the two ttyACM numbers swap across reboots. If they swap and this
launch keeps asking for ttyACM0, turtlebot3_node opens the ARM board, the base
never moves, and nothing in the logs says why.

Resolution order, first hit wins:

    1. the port:= launch argument, if you pass one
    2. /dev/jupiter/base      stable symlink from the udev rules
    3. /proc scan for the OpenCR's vendor:product (0483:5740)
    4. /dev/ttyACM0           last-resort guess, logged as such

Step 3 is what makes this work before the udev rule is installed: it asks the
kernel which tty is the STMicroelectronics CDC device instead of trusting a
number.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# STMicroelectronics CDC-ACM, which is what the OpenCR enumerates as.
OPENCR_VID = "0483"
OPENCR_PID = "5740"

STABLE_BASE_LINK = "/dev/jupiter/base"


def _find_opencr_tty():
    """Return the /dev/ttyACM* backed by the OpenCR, or '' if none."""
    for tty in sorted(glob.glob("/dev/ttyACM*")):
        name = os.path.basename(tty)
        base = os.path.join("/sys/class/tty", name, "device")
        if not os.path.isdir(base):
            continue
        cur = os.path.realpath(base)
        for _ in range(8):
            vid_p = os.path.join(cur, "idVendor")
            pid_p = os.path.join(cur, "idProduct")
            if os.path.isfile(vid_p) and os.path.isfile(pid_p):
                try:
                    with open(vid_p) as f:
                        vid = f.read().strip()
                    with open(pid_p) as f:
                        pid = f.read().strip()
                except OSError:
                    break
                if vid == OPENCR_VID and pid == OPENCR_PID:
                    return tty
                break
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    return ""


def _resolve_port():
    if os.path.exists(STABLE_BASE_LINK):
        return STABLE_BASE_LINK, "stable symlink"
    found = _find_opencr_tty()
    if found:
        return found, "OpenCR VID:PID scan"
    return "/dev/ttyACM0", "FALLBACK GUESS -- no OpenCR found"


def generate_launch_description():
    default_port, why = _resolve_port()

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            'humble',
            'burger.yaml'))

    core_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        name='turtlebot3_node',
        parameters=[
            tb3_param_dir,
            {
                'namespace': '',
                'odometry.frame_id': 'odom',
                'odometry.child_frame_id': 'base_footprint',
                'odometry.publish_tf': True,
                'odometry.use_imu': True
            }
        ],
        arguments=['-i', LaunchConfiguration('port')],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value=default_port,
            description='OpenCR serial port. Defaults to the value resolved '
                        'at launch time (%s: %s).' % (why, default_port)),
        LogInfo(msg='base port %s  [resolved by: %s]' % (default_port, why)),
        core_node,
    ])
