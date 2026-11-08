"""Lidar: RPLidar on a Silicon Labs CP2102.

PORT RESOLUTION
---------------
This used to hardcode `/dev/ttyUSB1`. That number is assigned by USB
enumeration order, and on this chassis the CP2102 (lidar) and the FTDI FT232H
(AX-12A head servo) swap between reboots. The failure mode was nasty: the
launch started fine, no error was printed, and the lidar simply produced no
scan because the node had opened the HEAD SERVO's bus instead.

Resolution order, first hit wins:

    1. `port:=` launch argument, if you pass one
    2. /dev/jupiter/lidar          -- stable symlink from udev/99-jupiter.rules
    3. the tty whose USB vendor:product is 10c4:ea60 (the CP2102 itself)
    4. /dev/ttyUSB0                -- last-resort guess, logged as such

Step 3 is what makes this work before you install the udev rule: it asks the
kernel which tty is actually the CP2102 rather than trusting a number.
"""

import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Silicon Labs CP210x. Matched on VID:PID, not on the tty name.
CP2102_VID = "10c4"
CP2102_PID = "ea60"

STABLE_LINK = "/dev/jupiter/lidar"


def _find_cp2102_tty():
    """Return the /dev/tty* backed by a CP2102, or '' when none is present."""
    for tty in sorted(glob.glob("/dev/ttyUSB*")):
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
                if vid == CP2102_VID and pid == CP2102_PID:
                    return tty
                break
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
    return ""


def _resolve_port():
    """Pick the lidar port and say which rule matched."""
    if os.path.exists(STABLE_LINK):
        return STABLE_LINK, "stable symlink"
    found = _find_cp2102_tty()
    if found:
        return found, "CP2102 VID:PID scan"
    return "/dev/ttyUSB0", "FALLBACK GUESS -- no CP2102 found"


def generate_launch_description():
    detected, why = _resolve_port()

    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "channel_type": "serial",
            "serial_port": LaunchConfiguration("port"),
            "serial_baudrate": 115200,
            "frame_id": "base_scan",
            "inverted": False,
            "angle_compensate": True,
            # Faster modes overflow the driver buffer after ~30 s and /scan
            # silently stops. Sensitivity is the only safe mode here.
            "scan_mode": "Sensitivity",
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value=detected,
            description="serial port for the RPLidar. Defaults to the value "
                        "resolved at launch time (%s: %s)." % (why, detected)),
        LogInfo(msg="lidar port %s  [resolved by: %s]" % (detected, why)),
        lidar_node,
    ])
