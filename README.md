# Jupiter 2 — ROS 2 Humble robot platform

A differential-drive mobile robot with a tower, a pan head, an RGB-D camera, a
RPLidar, and a 6-DOF SO-100 arm on the top deck. This repository is the ROS 2
Humble workspace: description, simulation, hardware bringup, teleop, navigation,
vision, voice, and a hardware monitoring TUI.

**Primary audience for this document is an AI coding agent or a human engineer
picking the repo up cold.** It records not just *what* exists but the non-obvious
failures that shaped the design, because several of them cost hours and will
recur if the reasoning is lost.

---

## Table of contents

- [Status](#status)
- [Quick start](#quick-start)
- [Repository layout](#repository-layout)
- [The hard-won knowledge](#the-hard-won-knowledge) ← read this before editing
- [URDF / xacro architecture](#urdf--xacro-architecture)
- [SO-100 arm](#so-100-arm)
- [Hardware reference](#hardware-reference)
- [Launch reference](#launch-reference)
- [Changelog](#changelog)
- [Known issues / not done yet](#known-issues--not-done-yet)

---

## Status

| Area | State | Evidence |
|---|---|---|
| SO-100 arm, real hardware | **Working** | Moves to commanded pose, `error_code: 0`, verified 2026-11-09 |
| SO-100 arm, Gazebo | **Working** | Controllers active, arm traverses trajectories |
| Base + lidar (real) | **Working** | `/odom` live, gyro calibrates, `/scan` ~13 Hz |
| Head servo (real) | **Working** | AX-12A over FTDI, needs torque enable |
| Astra Pro camera | **Working** | `/camera/color/image_raw` ~25 Hz |
| Gazebo simulation | **Working** | `/clock` 10 Hz, `/odom`, `/scan`, `/joint_states` |
| Navigation / SLAM | Partially verified | See [launch reference](#launch-reference) |
| MoveIt / IK for the arm | **Not ported** | No `so100_moveit_config` yet |
| Arm udev symlinks, base udev | Working (`/dev/jupiter/*`) | Verified live |

Workspace builds clean: 20 packages, `colcon build --symlink-install`.

---

## Quick start

### One-time setup per terminal

```bash
cd ~/Jupiter2Robot
unset VIRTUAL_ENV
export PATH=/usr/bin:/bin:/usr/local/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
source install/setup.bash
```

> **`unset VIRTUAL_ENV` is not optional.** With a Python venv active,
> `colcon build` fails inside `ament_package` with `ModuleNotFoundError:
> ament_package`. ROS 2 Humble here is built against `/usr/bin/python3` and a
> venv shadows it. This bites every time a shell has previously activated one.

### Build

```bash
colcon build --symlink-install
```

`--symlink-install` matters for Python nodes and launch files: edits take effect
without a rebuild. It does *not* apply to CMake-installed resources.

### Real arm driver (extra source step)

The SO-100 hardware plugin lives outside this repo:

```bash
source ~/fyp1/install/setup.bash
```

Only the **install tree** of `rbc_so100_hardware` is present on the machine; the
source is not in this repository. Without this source, `hardware.launch.py
fake:=false` fails to load its plugin.

### Verify the workspace is sane

```bash
ros2 pkg list | grep -c jupiter        # expect ~10
ros2 launch jupiterobot2_description preview.launch.py   # RViz + sliders, no Gazebo
```

---

## Repository layout

`src/` holds **20 built ROS 2 packages** plus a few loose files and unpackaged
trees. The loose files (`basic_syntac`, `self_location.py`, `time_stamp_fyp`,
`jupiterobot2_voice.zip`) are scratch or archived artifacts; `sllidar_ros2/`,
`exp1/`, `experiment_log/` and `experiment_log2/` have no `package.xml` and are
not built by colcon. Confirm the real list with:

```bash
colcon list
```

### Core packages

| Package | Build type | Role |
|---|---|---|
| `jupiterobot2_description` | ament_cmake | **Pure geometry.** URDF/xacro, meshes, preview launch. Knows nothing about Gazebo. |
| `jupiterobot2_gazebo` | ament_cmake | **All Gazebo XML** + hardware bringup + SLAM/nav launches + worlds. Depends on `description`. |
| `so100_description` | ament_cmake | SO-100 arm geometry, mount macro, `ros2_control` block, meshes, `safety.py` |
| `jupiterobot2_bringup` | ament_cmake | Real-robot base + lidar launches, top-level bringup |
| `jupiterobot2_head_description` | ament_cmake | Head mount and AX-12 head servo geometry |
| `jupiterobot2_head_bringup` | ament_python | `head_node` — drives the AX-12 head servo over FTDI |
| `jupiterobot2_navigation` | ament_cmake | Nav2 configuration and launch |
| `jupiterobot2_teleop` | ament_cmake | Keyboard (`teleop_keyboard`) and joystick teleop (C++) |
| `jupiterobot2_vision` | ament_python | Camera / detection nodes |
| `jupiterobot2_voice` | ament_python | Voice command nodes |
| `jupiter_monitor_hub` | ament_python | Driver manifest + launch for the monitor |
| `jupiter_monitor_msgs` | ament_cmake | Message definitions for the monitor |
| `jupiter_monitor_tui` | ament_python | Textual TUI dashboard (`monitor` on PATH) |

### Vendored / third-party

| Package | Note |
|---|---|
| `astra_camera`, `astra_camera_msgs` | Orbbec Astra driver — two packages under `ros2_astra_camera/` (11 MB, vendored) |
| `dynamixel_hardware_interface`, `dynamixel_interfaces` | ros2_control hardware for AX/MX servos |

> `src/sllidar_ros2/` has **no `package.xml`** and is not built. The RPLidar is
> driven by the installed system package. Do not assume that directory is live
> code.

### Experiments (not part of the robot stack)

`latency_test`, `vip_dist`, `vip_single` are built packages. `exp1`,
`experiment_log`, `experiment_log2` are not (no `package.xml`) and are largely
standalone FYP measurement work.

> `exp1` is **108 MB**, almost all of it the Vosk speech model under
> `src/exp1/models/`. That directory must stay out of git; see
> [`.gitignore`](#gitignore-and-what-not-to-commit).

---

## The hard-won knowledge

This section is the point of the document. Each item is a real failure that
presented misleadingly. Read it before changing anything.

### 1. Domain isolation — never work on 42

The **live robot runs on `ROS_DOMAIN_ID=42`**. Anything started in that domain
publishes a second `/robot_description`, a second TF tree, and competing
`/odom` and `/cmd_vel`.

```bash
ROS_DOMAIN_ID=42   # live robot — do not touch while testing
```

Use another domain for *everything* else:

| Purpose | Domain | Where set |
|---|---|---|
| Live robot | 42 | ambient |
| Preview / RViz | 77 | `preview.launch.py` default |
| Hardware bringup | 77 | `hardware.launch.py` default |
| Gazebo tests | 78 / 94 | ambient |

`preview.launch.py` accepts `domain:=` and logs a warning if you pass 42.

### 2. The arm driver numbers servos by joint order, not by ID

`rbc_so100_hardware` maps **joint 1 → servo 1, joint 2 → servo 2**, … It does
not read an ID parameter. There is no `servo_id` to configure.

Consequence: declaring 7 joints on a 6-servo bus makes the driver ping servo 7,
time out, and **abort controller_manager**:

```
Servo 1..6 initialized at position ...
[ERROR] No response from servo 7 during initialization
Failed to set the initial state of the component to active
```

This is why `head_joint` is declared **only** for the Gazebo and fake backends
(`urdf/hardware.xacro`). The real system has exactly the 6 arm joints. In real
mode there is no `head_joint` controller; drive the head with
`jupiterobot2_head_bringup` instead, and `hardware.launch.py` skips
`head_group_controller` accordingly.

The driver does **not** hardcode joint names — verified against the `.so`, which
contains no `Base_Shoulder_joint`-style strings. It reads names from the URDF.

### 3. Rest pose is a joint limit — the overload blink

Rest pose is all-zeros, **and 0.0 is also a joint limit** on two joints:

```
shoulder_lift   0.00 .. 3.50     <- 0.0 is the lower stop
elbow_flex     -3.14 .. 0.00     <- 0.0 is the upper stop
```

Servo EEPROM limits match those angles (servo 2 `min=2048`, servo 3 `max=2048` —
exactly 0.0 rad in ticks, `2048 = centre`, `4096 ticks / 360°`). So a joint at
rest is *already touching its stop*. Commanding it to `0.0` — or past it — is a
command to lean on a mechanical limit. The STS3215 responds by holding current
against the stop until it flags overload: the **red blinking LED**.

Measured, not assumed:

| Condition | Result |
|---|---|
| Servo 2 held AT lower limit, 4 s | peak load **3.2 %** |
| Servo 3 driven PAST upper limit | peak load **4.0 %**, refused to move |
| All six idle | **0.0 %**, error byte `0x00`, 27–30 °C |

The arm recovers; nothing is damaged. The rules that keep it that way:

- **Rest pose (all zeros) is safe and allowed.** It is the parked state.
- **Any non-zero goal keeps `STOP_MARGIN_RAD` (0.02 rad ≈ 1.1°) clear of every
  limit**, so you cannot command a joint to lean on a stop.
- `hardware.launch.py` runs `scripts/preflight_check.py` **before**
  `ros2_control_node` and refuses to start if the arm is not parked.

> **Do not add "go home on activate".** Home *is* a limit. That would jam the arm
> on every launch.

`safety.py` in `so100_description/so100_description/` is the single source of
truth for the margin rule. Its `LIMITS` table is validated against the expanded
URDF:

```bash
python3 src/so100_description/so100_description/safety.py <expanded.urdf>
# -> LIMITS matches <expanded.urdf>
```

### 4. The driver does NOT write EEPROM

An earlier revision of this README claimed the driver rewrote servo EEPROM
limits on every launch. **That was wrong** and was disproved by dumping the
limit/torque/offset/mode registers before and after a full `hardware.launch.py`
run: byte-identical across all 20 registers.

The values are pre-existing calibration. The consequence still matters — a
pre-existing EEPROM limit that disagrees with the URDF means the servo stops
before the URDF expects and you get a *tracking error* instead of a crash.
Servo 1 is already in that state:

```
servo 1   eeprom 284..3355    urdf wants 744..3352  (±2.0 rad)
```

Safe (EEPROM is narrower, servo just refuses earlier), but **widening that URDF
limit later will silently do nothing.** Fix with the Feetech tool, not by editing
the URDF.

### 5. The serial port is exclusive — sequencing matters

Only one process can hold `/dev/jupiter/arm`. Starting `preflight_check.py` and
`ros2_control_node` together makes the preflight fail with:

```
device reports readiness to read but returned no data (multiple access on port?)
```

which *looks* like broken hardware but is two readers. `hardware.launch.py`
therefore chains on `OnProcessExit` and only starts the manager if preflight
exited 0. **A guard that does not gate is worse than no guard** — an earlier
version logged the failure and continued anyway.

### 6. After a failed activate, the Feetech bus is wedged

Raw pings that worked beforehand return nothing until the board is unplugged and
replugged. If the arm goes silent after a crash, that is the cause — not a new
fault.

### 7. Feetech protocol: the checksum is a bitwise NOT

To test "are the servos alive?" independently of ROS:

```
checksum = (~(sid + 2)) & 0xFF        for a PING
packet   = FF FF <sid> 02 01 <checksum>
```

`(sid + 3) & 0xFF` is **wrong** and produces "no response" from a perfectly
healthy arm. Broadcast to `0xFE` returns a 6-byte status per servo:

```
ffff 01 02 00 fc   servo 1
ffff 02 02 00 fb   servo 2 ...
```

### 8. Plugin name landmine

The shipped `.so` registers its class as:

```
rbc_so100_controller/RBCSOARM100Interface      (per its own .xml — correct)
```

but the upstream xacro that ships alongside references
`rbc_so100_controller/SOARM100Interface`, **which does not exist**. The C++
symbols confirm the `.xml` is right. Override without editing anything:

```bash
ros2 launch jupiterobot2_gazebo hardware.launch.py \
  arm_plugin_class:=rbc_so100_controller/SomeOtherName
```

### 9. `robot_description` must be `ParameterValue(..., value_type=str)`

Passing a bare `Command(...)` into a node parameter makes `launch_ros`
YAML-parse it. Any URDF comment containing `key: value` kills the launch:

```
Unable to parse the value of parameter robot_description as yaml
```

`so100.urdf` contains comments like `Joint 1: base -> shoulder`. Fixed in every
launch that publishes `robot_description`. Note `gazebo.launch.py` never showed
the bug because it stripped comments — which is why it only appeared in
`display.launch.py` and the hardware bringup.

### 10. A link and a joint cannot share a name in Gazebo

`so100.urdf` originally had both named `gripper`. URDF permits it; Gazebo does
not:

```
Joint with name [gripper] ... has a name collision
```

Fixed-joint reduction then collapses the link and every pose relative to it
mis-resolves. The **link** is now `gripper_body`; the **joint** keeps the name
`gripper` because controllers and MoveIt address it.

### 11. xacro comment traps

- **xacro expands `${...}` inside comments.** Writing substitution syntax in
  prose fails expansion with `name 'X' is not defined`.
- **`--` is illegal in an XML comment**, and comments cannot nest. The error
  message points at the comment, not the real cause.

### 12. `fake` is inverted between the xacro and the launch

Two different defaults for the same concept, and they disagree:

| Where | Argument | Default | Meaning of default |
|---|---|---|---|
| `hardware.xacro` | `use_fake_hardware` | `true` | mock — safe, no port |
| `hardware.launch.py` | `fake` | `false` | **real servos** |

So a bare `xacro ...` opens nothing, but a bare `hardware.launch.py` talks to the
physical arm and runs the preflight check. Anyone assuming "no arguments = safe"
gets real hardware. Pass `fake:=true` explicitly for the mock backend.

### 13. Gazebo silently ignores a missing world file

`gazebo_world.launch.py` once defaulted to `jupiter_empty.world.world` — double
extension, file absent. Gazebo does **not** error on a missing world; it loads a
bare empty world. The bug presented as "my world is ignored."

Two startup messages that are **not** bugs:

- `Desired controller update period (0.02 s) is slower than the gazebo
  simulation period` — harmless `gazebo_ros2_control` warning.
- `Missing model.config for model ...` — Gazebo scanning non-model share dirs.
  Noise.

---

## URDF / xacro architecture

Two layers, one direction of dependency. `jupiterobot2_gazebo` depends on
`jupiterobot2_description`; the description package knows nothing about Gazebo.

```
jupiterobot2_description/urdf/           pure geometry, no plugins
  jupiter2_pro.urdf.xacro                <- GEOMETRY entry point
  config/
    properties.xacro                     M_PI, M_SCALE
    materials.xacro                      <material> colours
  robot/
    base.xacro                           chassis, wheels, casters, imu_link
  tower/
    pole_macros.xacro                    pole_40/60/80/200/300 macros
    plate1.xacro                         camera/lidar deck + 300 mm risers
    plate2.xacro                         arm deck + 200 mm risers
    plate3.xacro                         mic, e-stop, 80 mm posts
    monitor_frame.xacro                  monitor frame_1 -> frame_2
    head_and_monitor.xacro               head_mount_link, monitor, screen
  sensors/
    astra_pro_camera.urdf.xacro          Astra Pro mount + optical frames

jupiterobot2_gazebo/urdf/                ALL Gazebo XML
  jupiter2_pro.gazebo.xacro              <- SIM entry point
  hardware.xacro                         the ONE <ros2_control> block
  plugins/
    diff_drive.xacro                     /cmd_vel, /odom, wheel TF
    surfaces.xacro                       friction + materials, 1 block/link
  sensors/
    imu.xacro                            /imu
    lidar.xacro                          /scan
    cameras.xacro                        /camera/* and /camera_02/*
  ros2_control.xacro                     arm + gripper + head controllers
```

### Which entry point

| Use case | Entry point |
|---|---|
| RViz / MoveIt / real hardware | `jupiterobot2_description` → `jupiter2_pro.urdf.xacro` |
| Gazebo | `jupiterobot2_gazebo` → `jupiter2_pro.gazebo.xacro` |

The `launch/gazebo*.launch.py` files already point at the right one.

### Include order is load-bearing

In `jupiter2_pro.urdf.xacro`: **config first** (xacro resolves properties at
point of *use*), then tower modules in tree order (each attaches to a link
declared by the previous), then the arm.

The arm include pair must stay ordered too: `jupiter2_so100_arm.xacro` defines
the mount joint, `so100.urdf` supplies the `base` link it attaches to.

### Three invariants — all violated by the old tree

1. **A given `<gazebo reference="X">` appears at most once.** Duplicates do not
   merge in Gazebo, they overwrite — that is where "multiple inconsistent `<mu>`"
   and the caster friction silently changing `0.0 → 0.01` came from.
2. **Exactly one `<ros2_control>` element in the whole robot.** `head_joint`
   declared twice is what made the head look dead while controllers reported
   `active`.
3. **No Gazebo tags in a description package.** Camera material blocks used to
   live inside the camera mount macros, which is why moving sensors out exposed
   duplicates.

### Verification method worth reusing

Expand before and after a refactor, then compare:

- `{link names + mesh sets}`
- `{joint name, type, parent, child, xyz, rpy}`

Equal answers mean the change was structural-only.

### Files deleted during reorganisation

| Deleted | Why |
|---|---|
| `jupiter2_lite.urdf.xacro` | Unreferenced |
| `jupiter2_pro.urdf_saved_one.xacro` | Identical to pro plus a newline |
| `jupiterobot2_arm_description` | AX-12 arm, replaced by `so100_description` |
| `jupiterobot2_head_gazebo.xacro` | Empty stub |

The AX-12 STLs survive at `jupiterobot2_gazebo/models/ax12_arm/meshes/` because
the RoboCup world files still reference them from SDF.

---

## SO-100 arm

### Joints — these names everywhere, no aliases

| Joint | Limits (rad) | Servo ID |
|---|---|---|
| `shoulder_pan` | ±2.0 | 1 |
| `shoulder_lift` | 0.0 .. 3.5 | 2 |
| `elbow_flex` | −3.14 .. 0.0 | 3 |
| `wrist_flex` | −2.5 .. 1.2 | 4 |
| `wrist_roll` | ±3.14 | 5 |
| `gripper` | −0.2 .. 2.0 | 6 |

> The joint is `gripper`; the **link** is `gripper_body`. See
> [knowledge #10](#10-a-link-and-a-joint-cannot-share-a-name-in-gazebo).

### Controllers

| Controller | Joints |
|---|---|
| `arm_group_controller` | the 5 arm joints |
| `gripper_group_controller` | `gripper` |
| `head_group_controller` | `head_joint` — **gazebo/fake only**, absent on real hardware |

### Mount

```
plate_2_link          arm deck, z = 0.475 m
  -> arm_holder_link  3D-printed cradle
    -> base           SO-100 root
```

Transforms copied from `~/jupiter2_so100_deploy`, tuned on real hardware. Edit
`arm_holder_joint`'s origin to move the whole arm.

> **Do not route through `arm_mount_link`.** That link is 134 mm outboard and
> existed for the removed AX-12 arm. Hanging the SO-100 there floats it clear of
> the deck (`base` at x = +0.130 instead of −0.004). It is still declared in
> `tower/plate2.xacro` because the RoboCup worlds and RViz lists reference it,
> but nothing functional uses it.

Rest pose TF: `base_footprint → base → shoulder → upper_arm → lower_arm → wrist
→ gripper_body → tcp`, with `tcp` at x ≈ 0.11, z ≈ 0.48 in rest (arm root at
x = −0.004).

### Three backends, one switch, ONE `<ros2_control>`

All in `jupiterobot2_gazebo/urdf/hardware.xacro`:

| Invocation | Plugin | Joints declared |
|---|---|---|
| *(no args)* | `mock_components/GenericSystem` | 7 (arm + head) |
| `use_gazebo:=true` | `gazebo_ros2_control/GazeboSystem` | 7 (arm + head) |
| `use_fake_hardware:=false` | `rbc_so100_controller/RBCSOARM100Interface` | **6** (arm only) |

The xacro's `use_fake_hardware` defaults to `true`, so a bare `xacro` run
opens nothing. **But `hardware.launch.py`'s `fake` argument defaults to
`false`** — see the warning in [Launch reference](#launch-reference).
`gazebo.launch.py` passes `use_gazebo:=true`; `hardware.launch.py` passes
`use_fake_hardware`.

> **Never add a second `<ros2_control>` element.** Gazebo binds a joint to
> whichever block it sees first and silently drops writes aimed at the other.

### Driving it

```bash
# arm
ros2 action send_goal /arm_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll],
    points: [{positions: [0.0, 0.35, -0.45, -0.05, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}"

# gripper
ros2 action send_goal /gripper_group_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [gripper], points: [{positions: [0.5], time_from_start: {sec: 2, nanosec: 0}}]}}"
```

Use a pose where **no joint is at a limit** — see [knowledge #3](#3-rest-pose-is-a-joint-limit--the-overload-blink).

### Maintenance tools

| Tool | Purpose |
|---|---|
| `ros2 run jupiterobot2_gazebo preflight_check.py` | Report whether all 6 servos are at rest; exit 1 if not |
| `ros2 run jupiterobot2_gazebo restore_rest.py --dry-run` | Report drift from rest, move nothing |
| `ros2 run jupiterobot2_gazebo restore_rest.py` | Drive drifted joints back to rest slowly, then release torque |

Rest ticks (not all 2048 — each servo's parked position differs slightly):

```
servo 1..6 := 2042 2046 2052 2034 2050 2042
```

`restore_rest.py` will not work while `hardware.launch.py` is running — the bus
is exclusive.

### Not ported yet

`so100_moveit_config`. MoveIt and `pick_and_carry.py` still live in
`~/jupiter2_so100_deploy` against that bundle's simplified base. Porting needs
the SRDF collision matrix regenerated for the taller tower.

There is **no IK**: this brings up hardware and trajectory controllers, you
command joint angles directly.

---

## Hardware reference

### Device map

| Symlink | Underlying | Device | VID:PID | Baud |
|---|---|---|---|---|
| `/dev/jupiter/base` | `ttyACM0` | ROBOTIS OpenCR (base MCU) | `0483:5740` | — |
| `/dev/jupiter/arm` | `ttyACM1` | CH340 → Feetech STS3215 × 6 | `1a86:55d3` | 1000000 |
| `/dev/jupiter/head` | `ttyUSB0` | FTDI FT232H → AX-12A (ID 6) | `0403:6014` | 1000000, protocol 1.0 |
| `/dev/jupiter/lidar` | `ttyUSB1` | CP2102 → RPLidar | `10c4:ea60` | — |

Camera: Orbbec Astra **2bc5:065b**, serial `AD75733010B`. It is a
vendor-class USB device, so **no `/dev/video*` node is normal.**

No microphone exists on the robot; the only audio input is the laptop's ALC897
analog jack (44.1 kHz stereo native).

### Why the symlinks exist

`/dev/ttyUSB0` and `ttyUSB1` are assigned in USB enumeration order and change
across reboots or replugs. On this chassis the CP2102 (lidar) and FT232H (head
servo) regularly swap — `jupiterobot2_lidar.launch.py` once hardcoded
`/dev/ttyUSB1` and therefore opened the **head servo's bus** instead of the lidar.
Nothing errored; the lidar just sat silent while the head bus was held.

Matching on VID:PID is the fix (vendor IDs differ, unlike serial numbers which
depend on a specific board). Rules in `/etc/udev/rules.d/99-jupiter.rules`:

```bash
sudo cp 99-jupiter.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/jupiter/
```

> **Never hardcode `ttyACM*` or `ttyUSB*` in a launch file.** Use
> `/dev/jupiter/*`. This is exactly how the lidar once grabbed the head servo.

### Development host

ASUS TUF FX505DT running Ubuntu 22.04 + ROS 2 Humble — a dev/sim box, not
onboard compute. Work must stay portable to other machines including WSL2.

---

## Launch reference

### Simulation

```bash
ros2 launch jupiterobot2_gazebo gazebo.launch.py            # default world
ros2 launch jupiterobot2_gazebo gazebo_world.launch.py      # world:=
ros2 launch jupiterobot2_gazebo gazebo_world.launch.py \
  world:=$HOME/Jupiter2Robot/src/jupiterobot2_gazebo/worlds/robocup_2024.world.xml
```

Worlds: `jupiter_empty.world`, `jupiter.world`, `robocup_2024.world.xml`,
`robocup_2025.world.xml`, `trial1_t2.world.xml`.

Competition setups: `robocup_2024.launch.py`, `robocup_2025.launch.py`.

### Model preview (no Gazebo)

```bash
ros2 launch jupiterobot2_description preview.launch.py
ros2 launch jupiterobot2_description preview.launch.py sim:=true
```

Args: `sim`, `gui`, `no_gui`, `rviz`, `domain` (default 77), `frame`.

### Real hardware — SO-100 arm

```bash
source ~/fyp1/install/setup.bash

# DANGER: fake defaults to FALSE, so the bare command opens the real servo bus.
ros2 launch jupiterobot2_gazebo hardware.launch.py

# Mock backend — safe with the arm unplugged, opens no serial port:
ros2 launch jupiterobot2_gazebo hardware.launch.py fake:=true
```

> **`fake` defaults to `false`.** Unlike the xacro's `use_fake_hardware` (which
> defaults to `true`), the launch argument defaults to `false`, so
> `hardware.launch.py` with no arguments talks to real servos and runs the
> preflight check. Pass `fake:=true` deliberately when you want the mock
> backend. This asymmetry is a live footgun — check before launching.

Args: `fake` (default **false**), `port` (default `/dev/jupiter/arm`),
`baudrate` (1000000), `servo_speed` (2400), `servo_accel` (50),
`domain` (77), `arm_plugin_class`
(`rbc_so100_controller/RBCSOARM100Interface`).

Starts `robot_state_publisher`, `ros2_control_node`, and the controllers. Does
**not** start the base, lidar, camera, or head servo — those are deliberately
separate.

### Real hardware — base, lidar, head, camera

```bash
ros2 launch jupiterobot2_bringup jupiterobot2_core.launch.py     # base
ros2 launch jupiterobot2_bringup jupiterobot2_lidar.launch.py    # lidar
ros2 launch jupiterobot2_head_bringup head.launch.py             # head
ros2 launch astra_camera astra.launch.xml product_id:=0x065b     # camera
```

Head torque is **off until enabled**:

```bash
ros2 service call /head/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /head/joint_goal sensor_msgs/msg/JointState \
  "{name: [head_joint], position: [0.5]}"
```

Range is ±1.5 rad; larger values are clamped.

In Gazebo, use `/head_group_controller` (via ros2_control) instead — the servo
isn't there.

### Teleop

```bash
ros2 run jupiterobot2_teleop teleop_keyboard
ros2 launch jupiterobot2_teleop joystick_teleop.launch.py

# scripted
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

### Monitoring

```bash
ros2 launch jupiter_monitor_hub monitor.launch.py
monitor                    # TUI dashboard   (jupiter_monitor_tui)
panel                      # access panel: dashboard + per-subsystem control
monitor --once             # headless snapshot
monitor --once -v          # verbose
```

> `manifest_check` collides with jupitron's copy on `PATH`. Use the absolute
> path: `install/jupiter_monitor_hub/lib/jupiter_monitor_hub/manifest_check`.

### SLAM / navigation

`jupiterobot2_gazebo/launch/`: `slam.launch.py`, `slam_no_odom.launch.py`,
`slam_rviz.launch.py`, `slam_no_odom_rviz.launch.py`,
`hector_slam.launch.py`, `nav_standalone.launch.py`,
`jupiterobot2_navigation/launch/navigation.launch.py`.

These require `config/slam.rviz`; its absence used to produce a silently blank
RViz with no error.

---

## Related systems

| Path | What it is |
|---|---|
| `~/jupitron/` | ROS access layer exposing `/jupitron/*` topics |
| `~/project-jupiter/` | Knowledge base |
| `~/JUPITER_ROBOT_NOTES.md` | Working notes |
| `~/fyp1/` | SO-100 driver (`rbc_so100_hardware`), **install tree only** |
| `~/jupiter2_so100_deploy/` | Source bundle the arm mount + meshes came from |
| `~/jupiter_tui/` | Separate hardware TUI |
| `~/ft_scservo_debug_qt_fixed/` | Layout-fixed Feetech servo GUI |

---

## Changelog

### 2026-11-09 — SO-100 arm on real hardware working

**Fixed**
- `head_joint` removed from the real-hardware `ros2_control` block. The driver
  indexes servos by joint order, so 7 declared joints made it ping a
  non-existent servo 7 and abort `controller_manager`.
- `hardware.launch.py` now skips `head_group_controller` when `fake:=false`
  (real hardware), since that backend declares no `head_joint`; previously the
  spawner died with `head_joint/position Not existing`.
- `preflight_check.py` sequencing: it now runs **before** `ros2_control_node` via
  `OnProcessExit`, and its non-zero exit aborts the launch. Both bugs are
  documented — first it raced for the exclusive serial port, then it failed
  correctly but the launch continued anyway.

**Added**
- `so100_description/so100_description/safety.py` — margin rule as a single
  source of truth, with a self-check that validates `LIMITS` against the
  expanded URDF.
- `jupiterobot2_gazebo/scripts/preflight_check.py` — refuses to hand a non-parked
  arm to ros2_control.
- `jupiterobot2_gazebo/scripts/restore_rest.py` — parks drifted joints slowly,
  with `--dry-run`.

**Changed**
- `controllers.yaml` documents explicitly that there is no "go home on activate".

**Verified**
- Real arm: `shoulder_pan` 0.0048 → 0.0951 rad (commanded 0.10), `gripper` →
  0.2470 (commanded 0.25), both `SUCCEEDED`; `tcp` at `[0.229, 0.018, 0.587]`.
- Safe pose `[0, 0.35, −0.45, −0.05, 0]` → `error_code: 0`; return to rest
  within 0.01 rad; final state 0.0 % load, `err 0x00` on all six.
- Gazebo and fake backends still expand to exactly one `<ros2_control>`.

**Corrected**
- An earlier claim that the driver rewrites servo EEPROM limits on every launch
  was **wrong**; disproved by a before/after register dump (byte-identical).
  README corrected rather than silently edited.

### 2026-11-09 — Documentation and cleanup

- IMU noise blocks added (removed 6× `Get noise index not valid`); `/imu` now
  99 Hz. `self_collide` made explicit; `mu1`/`mu2` warnings eliminated.
- `config/slam.rviz` created; four launch files repointed at it
  (`slam_rviz`, `slam_no_odom_rviz`, `nav_standalone`, `rviz.launch`).
- Legacy AX-12 arm meshes moved to `jupiterobot2_gazebo/models/ax12_arm/` so
  RoboCup worlds keep working without a dead ROS package.
- `jupiter_monitor_hub/config/drivers.yaml` and Gazebo model paths repointed
  from `jupiterobot2_arm_description` to `so100_description`.

### Earlier — SO-100 arm integration and URDF reorganisation

- Created `so100_description`; replaced the AX-12 arm.
- Arm anchored to `plate_2_link`, not `arm_mount_link` (the latter is 134 mm
  outboard — see [knowledge](#so-100-arm)).
- Split URDF into `description` (pure geometry) and `gazebo` (all Gazebo XML).
  Enforced the three invariants above.
- Fixed `ParameterValue(..., value_type=str)` in every launch publishing
  `robot_description`.
- Fixed xacro comment traps (`${...}`, `--`).
- Renamed link `gripper` → `gripper_body` to resolve the Gazebo name collision.
- Restored `screen_link` and `pole_40_1` lost during the split; removed the
  empty `jupiterobot2_head_gazebo.xacro`.

---

## Known issues / not done yet

1. **No MoveIt config for the arm.** No IK; joint-angle goals only.
2. **Servo 1 EEPROM/URDF mismatch** (284..3355 vs 744..3352). Harmless now,
   blocks widening that URDF limit. Needs the Feetech tool.
3. **Bus wedging after a failed activate** has no automated recovery — requires
   physical replug.
4. **Rest ticks are hardcoded** in three places (`preflight_check.py`,
   `restore_rest.py`, `safety.py`). They should live in one config file.
5. **No CI, no tests.** `safety.py --self-check` is the only automated check.
6. **Large vendored assets** (`exp1` 108 MB, `ros2_astra_camera` 11 MB) will make
   the repo heavy. See below.
7. **Loose scratch files in `src/`** (`basic_syntac`, `self_location.py`,
   `time_stamp_fyp`, `jupiterobot2_voice.zip`) are not packages and not built.
8. **Voice/vision packages** are lightly verified compared to the arm stack.

---

## Gitignore and what not to commit

`.gitignore` is configured and **verified against the working tree**. What would
actually be staged right now:

```
341 files   18 MB total   largest file 1.5 MB
```

That is a comfortable public-repo size. Confirm before any push:

```bash
git add -An | wc -l                      # file count
git add -An | sed "s/^add '//;s/'$//" | while read f; do
  [ -f "$f" ] && stat -c%s "$f"; done | awk '{s+=$1} END {print s/1048576" MB"}'
```

Excluded on purpose:

| Path | Size | Why |
|---|---|---|
| `src/exp1/models/` | 107 MB | Vosk speech model — re-downloadable |
| `*.zip` | 39 MB | Includes the Vosk archive; also `jupiterobot2_voice.zip` |
| `*.pt`, `*.pth`, `*.onnx` | ~6 MB | `yolov8n.pt` and similar weights |
| `opencr_update*` | — | Base MCU firmware |
| `no_odom_maps.pgm/.yaml` | — | Generated map |
| `*.bag`, `*.db3`, `*.mcap` | — | Recorded data |
| `build/ install/ log/` | large | Build artifacts |

Verify any individual path:

```bash
git check-ignore -v <path>     # prints the matching rule
```

> GitHub hard-rejects any single file over 100 MB. Nothing tracked now comes
> close, but re-run the size check after adding meshes or datasets.

### Before the first push

The repo currently has **no commits** (`git log` → `does not have any commits
yet`) and no remote configured. A sensible sequence:

```bash
git add -A
git commit -m "Initial commit: Jupiter 2 ROS 2 Humble workspace"
git remote add origin git@github.com:<user>/<repo>.git
git push -u origin master
```

Add a `LICENSE` before publishing publicly — none is present, so the default is
all-rights-reserved.

---

## Environment notes for the next agent

- **Always** `unset VIRTUAL_ENV` and reset `PATH` before `colcon` or ROS
  commands. This is the single most repeated failure.
- Textual (used by `jupiter_monitor_tui`) is installed in `/usr/bin/python3`
  **on purpose** — a venv breaks `rclpy`.
- Check `ROS_DOMAIN_ID` before *anything*. 42 is the live robot.
- Verify claims about hardware with a probe before asserting them. Two separate
  bugs in this project came from trusting an inference over a measurement (the
  "arm is dead" conclusion, and the EEPROM-write claim).
- Do not commit, push, or rewrite history unless asked.
