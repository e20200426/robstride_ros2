# rob_py

A minimal ROS2 Python package (`ament_python`) for controlling **RobStride** brushless motors over SocketCAN.

## Package layout

```
rob_py/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/rob_py
├── rob_py/
│   ├── can_setup.py          # CAN interface bring-up helper
│   ├── mit_control_node.py   # MIT-mode position control (interactive)
│   ├── pp_control_node.py    # Profile-Position mode control (interactive)
│   ├── vel_control_node.py   # Velocity-mode control (interactive)
│   └── motor_scan_node.py    # CAN bus motor scanner
└── robstride_dynamics/       # bundled SDK (bus, protocol, tables)
    ├── bus.py                # RobstrideBus class (connect, enable, control methods)
    ├── protocol.py           # CommunicationType & ParameterType constants
    └── table.py              # Per-model limits (position, velocity, torque, kp, kd)
```

---

## Prerequisites

| Requirement | Notes |
|---|---|
| ROS2 (Humble / Iron / Jazzy) | `source /opt/ros/<distro>/setup.bash` |
| `python-can` | `pip install python-can` |
| `numpy`, `tqdm` | `pip install numpy tqdm` |
| `can-utils` | `sudo apt install can-utils` |
| SocketCAN hardware | USB-CAN adapter or on-board CAN |
| Passwordless `sudo` for `ip` | See [CAN setup](#can-interface-setup) |

---

## Installation

### Clone the repository

```bash
git clone https://github.com/e20200426/robstride_ros2.git
cd robstride_ros2
```

## Build

```bash
# Source your ROS2 distro first
source /opt/ros/humble/setup.bash 

cd robstride_ros2
colcon build --packages-select rob_py
source install/setup.bash
```

---

## CAN interface setup

Every node **AUTOMATICALLY** runs:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

before opening the bus. For this to work without a password prompt, add one line to `/etc/sudoers` (run `sudo visudo`):

```
<your_username>  ALL=(ALL) NOPASSWD: /sbin/ip
```

If the interface is already `UP` the commands are skipped.

---

## Nodes

### `mit_control_node` — interactive MIT-mode position control

Connects to a single motor, switches it to **MIT mode (Mode 0)**, holds it at **0 rad**, then lets you type new target positions at any time.

#### Configuration

Edit the constants at the top of [rob_py/mit_control_node.py](rob_py/mit_control_node.py):

```python
CAN_CHANNEL  = 'can0'       # SocketCAN interface name
BITRATE      = 1_000_000    # CAN bitrate (bps)
MOTOR_ID     = 1            # motor ID on the bus
MOTOR_MODEL  = 'rs-00'      # RobStride model string
KP           = 30.0         # MIT stiffness (Nm/rad)
KD           = 5.0          # MIT damping   (Nm·s/rad)
LOOP_RATE_HZ = 100          # control-loop frequency (Hz)
```

#### Run

```bash
ros2 run rob_py mit_control_node
```

#### Usage

```
[INFO] MIT mode active — motor 1 on can0
Enter target position in rad (or 'q' to quit):
>
  pos=+0.0003 rad  vel=+0.0001 rad/s  trq=+0.0000 Nm  temp=25.0°C
> 1.57
  → target set to 1.5700 rad
> -0.5
  → target set to -0.5000 rad
> q
[INFO] Stopping motor ...
[INFO] Done.
```

- Type a **number** (radians) and press Enter to move the motor.
- Type **`q`** or press **Ctrl+C** to safely stop and disable the motor.

---

### `pp_control_node` — interactive Profile-Position mode control

Connects to a single motor, switches it to **Profile-Position mode (Mode 1)**, configures a smooth motion profile, then lets you type absolute target positions. The motor uses its internal trapezoidal profiler to move smoothly between positions.

#### Configuration

Edit the constants at the top of [rob_py/pp_control_node.py](rob_py/pp_control_node.py):

```python
CAN_CHANNEL       = 'can0'      # SocketCAN interface name
BITRATE           = 1_000_000   # CAN bitrate (bps)
MOTOR_ID          = 3           # motor ID on the bus
MOTOR_MODEL       = 'rs-00'     # RobStride model string
PP_VELOCITY_MAX   = 20.0        # profile speed limit (rad/s)
PP_ACCELERATION   = 10.0        # profile acceleration (rad/s²)
TORQUE_LIMIT      = 2.0         # max torque during travel (Nm)
FEEDBACK_RATE_HZ  = 50          # status-poll frequency (Hz)
```

#### Run

```bash
ros2 run rob_py pp_control_node
```

#### Usage

```
[INFO] PP mode active — motor 3 on can0
       vel_max=20.0 rad/s  acc=10.0 rad/s²  torque_limit=2.0 Nm
Enter target position in rad (or 'q' to quit):
> 1.57
  → target set to 1.5700 rad
  pos=+1.5698 rad  vel=+0.0021 rad/s  trq=+0.1100 Nm  temp=27.1°C
> -1.57
  → target set to -1.5700 rad
> q
[INFO] Returning to home position ...
[INFO] Done.
```

- Type a **number** (radians) and press Enter to trigger a profiled move.
- Type **`q`** or press **Ctrl+C** to return to 0 rad and disable the motor.

---

### `vel_control_node` — interactive velocity-mode control

Connects to a single motor, switches it to **Velocity mode (Mode 2)**, then lets you type target speeds at any time. The motor's internal controller maintains the commanded speed.

#### Configuration

Edit the constants at the top of [rob_py/vel_control_node.py](rob_py/vel_control_node.py):

```python
CAN_CHANNEL    = 'can0'       # SocketCAN interface name
BITRATE        = 1_000_000    # CAN bitrate (bps)
MOTOR_ID       = 3            # motor ID on the bus
MOTOR_MODEL    = 'rs-00'      # RobStride model string
TORQUE_LIMIT   = 2.0          # max current in velocity mode (A)
LOOP_RATE_HZ   = 100          # control-loop frequency (Hz)
```

#### Run

```bash
ros2 run rob_py vel_control_node
```

#### Usage

```
[INFO] Velocity mode active — motor 3 on can0
Enter target velocity in rad/s (or 'q' to quit):
> 5.0
  → target set to 5.0000 rad/s
  pos=+1.2345 rad  vel=+5.0012 rad/s  trq=+0.4200 Nm  temp=26.3°C
> 0
  → target set to 0.0000 rad/s
> q
[INFO] Stopping motor ...
[INFO] Done.
```

- Type a **number** (rad/s) and press Enter to change speed.
- Type **`q`** or press **Ctrl+C** to safely ramp to zero and disable.

---

### `motor_scan_node` — CAN bus scanner

Scans the bus for all responding motor IDs and publishes the result as JSON, then exits.

#### Run

```bash
ros2 run rob_py motor_scan_node
```

Pass optional arguments:

```bash
ros2 run rob_py motor_scan_node --ros-args \
  -p can_channel:=can0 \
  -p start_id:=1 \
  -p end_id:=255 \
  -p bitrate:=1000000
```

#### Published topic

| Topic | Type | Content |
|---|---|---|
| `/rob_py/scan_result` | `std_msgs/String` | JSON object `{"<id>": "<uuid_hex>", ...}` |

Subscribe in another terminal:

```bash
ros2 topic echo /rob_py/scan_result
```

Example output:

```json
{"1": "a1b2c3d4e5f60102", "3": "deadbeef01020304"}
```

---

## can_setup helper

`rob_py.can_setup` can also be used directly in your own scripts:

```python
from rob_py.can_setup import setup_can_interface, teardown_can_interface

setup_can_interface('can0', bitrate=1_000_000)   # brings interface UP
# ... your code ...
teardown_can_interface('can0')                   # brings interface DOWN
```

---

## RobstrideBus API

The `robstride_dynamics.RobstrideBus` class exposes all motor control modes as first-class methods so you can drive motors from any script without handling raw CAN frames.

### Mode setup methods

These must be called while the motor is **disabled**. They configure the run mode and immediately re-enable the motor.

```python
from robstride_dynamics import RobstrideBus, Motor

motors = {'motor_3': Motor(id=3, model='rs-00')}
bus = RobstrideBus('can0', motors)
bus.connect()

# MIT mode (Mode 0) — set_run_mode only; enable separately
bus.set_run_mode('motor_3', mode=0)
bus.enable('motor_3')

# Profile-Position mode (Mode 1) — sets mode, enables, configures profile
bus.set_pp_mode('motor_3', vel_max=20.0, acceleration=10.0, torque_limit=2.0)

# Velocity mode (Mode 2) — sets mode, enables, sets torque limit
bus.set_velocity_mode('motor_3', torque_limit=2.0)
```

| Method | Mode | Description |
|---|---|---|
| `set_run_mode(motor, mode)` | any | Low-level write of `run_mode` parameter |
| `set_pp_mode(motor, vel_max, acceleration, torque_limit)` | 1 | Full PP mode setup |
| `set_velocity_mode(motor, torque_limit=2.0)` | 2 | Full velocity mode setup |

### Control methods

All control methods return a `(position, velocity, torque, temperature)` tuple from the motor's status frame.

```python
# MIT mode
bus.control_mit('motor_3', position=1.57, velocity=0.0, kp=30.0, kd=5.0, torque=0.0)

# Velocity mode
pos, vel, trq, temp = bus.control_velocity('motor_3', velocity=5.0)   # rad/s

# Profile-Position mode
pos, vel, trq, temp = bus.control_pp('motor_3', position=1.57)        # rad
```

| Method | Returns | Description |
|---|---|---|
| `control_mit(motor, position, velocity, kp, kd, torque)` | `(pos, vel, trq, temp)` | MIT torque/position control |
| `control_velocity(motor, velocity)` | `(pos, vel, trq, temp)` | Velocity setpoint |
| `control_pp(motor, position)` | `(pos, vel, trq, temp)` | PP position setpoint |

---

## Supported motor models

| Model string | Notes |
|---|---|
| `rs-00` | RobStride 00 |
| `rs-01` | RobStride 01 |
| `rs-02` | RobStride 02 |
| `rs-03` | RobStride 03 |
| `rs-06` | RobStride 06 |

Model strings must match the keys defined in `robstride_dynamics/table.py`.
