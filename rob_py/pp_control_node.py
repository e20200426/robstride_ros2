#!/usr/bin/env python3
"""
pp_control_node.py
------------------
Profile-Position (PP, Mode 1) motor control — no ROS topics.
The motor uses its internal motion profiler to move smoothly between
positions; the node sends target positions and displays live feedback.

Edit the constants below, then run:

    ros2 run rob_py pp_control_node
"""

import signal
import struct
import threading
import time

import rclpy

from rob_py.can_setup import setup_can_interface
from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType

# ── Configuration ─────────────────────────────────────────────────────────────
CAN_CHANNEL          = 'can0'
BITRATE              = 1_000_000   # bps
MOTOR_ID             = 3
MOTOR_MODEL          = 'rs-00'
PP_VELOCITY_MAX      = 20.0        # rad/s  — profile speed limit
PP_ACCELERATION      = 10.0        # rad/s² — profile acceleration
TORQUE_LIMIT         = 2.0         # Nm     — max torque during travel
FEEDBACK_RATE_HZ     = 50          # status-poll frequency
# ──────────────────────────────────────────────────────────────────────────────


def _transmit_param_f32(bus: RobstrideBus, motor_id: int, param_id: int, value: float):
    """Write a float32 parameter without blocking on a response."""
    value_buffer = struct.pack('<f', value)
    data = struct.pack('<HH', param_id, 0x00) + value_buffer
    bus.transmit(CommunicationType.WRITE_PARAMETER, bus.host_id, motor_id, data)


def set_pp_mode(bus: RobstrideBus, motor_name: str):
    """Switch motor to PP mode (run_mode = 1) while it is disabled."""
    param_id, _dtype, _ = ParameterType.MODE
    value_buffer = struct.pack('<bBH', 1, 0, 0)
    data = struct.pack('<HH', param_id, 0x00) + value_buffer
    bus.transmit(
        CommunicationType.WRITE_PARAMETER,
        bus.host_id,
        bus.motors[motor_name].id,
        data,
    )
    time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    if not setup_can_interface(CAN_CHANNEL, BITRATE):
        print(f"[ERROR] Could not bring up CAN interface '{CAN_CHANNEL}'. Exiting.")
        rclpy.shutdown()
        return

    motor_name  = f'motor_{MOTOR_ID}'
    motors      = {motor_name: Motor(id=MOTOR_ID, model=MOTOR_MODEL)}
    calibration = {motor_name: {'direction': 1, 'homing_offset': 0.0}}

    bus = RobstrideBus(CAN_CHANNEL, motors, calibration)
    bus.connect(handshake=True)

    # Must be in disabled state before changing run mode
    try:
        bus.disable(motor_name)
    except Exception:
        pass
    time.sleep(0.3)

    set_pp_mode(bus, motor_name)
    time.sleep(0.3)

    bus.enable(motor_name)
    time.sleep(0.3)

    # Configure profile parameters (set once after enabling)
    motor_id = bus.motors[motor_name].id
    _transmit_param_f32(bus, motor_id, ParameterType.PP_VELOCITY_MAX[0],     PP_VELOCITY_MAX)
    time.sleep(0.05)
    _transmit_param_f32(bus, motor_id, ParameterType.PP_ACCELERATION_TARGET[0], PP_ACCELERATION)
    time.sleep(0.05)
    _transmit_param_f32(bus, motor_id, ParameterType.TORQUE_LIMIT[0],        TORQUE_LIMIT)
    time.sleep(0.05)

    print(f"[INFO] PP mode active — motor {MOTOR_ID} on {CAN_CHANNEL}")
    print(f"       vel_max={PP_VELOCITY_MAX} rad/s  acc={PP_ACCELERATION} rad/s²  "
          f"torque_limit={TORQUE_LIMIT} Nm")

    running = True
    target_position = 0.0   # rad

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    def _input_thread():
        nonlocal target_position, running
        print("Enter target position in rad (or 'q' to quit):")
        while running:
            try:
                line = input("> ").strip()
                if line.lower() == 'q':
                    running = False
                    break
                target_position = float(line)
                print(f"  → target set to {target_position:.4f} rad")
            except ValueError:
                print("  Invalid input. Enter a number in radians.")
            except EOFError:
                running = False
                break

    input_t = threading.Thread(target=_input_thread, daemon=True)
    input_t.start()

    dt = 1.0 / FEEDBACK_RATE_HZ

    while running:
        try:
            # Continuously write the position target; the motor holds position if
            # the target is unchanged, and the write always returns a status frame.
            pos, vel, trq, temp = bus.write(
                motor_name, ParameterType.POSITION_TARGET, float(target_position)
            )
            print(
                f"\r  pos={pos:+.4f} rad  vel={vel:+.4f} rad/s  "
                f"trq={trq:+.4f} Nm  temp={temp:.1f}°C   ",
                end='', flush=True,
            )
        except Exception as exc:
            print(f"\n[WARN] Loop error: {exc}")

        time.sleep(dt)

    print("\n[INFO] Returning to home position ...")
    try:
        bus.write(motor_name, ParameterType.POSITION_TARGET, 0.0)
        time.sleep(0.8)
        bus.disable(motor_name)
        bus.disconnect()
    except Exception:
        pass

    rclpy.shutdown()
    print("[INFO] Done.")


if __name__ == '__main__':
    main()
