#!/usr/bin/env python3
"""OpenHumanoid v1.0 - ODrive Pro Motor Driver Interface

Supports: CAN-FD communication, position/velocity/torque control,
          state reading, parameter configuration, fault handling.

Hardware: ODrive Pro (v4.x firmware)
Bus: CAN-FD @ 1 Mbps nominal, 4 Mbps data
"""
import numpy as np
import time
from typing import Dict, Tuple, Optional, Callable
from dataclasses import dataclass
from enum import Enum

# Try to import CAN library
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    print("[ODrive] python-can not installed, using mock mode")

class ControlMode(Enum):
    VOLTAGE = 0
    TORQUE = 1
    VELOCITY = 2
    POSITION = 3

class AxisState(Enum):
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_POLARITY_CALIBRATION = 12
    ENCODER_HALL_PHASE_CALIBRATION = 13

@dataclass
class ODriveConfig:
    node_id: int = 0
    can_channel: str = "can0"
    can_bustype: str = "socketcan"
    bitrate: int = 1000000
    data_bitrate: int = 4000000

    # Motor parameters
    pole_pairs: int = 21
    torque_constant: float = 0.095  # Nm/A
    current_limit: float = 20.0     # A
    velocity_limit: float = 50.0    # rad/s (motor side)

    # Control gains (to be tuned per joint)
    pos_gain: float = 50.0
    vel_gain: float = 0.15
    vel_integrator_gain: float = 0.3

    # Encoder
    cpr: int = 4096

    # Gearbox
    gear_ratio: float = 7.67

class ODriveAxis:
    """Single axis interface for ODrive Pro."""

    # CAN ID format: Node ID [0-63] | Command ID
    CMD_ID_HEARTBEAT = 0x001
    CMD_ID_ESTOP = 0x002
    CMD_ID_GET_ERRORS = 0x003
    CMD_ID_RX_SDO = 0x004
    CMD_ID_TX_SDO = 0x005
    CMD_ID_GET_BUS_VOLTAGE_CURRENT = 0x006
    CMD_ID_CLEAR_ERRORS = 0x007
    CMD_ID_SET_AXIS_STATE = 0x007
    CMD_ID_GET_IQ = 0x014
    CMD_ID_GET_TEMPERATURE = 0x015
    CMD_ID_GET_TORQUES = 0x01C
    CMD_ID_GET_POWERS = 0x01D
    CMD_ID_SET_INPUT_POS = 0x00C
    CMD_ID_SET_INPUT_VEL = 0x00D
    CMD_ID_SET_INPUT_TORQUE = 0x00E
    CMD_ID_SET_LIMITS = 0x00F
    CMD_ID_SET_POS_GAIN = 0x01A
    CMD_ID_SET_VEL_GAINS = 0x01B
    CMD_ID_SET_TRAJ_VEL_LIMIT = 0x011
    CMD_ID_SET_TRAJ_ACCEL_LIMITS = 0x012
    CMD_ID_SET_TRAJ_INERTIA = 0x013
    CMD_ID_GET_ENCODER_ESTIMATES = 0x009
    CMD_ID_GET_SENSORLESS_ESTIMATES = 0x00A
    CMD_ID_GET_ENCODER_COUNT = 0x00B
    CMD_ID_SET_LINEAR_COUNT = 0x019
    CMD_ID_SET_POSITION = 0x00C
    CMD_ID_SET_VELOCITY = 0x00D
    CMD_ID_SET_TORQUE = 0x00E

    def __init__(self, config: ODriveConfig = None):
        self.cfg = config or ODriveConfig()
        self.bus = None
        self.connected = False

        # State
        self.pos_estimate = 0.0      # rad (output side)
        self.vel_estimate = 0.0      # rad/s
        self.torque_estimate = 0.0   # Nm
        self.current_q = 0.0         # A
        self.current_d = 0.0         # A
        self.bus_voltage = 0.0       # V
        self.bus_current = 0.0       # A
        self.motor_temp = 0.0        # C
        self.inverter_temp = 0.0     # C
        self.axis_state = AxisState.UNDEFINED
        self.axis_error = 0
        self.motor_error = 0
        self.encoder_error = 0
        self.controller_error = 0

        # Tracking
        self.pos_setpoint = 0.0
        self.vel_setpoint = 0.0
        self.torque_setpoint = 0.0

    def connect(self) -> bool:
        """Connect to CAN bus."""
        if not CAN_AVAILABLE:
            print("[ODrive] Running in MOCK mode (no CAN hardware)")
            self.connected = True
            return True

        try:
            self.bus = can.interface.Bus(
                channel=self.cfg.can_channel,
                bustype=self.cfg.can_bustype,
                bitrate=self.cfg.bitrate,
                data_bitrate=self.cfg.data_bitrate,
                fd=True
            )
            self.connected = True
            print(f"[ODrive] Connected to {self.cfg.can_channel}")
            return True
        except Exception as e:
            print(f"[ODrive] Connection failed: {e}")
            return False

    def disconnect(self):
        if self.bus:
            self.bus.shutdown()
        self.connected = False

    def _send_can(self, cmd_id: int, data: bytes, is_extended: bool = False):
        """Send CAN message."""
        if not self.connected:
            return

        if CAN_AVAILABLE and self.bus:
            arb_id = (self.cfg.node_id << 5) | cmd_id
            msg = can.Message(
                arbitration_id=arb_id,
                data=data,
                is_extended_id=is_extended,
                is_fd=True
            )
            self.bus.send(msg)
        else:
            # Mock mode: simulate response
            pass

    def _recv_can(self, timeout: float = 0.1) -> Optional[can.Message]:
        """Receive CAN message."""
        if not CAN_AVAILABLE or not self.bus:
            return None
        return self.bus.recv(timeout)

    def set_axis_state(self, state: AxisState):
        """Set axis state (e.g., CLOSED_LOOP_CONTROL)."""
        data = state.value.to_bytes(4, 'little', signed=False)
        self._send_can(self.CMD_ID_SET_AXIS_STATE, data)
        self.axis_state = state
        print(f"[ODrive] Axis state -> {state.name}")

    def set_position(self, position: float, velocity_ff: float = 0.0, torque_ff: float = 0.0):
        """Set position setpoint (rad, output side)."""
        # Convert to motor-side counts
        pos_motor = position * self.cfg.gear_ratio
        vel_ff_motor = velocity_ff * self.cfg.gear_ratio

        # Pack as IEEE 754 floats (little endian)
        import struct
        data = struct.pack('<fff', pos_motor, vel_ff_motor, torque_ff)
        self._send_can(self.CMD_ID_SET_INPUT_POS, data)
        self.pos_setpoint = position

    def set_velocity(self, velocity: float, torque_ff: float = 0.0):
        """Set velocity setpoint (rad/s, output side)."""
        vel_motor = velocity * self.cfg.gear_ratio
        import struct
        data = struct.pack('<ff', vel_motor, torque_ff)
        self._send_can(self.CMD_ID_SET_INPUT_VEL, data)
        self.vel_setpoint = velocity

    def set_torque(self, torque: float):
        """Set torque setpoint (Nm, output side)."""
        import struct
        data = struct.pack('<f', torque)
        self._send_can(self.CMD_ID_SET_INPUT_TORQUE, data)
        self.torque_setpoint = torque

    def set_limits(self, velocity_limit: float, current_limit: float):
        """Set velocity and current limits."""
        import struct
        vel_motor = velocity_limit * self.cfg.gear_ratio
        data = struct.pack('<ff', vel_motor, current_limit)
        self._send_can(self.CMD_ID_SET_LIMITS, data)

    def set_gains(self, pos_gain: float, vel_gain: float, vel_integrator_gain: float):
        """Set control gains."""
        import struct
        data = struct.pack('<fff', pos_gain, vel_gain, vel_integrator_gain)
        self._send_can(self.CMD_ID_SET_POS_GAIN, data)
        self.cfg.pos_gain = pos_gain
        self.cfg.vel_gain = vel_gain
        self.cfg.vel_integrator_gain = vel_integrator_gain

    def set_linear_count(self, count: int):
        """Set encoder linear count (for zeroing)."""
        data = count.to_bytes(4, 'little', signed=True)
        self._send_can(self.CMD_ID_SET_LINEAR_COUNT, data)

    def clear_errors(self):
        """Clear all axis errors."""
        self._send_can(self.CMD_ID_CLEAR_ERRORS, b'\x00\x00\x00\x00\x00\x00\x00\x00')
        self.axis_error = 0
        self.motor_error = 0
        self.encoder_error = 0
        self.controller_error = 0

    def estop(self):
        """Trigger emergency stop."""
        self._send_can(self.CMD_ID_ESTOP, b'\x00\x00\x00\x00\x00\x00\x00\x00')
        print("[ODrive] ESTOP sent")

    def read_state(self) -> Dict:
        """Read current axis state from CAN."""
        if not CAN_AVAILABLE:
            # Mock mode: return simulated state
            return self._mock_state()

        # Request encoder estimates
        self._send_can(self.CMD_ID_GET_ENCODER_ESTIMATES, b'')
        msg = self._recv_can(timeout=0.05)
        if msg:
            import struct
            pos, vel = struct.unpack('<ff', msg.data[:8])
            self.pos_estimate = pos / self.cfg.gear_ratio
            self.vel_estimate = vel / self.cfg.gear_ratio

        # Request torques
        self._send_can(self.CMD_ID_GET_TORQUES, b'')
        msg = self._recv_can(timeout=0.05)
        if msg:
            import struct
            self.torque_estimate = struct.unpack('<f', msg.data[:4])[0]

        # Request temperatures
        self._send_can(self.CMD_ID_GET_TEMPERATURE, b'')
        msg = self._recv_can(timeout=0.05)
        if msg:
            import struct
            self.motor_temp, self.inverter_temp = struct.unpack('<ff', msg.data[:8])

        return self.get_state_dict()

    def _mock_state(self) -> Dict:
        """Simulate state for testing without hardware."""
        # Simple physics simulation
        dt = 0.01
        kp, kd = self.cfg.pos_gain, self.cfg.vel_gain

        pos_err = self.pos_setpoint - self.pos_estimate
        vel_cmd = kp * pos_err + self.vel_setpoint
        vel_err = vel_cmd - self.vel_estimate
        torque = self.cfg.torque_constant * (kd * vel_err + self.torque_setpoint)
        torque = np.clip(torque, -self.cfg.current_limit * self.cfg.torque_constant,
                         self.cfg.current_limit * self.cfg.torque_constant)

        # Simple motor dynamics: J*ddq = torque - friction
        J = 0.001  # reflected inertia (kg*m^2)
        friction = 0.1 * np.sign(self.vel_estimate) + 0.01 * self.vel_estimate
        accel = (torque - friction) / J
        self.vel_estimate += accel * dt
        self.pos_estimate += self.vel_estimate * dt
        self.torque_estimate = torque
        self.motor_temp = 25.0 + abs(torque) * 2.0

        return self.get_state_dict()

    def get_state_dict(self) -> Dict:
        return {
            "pos_estimate": self.pos_estimate,
            "vel_estimate": self.vel_estimate,
            "torque_estimate": self.torque_estimate,
            "motor_temp": self.motor_temp,
            "inverter_temp": self.inverter_temp,
            "axis_state": self.axis_state.name,
            "axis_error": self.axis_error,
            "pos_setpoint": self.pos_setpoint,
            "vel_setpoint": self.vel_setpoint,
            "torque_setpoint": self.torque_setpoint,
        }

    def full_calibration(self):
        """Run full calibration sequence."""
        print("[ODrive] Starting full calibration...")
        self.set_axis_state(AxisState.FULL_CALIBRATION_SEQUENCE)
        time.sleep(15)  # Calibration takes ~10-15s
        self.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
        print("[ODrive] Calibration complete, entering closed loop")

    def quick_calibration(self):
        """Quick calibration (assumes motor already characterized)."""
        print("[ODrive] Quick calibration...")
        self.set_axis_state(AxisState.ENCODER_OFFSET_CALIBRATION)
        time.sleep(5)
        self.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
        print("[ODrive] Quick calibration complete")

# ============================================================
# Multi-axis manager for a single ODrive Pro (2 axes)
# ============================================================
class ODrivePro:
    def __init__(self, node_id: int = 0):
        self.node_id = node_id
        self.axis0 = ODriveAxis(ODriveConfig(node_id=node_id, node_axis=0))
        self.axis1 = ODriveAxis(ODriveConfig(node_id=node_id, node_axis=1))
        self.axes = [self.axis0, self.axis1]

    def connect(self) -> bool:
        return self.axis0.connect()

    def disconnect(self):
        self.axis0.disconnect()

    def calibrate_all(self):
        for i, axis in enumerate(self.axes):
            print(f"[ODrive] Calibrating axis {i}...")
            axis.full_calibration()

    def get_all_states(self) -> Dict:
        return {
            "axis0": self.axis0.read_state(),
            "axis1": self.axis1.read_state(),
        }
