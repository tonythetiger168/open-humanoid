#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Safety Monitor System

Functions:
  - Fall detection (base height + orientation)
  - Joint limit monitoring
  - Overcurrent / overtemperature protection
  - Emergency stop handling
  - Watchdog timer
  - Communication health check
"""
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum, auto
import time

class SafetyLevel(Enum):
    NORMAL = auto()
    WARNING = auto()
    CRITICAL = auto()
    EMERGENCY = auto()

@dataclass
class SafetyConfig:
    # Fall detection
    fall_base_z_threshold: float = 0.40
    fall_roll_threshold: float = 1.0  # rad (~57 deg)
    fall_pitch_threshold: float = 1.0

    # Joint limits
    joint_limit_margin: float = 0.05  # rad
    max_joint_velocity: float = 6.0   # rad/s
    max_joint_torque: float = 15.0   # Nm

    # Power
    battery_low_voltage: float = 22.0
    battery_critical_voltage: float = 20.0
    max_continuous_current: float = 40.0
    max_peak_current: float = 80.0
    temperature_warning: float = 70.0
    temperature_critical: float = 85.0

    # Timing
    emergency_stop_timeout_ms: float = 100
    watchdog_timeout_ms: float = 500
    communication_timeout_ms: float = 200

    # Recovery
    auto_recovery_attempts: int = 3
    recovery_delay_s: float = 2.0

class SafetyMonitor:
    def __init__(self, config: SafetyConfig = None):
        self.cfg = config or SafetyConfig()
        self.level = SafetyLevel.NORMAL
        self.emergency_active = False
        self.emergency_source = None
        self.watchdog_last_feed = time.time()
        self.communication_last_seen = time.time()
        self.recovery_attempts = 0
        self.fault_history: List[Dict] = []
        self.max_history = 100

    def check_fall(self, base_pos: np.ndarray, base_rpy: np.ndarray) -> bool:
        """Detect if robot has fallen."""
        if base_pos[2] < self.cfg.fall_base_z_threshold:
            self._trigger_fault("FALL", f"Base Z too low: {base_pos[2]:.3f}m")
            return True
        if abs(base_rpy[0]) > self.cfg.fall_roll_threshold:
            self._trigger_fault("FALL", f"Roll exceeded: {np.degrees(base_rpy[0]):.1f}deg")
            return True
        if abs(base_rpy[1]) > self.cfg.fall_pitch_threshold:
            self._trigger_fault("FALL", f"Pitch exceeded: {np.degrees(base_rpy[1]):.1f}deg")
            return True
        return False

    def check_joint_limits(self, joint_states: Dict[str, Dict], joint_limits: Dict[str, Tuple[float, float]]) -> List[str]:
        """Check if any joint is near or exceeding limits."""
        violations = []
        for name, state in joint_states.items():
            if name not in joint_limits:
                continue
            lower, upper = joint_limits[name]
            pos = state["position"]
            vel = state["velocity"]
            effort = state.get("effort", 0.0)

            # Position limit
            if pos < lower + self.cfg.joint_limit_margin or pos > upper - self.cfg.joint_limit_margin:
                violations.append(f"{name}: position near limit ({pos:.3f} rad)")
                self._trigger_warning("JOINT_LIMIT", f"{name} near limit: {pos:.3f} rad")

            # Velocity limit
            if abs(vel) > self.cfg.max_joint_velocity:
                violations.append(f"{name}: velocity exceeded ({vel:.3f} rad/s)")
                self._trigger_critical("JOINT_VEL", f"{name} velocity: {vel:.3f} rad/s")

            # Torque limit
            if abs(effort) > self.cfg.max_joint_torque:
                violations.append(f"{name}: torque exceeded ({effort:.3f} Nm)")
                self._trigger_critical("JOINT_TORQUE", f"{name} torque: {effort:.3f} Nm")

        return violations

    def check_power(self, battery_voltage: float, battery_current: float, 
                    temperatures: Dict[str, float]) -> bool:
        """Check power system health."""
        fault = False

        # Battery voltage
        if battery_voltage < self.cfg.battery_critical_voltage:
            self._trigger_emergency("BATTERY_CRITICAL", f"Voltage: {battery_voltage:.1f}V")
            fault = True
        elif battery_voltage < self.cfg.battery_low_voltage:
            self._trigger_warning("BATTERY_LOW", f"Voltage: {battery_voltage:.1f}V")

        # Current
        if battery_current > self.cfg.max_peak_current:
            self._trigger_emergency("OVERCURRENT", f"Current: {battery_current:.1f}A")
            fault = True
        elif battery_current > self.cfg.max_continuous_current:
            self._trigger_critical("OVERCURRENT", f"Current: {battery_current:.1f}A")
            fault = True

        # Temperatures
        for name, temp in temperatures.items():
            if temp > self.cfg.temperature_critical:
                self._trigger_emergency("OVERTEMP", f"{name}: {temp:.1f}C")
                fault = True
            elif temp > self.cfg.temperature_warning:
                self._trigger_warning("TEMP_HIGH", f"{name}: {temp:.1f}C")

        return fault

    def feed_watchdog(self):
        """Feed the watchdog timer."""
        self.watchdog_last_feed = time.time()

    def check_watchdog(self) -> bool:
        """Check if watchdog has timed out."""
        elapsed = (time.time() - self.watchdog_last_feed) * 1000
        if elapsed > self.cfg.watchdog_timeout_ms:
            self._trigger_emergency("WATCHDOG", f"Timeout: {elapsed:.0f}ms")
            return True
        return False

    def check_communication(self, last_message_time: float) -> bool:
        """Check if communication is healthy."""
        elapsed = (time.time() - last_message_time) * 1000
        if elapsed > self.cfg.communication_timeout_ms:
            self._trigger_critical("COMM_LOSS", f"No message for {elapsed:.0f}ms")
            return True
        return False

    def trigger_emergency_stop(self, source: str, detail: str = ""):
        """Manually trigger emergency stop."""
        self.emergency_active = True
        self.emergency_source = source
        self.level = SafetyLevel.EMERGENCY
        self._log_fault("EMERGENCY_STOP", f"Source: {source}, Detail: {detail}")

    def reset_emergency(self) -> bool:
        """Attempt to reset from emergency state."""
        if self.level != SafetyLevel.EMERGENCY:
            return True

        if self.recovery_attempts >= self.cfg.auto_recovery_attempts:
            self._log_fault("RECOVERY_FAILED", "Max recovery attempts exceeded")
            return False

        self.recovery_attempts += 1
        time.sleep(self.cfg.recovery_delay_s)

        # Check if conditions are safe
        if self.level == SafetyLevel.EMERGENCY:
            # Only reset if emergency source is resolved
            self.emergency_active = False
            self.emergency_source = None
            self.level = SafetyLevel.NORMAL
            self._log_fault("RECOVERY", f"Attempt {self.recovery_attempts} successful")
            return True

        return False

    def get_status(self) -> Dict:
        """Get current safety status."""
        return {
            "level": self.level.name,
            "emergency_active": self.emergency_active,
            "emergency_source": self.emergency_source,
            "recovery_attempts": self.recovery_attempts,
            "fault_count": len(self.fault_history),
            "watchdog_ms_ago": (time.time() - self.watchdog_last_feed) * 1000,
        }

    def _trigger_warning(self, code: str, detail: str):
        if self.level.value < SafetyLevel.WARNING.value:
            self.level = SafetyLevel.WARNING
        self._log_fault(code, detail)

    def _trigger_critical(self, code: str, detail: str):
        if self.level.value < SafetyLevel.CRITICAL.value:
            self.level = SafetyLevel.CRITICAL
        self._log_fault(code, detail)

    def _trigger_emergency(self, code: str, detail: str):
        self.level = SafetyLevel.EMERGENCY
        self.emergency_active = True
        self.emergency_source = code
        self._log_fault(code, detail)

    def _trigger_fault(self, code: str, detail: str):
        if self.level.value < SafetyLevel.CRITICAL.value:
            self.level = SafetyLevel.CRITICAL
        self._log_fault(code, detail)

    def _log_fault(self, code: str, detail: str):
        fault = {
            "timestamp": time.time(),
            "code": code,
            "detail": detail,
            "level": self.level.name,
        }
        self.fault_history.append(fault)
        if len(self.fault_history) > self.max_history:
            self.fault_history.pop(0)
        print(f"[SAFETY] {self.level.name}: {code} - {detail}")

# ============================================================
# Emergency Stop Handler (Hardware Interface)
# ============================================================
class EmergencyStopHandler:
    def __init__(self):
        self.stop_active = False
        self.hardware_pin = None  # GPIO pin for hardware E-stop
        self.software_triggered = False

    def init_hardware(self, gpio_pin: int = 18):
        """Initialize hardware emergency stop pin."""
        try:
            import RPi.GPIO as GPIO
            self.hardware_pin = gpio_pin
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(gpio_pin, GPIO.FALLING, callback=self._hw_callback, bouncetime=200)
            print(f"[E-Stop] Hardware E-stop initialized on GPIO {gpio_pin}")
        except ImportError:
            print("[E-Stop] RPi.GPIO not available, software-only mode")

    def _hw_callback(self, channel):
        self.stop_active = True
        self.software_triggered = False
        print("[E-Stop] HARDWARE EMERGENCY STOP TRIGGERED!")

    def trigger_software(self):
        self.stop_active = True
        self.software_triggered = True
        print("[E-Stop] SOFTWARE EMERGENCY STOP TRIGGERED!")

    def reset(self):
        if not self.software_triggered and self.hardware_pin is not None:
            # Cannot reset hardware stop without physical reset
            print("[E-Stop] Cannot reset: hardware stop active")
            return False
        self.stop_active = False
        self.software_triggered = False
        print("[E-Stop] Emergency stop reset")
        return True

    def is_active(self) -> bool:
        return self.stop_active
