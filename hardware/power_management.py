#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Power Management System

Monitors:
  - Battery voltage, current, SOC, temperature
  - Power distribution to subsystems
  - Energy consumption estimation
  - Low battery protection
  - Charging management
"""
import numpy as np
from typing import Dict, Tuple
from dataclasses import dataclass
import time

@dataclass
class BatteryConfig:
    nominal_voltage: float = 24.0      # V
    nominal_capacity_ah: float = 20.0  # Ah
    cell_count: int = 8                # 8S LiFePO4
    cell_nominal_v: float = 3.2        # V per cell
    cell_max_v: float = 3.65           # V per cell
    cell_min_v: float = 2.5            # V per cell
    internal_resistance: float = 0.02  # Ohms
    max_discharge_current: float = 40.0  # A
    max_charge_current: float = 20.0   # A
    temperature_min: float = -10.0     # C
    temperature_max: float = 60.0      # C

    # Power budget (W)
    compute_power: float = 60.0        # Jetson AGX Orin
    motor_max_power: float = 500.0     # All 28 motors
    sensor_power: float = 15.0         # Cameras, IMU, etc.
    communication_power: float = 10.0  # CAN, WiFi, etc.
    cooling_power: float = 20.0        # Fans
    misc_power: float = 20.0           # LEDs, displays, etc.

class BatteryManager:
    def __init__(self, config: BatteryConfig = None):
        self.cfg = config or BatteryConfig()
        self.voltage = self.cfg.nominal_voltage
        self.current = 0.0
        self.temperature = 25.0
        self.soc = 100.0  # State of charge (%)
        self.soh = 100.0  # State of health (%)
        self.cycle_count = 0

        # Coulomb counting
        self.capacity_remaining_ah = self.cfg.nominal_capacity_ah
        self.total_discharged_ah = 0.0
        self.last_update = time.time()

        # Power tracking
        self.instant_power = 0.0
        self.energy_consumed_wh = 0.0
        self.energy_history: List[Tuple[float, float]] = []  # (time, power)

    def update(self, voltage: float, current: float, temperature: float):
        """Update battery state with new measurements."""
        dt = time.time() - self.last_update
        self.last_update = time.time()

        self.voltage = voltage
        self.current = current
        self.temperature = temperature
        self.instant_power = voltage * current

        # Coulomb counting for SOC
        if current > 0:  # Discharging
            self.capacity_remaining_ah -= current * dt / 3600.0
            self.total_discharged_ah += current * dt / 3600.0
        else:  # Charging
            self.capacity_remaining_ah -= current * dt / 3600.0

        self.capacity_remaining_ah = np.clip(self.capacity_remaining_ah, 0, self.cfg.nominal_capacity_ah)
        self.soc = (self.capacity_remaining_ah / self.cfg.nominal_capacity_ah) * 100.0

        # Energy tracking
        energy_wh = self.instant_power * dt / 3600.0
        self.energy_consumed_wh += energy_wh
        self.energy_history.append((time.time(), self.instant_power))

        # Trim history (keep last hour)
        cutoff = time.time() - 3600
        self.energy_history = [(t, p) for t, p in self.energy_history if t > cutoff]

        # Cycle counting
        if self.total_discharged_ah >= self.cfg.nominal_capacity_ah:
            self.cycle_count += 1
            self.total_discharged_ah -= self.cfg.nominal_capacity_ah

    def estimate_runtime(self, average_power: float = None) -> float:
        """Estimate remaining runtime in minutes."""
        if average_power is None:
            # Use recent average
            if len(self.energy_history) > 10:
                recent_power = np.mean([p for _, p in self.energy_history[-100:]])
            else:
                recent_power = self.instant_power
        else:
            recent_power = average_power

        if recent_power <= 0:
            return float('inf')

        remaining_wh = self.capacity_remaining_ah * self.voltage
        runtime_h = remaining_wh / recent_power
        return runtime_h * 60.0  # minutes

    def get_power_budget(self) -> Dict[str, float]:
        """Get current power consumption breakdown."""
        total = (self.cfg.compute_power + self.cfg.sensor_power + 
                 self.cfg.communication_power + self.cfg.cooling_power + 
                 self.cfg.misc_power)

        # Motor power varies with activity
        motor_power = min(self.instant_power - total, self.cfg.motor_max_power)
        motor_power = max(0, motor_power)

        return {
            "compute": self.cfg.compute_power,
            "motors": motor_power,
            "sensors": self.cfg.sensor_power,
            "communication": self.cfg.communication_power,
            "cooling": self.cfg.cooling_power,
            "misc": self.cfg.misc_power,
            "total": total + motor_power,
            "available": self.instant_power,
        }

    def check_health(self) -> Tuple[bool, List[str]]:
        """Check battery health and return warnings."""
        warnings = []
        critical = False

        # Voltage
        min_v = self.cfg.cell_min_v * self.cfg.cell_count
        max_v = self.cfg.cell_max_v * self.cfg.cell_count
        if self.voltage < min_v:
            warnings.append(f"CRITICAL: Battery voltage {self.voltage:.1f}V below minimum {min_v:.1f}V")
            critical = True
        elif self.voltage < min_v + 2.0:
            warnings.append(f"WARNING: Battery voltage low {self.voltage:.1f}V")

        # Temperature
        if self.temperature > self.cfg.temperature_max:
            warnings.append(f"CRITICAL: Battery temp {self.temperature:.1f}C exceeds max {self.cfg.temperature_max:.1f}C")
            critical = True
        elif self.temperature > self.cfg.temperature_max - 10:
            warnings.append(f"WARNING: Battery temp high {self.temperature:.1f}C")
        elif self.temperature < self.cfg.temperature_min:
            warnings.append(f"WARNING: Battery temp low {self.temperature:.1f}C")

        # SOC
        if self.soc < 10:
            warnings.append(f"CRITICAL: Battery SOC {self.soc:.1f}% critically low")
            critical = True
        elif self.soc < 20:
            warnings.append(f"WARNING: Battery SOC low {self.soc:.1f}%")

        # Current
        if self.current > self.cfg.max_discharge_current:
            warnings.append(f"CRITICAL: Discharge current {self.current:.1f}A exceeds max {self.cfg.max_discharge_current:.1f}A")
            critical = True

        # SOH
        if self.soh < 80:
            warnings.append(f"WARNING: Battery health degraded {self.soh:.1f}%")

        return critical, warnings

    def get_status(self) -> Dict:
        """Get complete battery status."""
        critical, warnings = self.check_health()
        runtime = self.estimate_runtime()

        return {
            "voltage": self.voltage,
            "current": self.current,
            "power": self.instant_power,
            "temperature": self.temperature,
            "soc": self.soc,
            "soh": self.soh,
            "cycle_count": self.cycle_count,
            "capacity_remaining_ah": self.capacity_remaining_ah,
            "energy_consumed_wh": self.energy_consumed_wh,
            "estimated_runtime_min": runtime,
            "critical": critical,
            "warnings": warnings,
        }

class PowerDistributor:
    """Manages power distribution to subsystems with priority."""
    def __init__(self):
        self.subsystems = {
            "compute": {"priority": 1, "enabled": True, "current_limit": 3.0},    # Jetson
            "motors": {"priority": 2, "enabled": True, "current_limit": 25.0},    # All joints
            "sensors": {"priority": 3, "enabled": True, "current_limit": 1.0},     # Cameras, IMU
            "communication": {"priority": 3, "enabled": True, "current_limit": 0.5},  # CAN, WiFi
            "cooling": {"priority": 4, "enabled": True, "current_limit": 1.0},     # Fans
            "misc": {"priority": 5, "enabled": True, "current_limit": 1.0},        # LEDs, etc.
        }

    def set_emergency_mode(self):
        """Disable non-essential subsystems during emergency."""
        for name, sub in self.subsystems.items():
            if sub["priority"] > 2:
                sub["enabled"] = False
                print(f"[Power] Disabled {name} for emergency mode")

    def set_low_power_mode(self):
        """Reduce power consumption for extended operation."""
        self.subsystems["cooling"]["enabled"] = False
        self.subsystems["misc"]["enabled"] = False
        self.subsystems["compute"]["current_limit"] = 2.0  # Reduce Jetson power
        print("[Power] Entered low power mode")

    def get_status(self) -> Dict:
        return {name: {"enabled": sub["enabled"], "limit": sub["current_limit"]} 
                for name, sub in self.subsystems.items()}
