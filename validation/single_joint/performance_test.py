#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Single Joint Performance Test Suite

Tests:
  1. Step response (position/velocity)
  2. Sinusoidal tracking (frequency sweep)
  3. Bode plot estimation
  4. Max velocity test
  5. Max torque test
  6. Thermal test
"""
import numpy as np
import time
import json
from typing import Dict, List
from dataclasses import dataclass
import matplotlib.pyplot as plt

@dataclass
class PerfTestConfig:
    step_amplitude: float = 0.5        # rad
    step_settle_threshold: float = 0.02  # rad (2% of amplitude)
    sine_amplitude: float = 0.3        # rad
    sine_frequencies: List[float] = None
    max_velocity_test: float = 5.0     # rad/s
    max_torque_test: float = 10.0      # Nm
    thermal_duration: float = 300.0    # seconds
    thermal_sample_interval: float = 5.0

    def __post_init__(self):
        if self.sine_frequencies is None:
            self.sine_frequencies = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]  # Hz

class PerformanceTester:
    def __init__(self, axis, config: PerfTestConfig = None):
        self.axis = axis
        self.cfg = config or PerfTestConfig()
        self.results = {}

    def run_all_tests(self) -> Dict:
        """Run complete performance test suite."""
        print("=" * 60)
        print("Single Joint Performance Test Suite")
        print("=" * 60)

        results = {}

        # Step response
        print("\n[1/6] Step Response Test")
        results["step_response"] = self._test_step_response()

        # Sinusoidal tracking
        print("\n[2/6] Sinusoidal Tracking Test")
        results["sine_tracking"] = self._test_sine_tracking()

        # Bode plot
        print("\n[3/6] Bode Plot Estimation")
        results["bode"] = self._estimate_bode()

        # Max velocity
        print("\n[4/6] Max Velocity Test")
        results["max_velocity"] = self._test_max_velocity()

        # Max torque
        print("\n[5/6] Max Torque Test")
        results["max_torque"] = self._test_max_torque()

        # Thermal
        print("\n[6/6] Thermal Test")
        results["thermal"] = self._test_thermal()

        self.results = results
        self._print_summary(results)
        return results

    def _test_step_response(self) -> Dict:
        """Measure step response characteristics."""
        amp = self.cfg.step_amplitude

        # Step up
        t_data, pos_data, vel_data, torque_data = self._record_step(0.0, amp, duration=3.0)

        # Calculate metrics
        metrics = self._calculate_step_metrics(t_data, pos_data, amp)

        print(f"  Rise time (10-90%): {metrics['rise_time']:.3f}s")
        print(f"  Settling time (2%): {metrics['settling_time']:.3f}s")
        print(f"  Overshoot: {metrics['overshoot']:.2f}%")
        print(f"  Steady-state error: {metrics['ss_error']:.4f}rad")

        return {
            "metrics": metrics,
            "time": t_data,
            "position": pos_data,
            "velocity": vel_data,
            "torque": torque_data,
            "pass": metrics["overshoot"] < 20 and metrics["settling_time"] < 1.0,
        }

    def _record_step(self, start_pos: float, end_pos: float, duration: float = 3.0) -> tuple:
        """Record response to a step command."""
        dt = 0.01
        n_samples = int(duration / dt)

        t_data = []
        pos_data = []
        vel_data = []
        torque_data = []

        # Go to start
        self.axis.set_position(start_pos)
        time.sleep(1.0)

        # Step
        self.axis.set_position(end_pos)

        start_time = time.time()
        for i in range(n_samples):
            t = time.time() - start_time
            state = self.axis.read_state()

            t_data.append(t)
            pos_data.append(state["pos_estimate"])
            vel_data.append(state["vel_estimate"])
            torque_data.append(state["torque_estimate"])

            time.sleep(dt)

        return t_data, pos_data, vel_data, torque_data

    def _calculate_step_metrics(self, t_data, pos_data, amplitude):
        """Calculate step response metrics."""
        pos = np.array(pos_data)
        t = np.array(t_data)

        start_val = pos[0]
        target = start_val + amplitude

        # Rise time: 10% to 90%
        ten_pct = start_val + 0.1 * amplitude
        ninety_pct = start_val + 0.9 * amplitude

        t_10 = t[np.where(pos >= ten_pct)[0][0]] if np.any(pos >= ten_pct) else 0
        t_90 = t[np.where(pos >= ninety_pct)[0][0]] if np.any(pos >= ninety_pct) else 0
        rise_time = t_90 - t_10

        # Overshoot
        max_val = np.max(pos)
        overshoot = ((max_val - target) / amplitude) * 100 if amplitude > 0 else 0

        # Settling time (within 2% of final)
        threshold = 0.02 * abs(amplitude)
        settled = np.where(np.abs(pos - target) < threshold)[0]
        settling_time = t[settled[-1]] if len(settled) > 0 and settled[-1] > len(pos)//2 else t[-1]

        # Steady-state error (last 10% average)
        ss_start = int(len(pos) * 0.9)
        ss_error = np.mean(pos[ss_start:]) - target

        return {
            "rise_time": float(rise_time),
            "settling_time": float(settling_time),
            "overshoot": float(overshoot),
            "ss_error": float(ss_error),
        }

    def _test_sine_tracking(self) -> Dict:
        """Test sinusoidal tracking at multiple frequencies."""
        amp = self.cfg.sine_amplitude
        frequencies = self.cfg.sine_frequencies

        tracking_errors = []
        phase_lags = []

        for freq in frequencies:
            period = 1.0 / freq
            duration = period * 3  # 3 cycles
            dt = 0.01
            n_samples = int(duration / dt)

            t_data = []
            cmd_data = []
            pos_data = []

            start_time = time.time()
            for i in range(n_samples):
                t = time.time() - start_time
                cmd = amp * np.sin(2 * np.pi * freq * t)

                self.axis.set_position(cmd)
                state = self.axis.read_state()

                t_data.append(t)
                cmd_data.append(cmd)
                pos_data.append(state["pos_estimate"])

                time.sleep(dt)

            # Calculate tracking error (RMS)
            cmd = np.array(cmd_data)
            pos = np.array(pos_data)
            error = cmd - pos
            rms_error = np.sqrt(np.mean(error**2))

            # Phase lag (cross-correlation)
            corr = np.correlate(cmd, pos, mode='full')
            lag = np.argmax(corr) - (len(cmd) - 1)
            phase_lag = (lag * dt) / period * 360  # degrees

            tracking_errors.append(rms_error)
            phase_lags.append(phase_lag)

            print(f"  {freq:.1f}Hz: RMS error={rms_error:.4f}rad, Phase lag={phase_lag:.1f}deg")

        return {
            "frequencies": frequencies,
            "rms_errors": tracking_errors,
            "phase_lags": phase_lags,
            "pass": all(e < 0.05 for e in tracking_errors[:3]),  # Low freq should track well
        }

    def _estimate_bode(self) -> Dict:
        """Estimate Bode plot from sine tracking data."""
        if "sine_tracking" not in self.results:
            return {}

        sine = self.results["sine_tracking"]
        freqs = np.array(sine["frequencies"])
        errors = np.array(sine["rms_errors"])

        # Magnitude: 1 / (1 + error/amplitude) simplified
        amp = self.cfg.sine_amplitude
        mag = 20 * np.log10(amp / (errors + 0.001))

        # Phase from tracking data
        phase = np.array(sine["phase_lags"])

        return {
            "frequencies": freqs.tolist(),
            "magnitude_db": mag.tolist(),
            "phase_deg": phase.tolist(),
        }

    def _test_max_velocity(self) -> Dict:
        """Test maximum achievable velocity."""
        test_velocities = [1.0, 2.0, 3.0, 4.0, 5.0]
        achieved = []

        for vel in test_velocities:
            self.axis.set_velocity(vel)
            time.sleep(1.0)
            state = self.axis.read_state()
            actual_vel = state["vel_estimate"]
            achieved.append(actual_vel)

            print(f"  Command: {vel:.1f}rad/s -> Actual: {actual_vel:.2f}rad/s")

            if actual_vel < vel * 0.9:
                print(f"  Saturation detected at ~{actual_vel:.2f}rad/s")
                break

        self.axis.set_velocity(0.0)

        max_achieved = max(achieved)
        return {
            "test_velocities": test_velocities,
            "achieved_velocities": achieved,
            "max_velocity": float(max_achieved),
            "pass": max_achieved >= 2.0,  # At least 2 rad/s
        }

    def _test_max_torque(self) -> Dict:
        """Test maximum torque output."""
        test_torques = [2.0, 4.0, 6.0, 8.0, 10.0]
        achieved = []

        for torque in test_torques:
            self.axis.set_torque(torque)
            time.sleep(0.5)
            state = self.axis.read_state()
            actual_torque = state["torque_estimate"]
            achieved.append(actual_torque)

            print(f"  Command: {torque:.1f}Nm -> Actual: {actual_torque:.2f}Nm")

            if actual_torque < torque * 0.8:
                print(f"  Saturation detected at ~{actual_torque:.2f}Nm")
                break

        self.axis.set_torque(0.0)

        max_achieved = max(achieved)
        return {
            "test_torques": test_torques,
            "achieved_torques": achieved,
            "max_torque": float(max_achieved),
            "pass": max_achieved >= 5.0,  # At least 5 Nm
        }

    def _test_thermal(self) -> Dict:
        """Monitor temperature during continuous operation."""
        print(f"  Running continuous motion for {self.cfg.thermal_duration}s")

        temps = []
        times = []

        start = time.time()
        while time.time() - start < self.cfg.thermal_duration:
            # Sinusoidal motion to generate heat
            t = time.time() - start
            pos = 0.5 * np.sin(2 * np.pi * 0.5 * t)
            self.axis.set_position(pos)

            if int(t) % int(self.cfg.thermal_sample_interval) == 0:
                state = self.axis.read_state()
                temps.append(state["motor_temp"])
                times.append(t)
                print(f"  T={t:.0f}s: Motor temp = {state['motor_temp']:.1f}C")

                if state["motor_temp"] > 80:
                    print("  WARNING: Temperature approaching limit!")
                if state["motor_temp"] > 90:
                    print("  CRITICAL: Overheating! Stopping test.")
                    break

            time.sleep(0.1)

        self.axis.set_position(0.0)

        max_temp = max(temps) if temps else 0
        return {
            "duration": time.time() - start,
            "temperatures": temps,
            "times": times,
            "max_temperature": float(max_temp),
            "pass": max_temp < 85,
        }

    def _print_summary(self, results: Dict):
        print("\n" + "=" * 60)
        print("Performance Test Summary")
        print("=" * 60)

        if "step_response" in results:
            m = results["step_response"]["metrics"]
            print(f"  Step Response:")
            print(f"    Rise time:   {m['rise_time']:.3f}s")
            print(f"    Settling:    {m['settling_time']:.3f}s")
            print(f"    Overshoot:   {m['overshoot']:.1f}%")

        if "max_velocity" in results:
            print(f"  Max Velocity: {results['max_velocity']['max_velocity']:.2f}rad/s")

        if "max_torque" in results:
            print(f"  Max Torque:   {results['max_torque']['max_torque']:.2f}Nm")

        if "thermal" in results:
            print(f"  Max Temp:     {results['thermal']['max_temperature']:.1f}C")

        all_pass = all(r.get("pass", True) for r in results.values())
        print("\n" + ("ALL TESTS PASSED" if all_pass else "SOME TESTS FAILED"))
        print("=" * 60)

    def plot_step_response(self, save_path: str = None):
        if "step_response" not in self.results:
            return

        step = self.results["step_response"]
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        t = np.array(step["time"])

        axes[0].plot(t, step["position"], 'b-', linewidth=1.5)
        axes[0].axhline(y=self.cfg.step_amplitude, color='r', linestyle='--', alpha=0.5, label='Target')
        axes[0].set_ylabel('Position (rad)')
        axes[0].set_title('Step Response')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t, step["velocity"], 'g-', linewidth=1.5)
        axes[1].set_ylabel('Velocity (rad/s)')
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(t, step["torque"], 'r-', linewidth=1.5)
        axes[2].set_ylabel('Torque (Nm)')
        axes[2].set_xlabel('Time (s)')
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()

    def plot_bode(self, save_path: str = None):
        if "bode" not in self.results:
            return

        bode = self.results["bode"]
        freqs = np.array(bode["frequencies"])
        mag = np.array(bode["magnitude_db"])
        phase = np.array(bode["phase_deg"])

        fig, axes = plt.subplots(2, 1, figsize=(10, 8))

        axes[0].semilogx(freqs, mag, 'b-o', linewidth=1.5, markersize=6)
        axes[0].axhline(y=-3, color='r', linestyle='--', alpha=0.5, label='-3dB line')
        axes[0].set_ylabel('Magnitude (dB)')
        axes[0].set_title('Estimated Bode Plot')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3, which='both')

        axes[1].semilogx(freqs, phase, 'g-o', linewidth=1.5, markersize=6)
        axes[1].axhline(y=-90, color='r', linestyle='--', alpha=0.5)
        axes[1].set_ylabel('Phase (deg)')
        axes[1].set_xlabel('Frequency (Hz)')
        axes[1].grid(True, alpha=0.3, which='both')

        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close()

    def save_results(self, filepath: str):
        with open(filepath, "w") as f:
            json.dump(self.results, f, indent=2)
        print(f"Results saved to {filepath}")

# Standalone
if __name__ == "__main__":
    from odrive_interface import ODriveAxis, ODriveConfig

    axis = ODriveAxis(ODriveConfig())
    axis.connect()

    tester = PerformanceTester(axis)
    results = tester.run_all_tests()
    tester.plot_step_response("data/step_response.png")
    tester.plot_bode("data/bode_plot.png")
    tester.save_results("data/performance_test.json")

    axis.disconnect()
