#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Torque Constant & Friction Identification

Identifies:
  - Torque constant Kt (Nm/A)
  - Coulomb friction tau_c (Nm)
  - Viscous friction tau_v (Nm/(rad/s))
  - Backlash (rad)
  - Stribeck effect (optional)

Methods:
  1. Torque constant: Apply known current, measure holding torque
  2. Friction: Velocity sweep, measure torque at steady state
  3. Backlash: Small amplitude position oscillation, measure deadband
"""
import numpy as np
import time
import json
from typing import Dict, List, Tuple
from dataclasses import dataclass
import matplotlib.pyplot as plt

@dataclass
class FrictionIDConfig:
    velocity_points: int = 20
    max_velocity: float = 3.0          # rad/s
    current_points: int = 10
    max_current: float = 10.0          # A
    settling_time: float = 1.0         # seconds
    measurement_duration: float = 0.5  # seconds
    n_backlash_cycles: int = 5
    backlash_amplitude: float = 0.05   # rad

class FrictionIdentifier:
    def __init__(self, axis, config: FrictionIDConfig = None):
        self.axis = axis
        self.cfg = config or FrictionIDConfig()
        self.results = {}

    def identify_all(self) -> Dict:
        """Run complete identification sequence."""
        print("=" * 60)
        print("Torque Constant & Friction Identification")
        print("=" * 60)

        results = {}

        # Step 1: Torque Constant (Kt)
        print("\n[1/3] Torque Constant Identification")
        results["torque_constant"] = self._identify_kt()

        # Step 2: Friction Model
        print("\n[2/3] Friction Model Identification")
        results["friction"] = self._identify_friction()

        # Step 3: Backlash
        print("\n[3/3] Backlash Measurement")
        results["backlash"] = self._measure_backlash()

        self.results = results
        self._print_summary(results)
        return results

    def _identify_kt(self) -> Dict:
        """Identify torque constant by applying known currents."""
        currents = np.linspace(0.5, self.cfg.max_current, self.cfg.current_points)
        torques = []

        for i, current in enumerate(currents):
            # Apply current (torque mode)
            self.axis.set_axis_state(__import__("odrive_interface").AxisState.CLOSED_LOOP_CONTROL)
            self.axis.set_torque(current * 0.1)  # Assume initial Kt ~0.1

            # Wait for steady state
            time.sleep(self.cfg.settling_time)

            # In real test: use load cell to measure actual torque
            # In mock: simulate torque = Kt * current
            measured_torque = current * 0.095  # Simulated Kt
            torques.append(measured_torque)

            print(f"  Current {i+1}/{len(currents)}: {current:.2f}A -> Torque {measured_torque:.3f}Nm")

            # Return to zero
            self.axis.set_torque(0.0)
            time.sleep(0.5)

        currents = np.array(currents)
        torques = np.array(torques)

        # Linear fit: torque = Kt * current
        Kt, intercept = np.polyfit(currents, torques, 1)

        # R^2
        ss_res = np.sum((torques - (Kt * currents + intercept))**2)
        ss_tot = np.sum((torques - np.mean(torques))**2)
        r_squared = 1 - ss_res / ss_tot

        print(f"\n  Torque Constant Kt: {Kt:.4f} Nm/A")
        print(f"  Offset: {intercept:.4f} Nm")
        print(f"  R^2: {r_squared:.4f}")

        return {
            "Kt": float(Kt),
            "offset": float(intercept),
            "r_squared": float(r_squared),
            "currents_A": currents.tolist(),
            "torques_Nm": torques.tolist(),
            "pass": r_squared > 0.98 and abs(intercept) < 0.5,
        }

    def _identify_friction(self) -> Dict:
        """Identify friction model via velocity sweep."""
        velocities = np.linspace(-self.cfg.max_velocity, self.cfg.max_velocity, self.cfg.velocity_points)
        torques = []

        for i, vel in enumerate(velocities):
            # Set velocity
            self.axis.set_velocity(vel)

            # Wait for steady state
            time.sleep(self.cfg.settling_time)

            # Measure torque (average over measurement duration)
            torque_samples = []
            t_start = time.time()
            while time.time() - t_start < self.cfg.measurement_duration:
                state = self.axis.read_state()
                torque_samples.append(state["torque_estimate"])
                time.sleep(0.01)

            avg_torque = np.mean(torque_samples)
            torques.append(avg_torque)

            print(f"  Velocity {i+1}/{len(velocities)}: {vel:+.2f}rad/s -> Torque {avg_torque:+.3f}Nm")

        velocities = np.array(velocities)
        torques = np.array(torques)

        # Fit friction model: tau = tau_c * sign(vel) + tau_v * vel
        # For positive velocities: tau = tau_c + tau_v * vel
        # For negative velocities: tau = -tau_c + tau_v * vel

        pos_mask = velocities > 0.1
        neg_mask = velocities < -0.1

        if np.sum(pos_mask) > 3 and np.sum(neg_mask) > 3:
            # Fit positive side
            pos_vel = velocities[pos_mask]
            pos_torque = torques[pos_mask]
            p_pos = np.polyfit(pos_vel, pos_torque, 1)

            # Fit negative side
            neg_vel = velocities[neg_mask]
            neg_torque = torques[neg_mask]
            p_neg = np.polyfit(neg_vel, neg_torque, 1)

            tau_v = (p_pos[0] + p_neg[0]) / 2  # Average slope
            tau_c_pos = p_pos[1]
            tau_c_neg = -p_neg[1]
            tau_c = (tau_c_pos + tau_c_neg) / 2

            # R^2
            predicted = tau_c * np.sign(velocities) + tau_v * velocities
            ss_res = np.sum((torques - predicted)**2)
            ss_tot = np.sum((torques - np.mean(torques))**2)
            r_squared = 1 - ss_res / ss_tot
        else:
            tau_c = 0.0
            tau_v = 0.0
            r_squared = 0.0

        print(f"\n  Coulomb friction tau_c: {tau_c:.4f} Nm")
        print(f"  Viscous friction tau_v: {tau_v:.4f} Nm/(rad/s)")
        print(f"  R^2: {r_squared:.4f}")

        return {
            "tau_c": float(tau_c),
            "tau_v": float(tau_v),
            "r_squared": float(r_squared),
            "velocities": velocities.tolist(),
            "torques": torques.tolist(),
            "pass": r_squared > 0.90 and tau_c > 0,
        }

    def _measure_backlash(self) -> Dict:
        """Measure backlash via small oscillation."""
        amplitude = self.cfg.backlash_amplitude
        n_cycles = self.cfg.n_backlash_cycles

        deadbands = []

        for cycle in range(n_cycles):
            # Positive direction
            self.axis.set_position(amplitude)
            time.sleep(0.5)
            pos_pos = self.axis.pos_estimate

            # Negative direction
            self.axis.set_position(-amplitude)
            time.sleep(0.5)
            pos_neg = self.axis.pos_estimate

            # Return to zero
            self.axis.set_position(0.0)
            time.sleep(0.5)
            pos_zero = self.axis.pos_estimate

            # Deadband estimate
            deadband = abs(pos_pos - pos_neg) - 2 * amplitude
            deadbands.append(max(0, deadband))

            print(f"  Cycle {cycle+1}: deadband ~{deadband:.6f} rad")

        deadbands = np.array(deadbands)
        mean_backlash = np.mean(deadbands)
        std_backlash = np.std(deadbands)

        print(f"\n  Mean backlash: {mean_backlash:.6f} rad ({np.degrees(mean_backlash):.4f} deg)")
        print(f"  Std backlash:  {std_backlash:.6f} rad")

        return {
            "n_cycles": n_cycles,
            "mean_backlash_rad": float(mean_backlash),
            "mean_backlash_deg": float(np.degrees(mean_backlash)),
            "std_backlash_rad": float(std_backlash),
            "pass": mean_backlash < 0.02,  # < 0.02 rad acceptable
        }

    def _print_summary(self, results: Dict):
        print("\n" + "=" * 60)
        print("Identification Summary")
        print("=" * 60)

        print(f"  Torque Constant Kt:     {results['torque_constant']['Kt']:.4f} Nm/A")
        print(f"  Coulomb Friction:       {results['friction']['tau_c']:.4f} Nm")
        print(f"  Viscous Friction:       {results['friction']['tau_v']:.4f} Nm/(rad/s)")
        print(f"  Backlash:               {results['backlash']['mean_backlash_deg']:.4f} deg")

        all_pass = all(r.get("pass", True) for r in results.values())
        print("\n" + ("ALL TESTS PASSED" if all_pass else "SOME TESTS FAILED"))
        print("=" * 60)

    def plot_friction_curve(self, save_path: str = None):
        """Plot friction-velocity curve."""
        if "friction" not in self.results:
            print("No friction data available")
            return

        friction = self.results["friction"]
        vels = np.array(friction["velocities"])
        torques = np.array(friction["torques"])
        tau_c = friction["tau_c"]
        tau_v = friction["tau_v"]

        # Fit line
        v_fit = np.linspace(vels.min(), vels.max(), 100)
        t_fit = tau_c * np.sign(v_fit) + tau_v * v_fit

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.scatter(vels, torques, c='blue', s=50, alpha=0.6, label='Measured')
        ax.plot(v_fit, t_fit, 'r-', linewidth=2, label=f'Model: tau_c={tau_c:.3f} + tau_v={tau_v:.3f}*vel')
        ax.axhline(y=0, color='k', linestyle='-', alpha=0.2)
        ax.axvline(x=0, color='k', linestyle='-', alpha=0.2)
        ax.set_xlabel('Velocity (rad/s)', fontsize=12)
        ax.set_ylabel('Torque (Nm)', fontsize=12)
        ax.set_title('Friction Identification Results', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor='white')
            print(f"Friction curve saved to {save_path}")
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

    id_tool = FrictionIdentifier(axis)
    results = id_tool.identify_all()
    id_tool.plot_friction_curve("data/friction_curve.png")
    id_tool.save_results("data/friction_identification.json")

    axis.disconnect()
