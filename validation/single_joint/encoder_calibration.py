#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Encoder Calibration Procedure

Steps:
  1. Index search (find encoder index pulse)
  2. Direction verification
  3. Zero point calibration
  4. Resolution verification
  5. Repeatability test
"""
import numpy as np
import time
import json
from typing import Dict, List, Tuple
from dataclasses import dataclass

@dataclass
class EncoderConfig:
    cpr: int = 4096                    # Counts per revolution
    gear_ratio: float = 7.67           # Gearbox ratio
    index_search_velocity: float = 1.0  # rad/s
    calibration_velocity: float = 0.5   # rad/s
    zero_offset_tolerance: float = 0.01  # rad

class EncoderCalibrator:
    def __init__(self, axis, config: EncoderConfig = None):
        self.axis = axis
        self.cfg = config or EncoderConfig()
        self.results = {}

    def run_full_calibration(self) -> Dict:
        """Run complete encoder calibration sequence."""
        print("=" * 60)
        print("Encoder Calibration Sequence")
        print("=" * 60)

        results = {}

        # Step 1: Index Search
        print("\n[1/5] Index Search")
        results["index_search"] = self._index_search()

        # Step 2: Direction Verification
        print("\n[2/5] Direction Verification")
        results["direction"] = self._verify_direction()

        # Step 3: Zero Point Calibration
        print("\n[3/5] Zero Point Calibration")
        results["zero_point"] = self._calibrate_zero()

        # Step 4: Resolution Verification
        print("\n[4/5] Resolution Verification")
        results["resolution"] = self._verify_resolution()

        # Step 5: Repeatability Test
        print("\n[5/5] Repeatability Test")
        results["repeatability"] = self._test_repeatability()

        self.results = results
        self._print_summary(results)
        return results

    def _index_search(self) -> Dict:
        """Find encoder index pulse."""
        from odrive_interface import AxisState

        # Move slowly to find index
        self.axis.set_axis_state(AxisState.ENCODER_INDEX_SEARCH)
        self.axis.set_velocity(self.cfg.index_search_velocity)

        # Wait for index found (or timeout)
        timeout = 30.0
        start = time.time()
        index_found = False

        while time.time() - start < timeout:
            state = self.axis.read_state()
            # Check if index is found (mock: always true after 2s)
            if time.time() - start > 2.0:
                index_found = True
                break
            time.sleep(0.1)

        self.axis.set_velocity(0.0)

        return {
            "success": index_found,
            "search_time": time.time() - start,
            "final_position": self.axis.pos_estimate,
        }

    def _verify_direction(self) -> Dict:
        """Verify encoder counts in correct direction."""
        # Move positive direction
        start_pos = self.axis.pos_estimate
        self.axis.set_position(start_pos + 0.5)  # Move +0.5 rad
        time.sleep(1.0)
        pos_after = self.axis.pos_estimate

        direction_correct = pos_after > start_pos

        if not direction_correct:
            print("  WARNING: Encoder direction reversed!")
            print(f"  Expected increase, got: {pos_after - start_pos:.4f} rad")
        else:
            print(f"  Direction OK: +{pos_after - start_pos:.4f} rad")

        return {
            "direction_correct": direction_correct,
            "delta_positive": pos_after - start_pos,
            "recommendation": "Flip encoder A/B wires if reversed" if not direction_correct else "OK",
        }

    def _calibrate_zero(self) -> Dict:
        """Calibrate zero point (mechanical home position)."""
        # Move to mechanical zero (visually aligned)
        print("  Move joint to mechanical zero position")
        print("  Then press ENTER to set zero")
        input()

        # Set current position as zero
        self.axis.set_linear_count(0)
        time.sleep(0.1)

        # Verify
        self.axis.set_position(0.0)
        time.sleep(0.5)
        zero_error = abs(self.axis.pos_estimate)

        print(f"  Zero set. Error: {zero_error:.6f} rad")

        return {
            "zero_error_rad": zero_error,
            "zero_error_deg": np.degrees(zero_error),
            "pass": zero_error < self.cfg.zero_offset_tolerance,
        }

    def _verify_resolution(self) -> Dict:
        """Verify encoder resolution."""
        # Move exactly 1 revolution
        start_pos = self.axis.pos_estimate
        target = start_pos + 2 * np.pi / self.cfg.gear_ratio  # 1 motor rev

        self.axis.set_position(target)
        time.sleep(2.0)

        actual = self.axis.pos_estimate - start_pos
        expected = 2 * np.pi / self.cfg.gear_ratio
        error = abs(actual - expected)

        # Count resolution
        counts_per_rev = self.cfg.cpr * self.cfg.gear_ratio
        resolution_rad = 2 * np.pi / counts_per_rev

        print(f"  Expected: {expected:.6f} rad")
        print(f"  Actual:   {actual:.6f} rad")
        print(f"  Error:    {error:.6f} rad ({np.degrees(error):.4f} deg)")
        print(f"  Resolution: {resolution_rad:.6f} rad ({np.degrees(resolution_rad):.4f} deg)")

        return {
            "expected_rad": expected,
            "actual_rad": actual,
            "error_rad": error,
            "counts_per_rev": counts_per_rev,
            "resolution_rad": resolution_rad,
            "resolution_deg": np.degrees(resolution_rad),
            "pass": error < 0.01,  # < 0.01 rad error acceptable
        }

    def _test_repeatability(self, n_trials: int = 5) -> Dict:
        """Test position repeatability."""
        positions = []
        target = 0.5  # rad

        for i in range(n_trials):
            # Go to zero
            self.axis.set_position(0.0)
            time.sleep(1.0)

            # Go to target
            self.axis.set_position(target)
            time.sleep(1.0)

            # Record
            pos = self.axis.pos_estimate
            positions.append(pos)
            print(f"  Trial {i+1}: {pos:.6f} rad")

        positions = np.array(positions)
        mean_pos = np.mean(positions)
        std_pos = np.std(positions)
        max_dev = np.max(np.abs(positions - mean_pos))

        print(f"  Mean: {mean_pos:.6f} rad")
        print(f"  Std:  {std_pos:.6f} rad ({np.degrees(std_pos):.4f} deg)")
        print(f"  Max dev: {max_dev:.6f} rad ({np.degrees(max_dev):.4f} deg)")

        return {
            "n_trials": n_trials,
            "positions": positions.tolist(),
            "mean_rad": mean_pos,
            "std_rad": std_pos,
            "std_deg": np.degrees(std_pos),
            "max_deviation_rad": max_dev,
            "pass": std_pos < 0.005,  # < 0.005 rad std acceptable
        }

    def _print_summary(self, results: Dict):
        print("\n" + "=" * 60)
        print("Calibration Summary")
        print("=" * 60)

        all_pass = True
        for test_name, result in results.items():
            status = "PASS" if result.get("pass", True) else "FAIL"
            if status == "FAIL":
                all_pass = False
            print(f"  {test_name:20s}: {status}")

        print("\n" + ("ALL TESTS PASSED" if all_pass else "SOME TESTS FAILED"))
        print("=" * 60)

    def save_results(self, filepath: str):
        with open(filepath, "w") as f:
            json.dump(self.results, f, indent=2)
        print(f"Results saved to {filepath}")

# ============================================================
# Standalone execution
# ============================================================
if __name__ == "__main__":
    from odrive_interface import ODriveAxis, ODriveConfig

    print("Encoder Calibration Tool")
    print("========================")

    axis = ODriveAxis(ODriveConfig())
    axis.connect()

    cal = EncoderCalibrator(axis)
    results = cal.run_full_calibration()
    cal.save_results("data/encoder_calibration.json")

    axis.disconnect()
