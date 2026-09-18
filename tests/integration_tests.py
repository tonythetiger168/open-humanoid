#!/usr/bin/env python3
"""OpenHumanoid v1.0 - System Integration Test Suite"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from simulation.pybullet_env import OpenHumanoidSim, SimConfig
from control.main_controller import RobotController, ControlMode
import json

class IntegrationTests:
    def __init__(self):
        self.results = {}

    def test_01_stand_balance(self, duration=3.0):
        """Test 1: Static standing balance for 3 seconds."""
        print("\n[Test 01] Stand Balance Test")
        cfg = SimConfig(urdf_path="../design/open_humanoid_v1.urdf", use_gui=False,
                        sim_freq=240, control_freq=50, initial_base_pos=(0,0,1.05))
        with OpenHumanoidSim(cfg) as sim:
            ctrl = RobotController(sim, ControlMode.STAND)
            ctrl.reset()
            for _ in range(int(duration / cfg.time_step)):
                sim.step(lambda s: ctrl.update())
            bz = sim.base_pose["pos"][2]
            passed = bz > 0.8
            self.results["stand_balance"] = {"passed": passed, "base_z": float(bz)}
            print(f"  Result: {'PASS' if passed else 'FAIL'} (BaseZ={bz:.3f}m)")
            return passed

    def test_02_walk_forward(self, duration=5.0):
        """Test 2: Walk forward for 5 seconds without falling."""
        print("\n[Test 02] Walk Forward Test")
        cfg = SimConfig(urdf_path="../design/open_humanoid_v1.urdf", use_gui=False,
                        sim_freq=240, control_freq=50, initial_base_pos=(0,0,1.05))
        with OpenHumanoidSim(cfg) as sim:
            ctrl = RobotController(sim, ControlMode.STAND)
            ctrl.reset()
            # Stand first
            for _ in range(int(1.0 / cfg.time_step)):
                sim.step(lambda s: ctrl.update())
            # Switch to walk
            ctrl.set_mode(ControlMode.WALK)
            start_x = sim.base_pose["pos"][0]
            for _ in range(int(duration / cfg.time_step)):
                sim.step(lambda s: ctrl.update())
            end_x = sim.base_pose["pos"][0]
            bz = sim.base_pose["pos"][2]
            distance = end_x - start_x
            passed = bz > 0.5 and not ctrl.fallen
            self.results["walk_forward"] = {"passed": passed, "distance_m": float(distance), "base_z": float(bz)}
            print(f"  Result: {'PASS' if passed else 'FAIL'} (Dist={distance:.2f}m, Z={bz:.3f}m)")
            return passed

    def test_03_fall_recovery(self):
        """Test 3: Fall detection and recovery."""
        print("\n[Test 03] Fall Recovery Test")
        cfg = SimConfig(urdf_path="../design/open_humanoid_v1.urdf", use_gui=False,
                        sim_freq=240, control_freq=50, initial_base_pos=(0,0,1.05))
        with OpenHumanoidSim(cfg) as sim:
            ctrl = RobotController(sim, ControlMode.STAND)
            ctrl.reset()
            # Run briefly
            for _ in range(int(0.5 / cfg.time_step)):
                sim.step(lambda s: ctrl.update())
            # Simulate fall by pushing base
            p = __import__("pybullet")
            p.applyExternalForce(sim.robot_id, -1, [50, 0, -20], [0,0,0], p.LINK_FRAME)
            # Continue
            for _ in range(int(1.0 / cfg.time_step)):
                sim.step(lambda s: ctrl.update())
            passed = ctrl.mode == ControlMode.STAND  # Should recover to stand
            self.results["fall_recovery"] = {"passed": passed, "final_mode": ctrl.mode}
            print(f"  Result: {'PASS' if passed else 'FAIL'} (Final mode: {ctrl.mode})")
            return passed

    def test_04_terrain_flat(self, duration=3.0):
        """Test 4: Balance on flat ground."""
        print("\n[Test 04] Flat Terrain Test")
        cfg = SimConfig(urdf_path="../design/open_humanoid_v1.urdf", use_gui=False,
                        terrain_type="flat", sim_freq=240, control_freq=50)
        with OpenHumanoidSim(cfg) as sim:
            ctrl = RobotController(sim, ControlMode.STAND)
            ctrl.reset()
            for _ in range(int(duration / cfg.time_step)):
                sim.step(lambda s: ctrl.update())
            bz = sim.base_pose["pos"][2]
            passed = bz > 0.8
            self.results["terrain_flat"] = {"passed": passed, "base_z": float(bz)}
            print(f"  Result: {'PASS' if passed else 'FAIL'} (BaseZ={bz:.3f}m)")
            return passed

    def run_all(self):
        print("=" * 50)
        print("OpenHumanoid v1.0 - Integration Test Suite")
        print("=" * 50)
        tests = [self.test_01_stand_balance, self.test_02_walk_forward,
                 self.test_03_fall_recovery, self.test_04_terrain_flat]
        passed = sum(t() for t in tests)
        total = len(tests)
        print("\n" + "=" * 50)
        print(f"Results: {passed}/{total} tests passed")
        print("=" * 50)
        with open("test_results.json", "w") as f:
            json.dump(self.results, f, indent=2)
        return passed == total

if __name__ == "__main__":
    suite = IntegrationTests()
    suite.run_all()
