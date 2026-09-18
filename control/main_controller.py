#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Main Control Loop (STAND/WALK/TELEOP/FALLEN)"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from simulation.pybullet_env import OpenHumanoidSim, SimConfig
from control.balance_controller import BalanceController, BalanceConfig, SimpleBalancePD
from control.gait_generator import WalkingPatternGenerator, WalkConfig, WholeBodyController, LegIK, GaitPhase
from perception.state_estimator import StateEstimator
import numpy as np

class ControlMode:
    STAND = "stand"
    WALK = "walk"
    TELEOP = "teleop"
    FALLEN = "fallen"

class RobotController:
    def __init__(self, sim: OpenHumanoidSim, mode=ControlMode.STAND):
        self.sim = sim
        self.mode = mode
        self.dt = 1.0 / sim.cfg.control_freq
        self.balance = BalanceController(BalanceConfig(com_height=0.80))
        self.gait = WalkingPatternGenerator(WalkConfig())
        self.wbc = WholeBodyController(LegIK(thigh=0.24, shin=0.20))
        self.state_est = StateEstimator(dt=self.dt)
        self.simple_pd = SimpleBalancePD(kp=100.0, kd=8.0)
        self.step_count = 0
        self.fallen = False
        self.fall_threshold = 0.4
        self.command_vel = np.array([0.0, 0.0, 0.0])
        self.target_yaw = 0.0
        self.log = []

    def reset(self):
        self.balance.reset(self.sim.base_pose["pos"][:2])
        self.state_est.reset(self.sim.base_pose["pos"])
        self.gait.init(np.array([0.0, 0.08, 0.0]), np.array([0.0, -0.08, 0.0]), self.sim.base_pose["pos"])
        self.step_count = 0
        self.fallen = False
        self.mode = ControlMode.STAND

    def set_mode(self, mode):
        print(f"[Ctrl] Mode: {self.mode} -> {mode}")
        self.mode = mode
        if mode == ControlMode.STAND:
            self.balance.reset(self.sim.base_pose["pos"][:2])
        elif mode == ControlMode.WALK:
            self.gait.init(np.array([0.0, 0.08, 0.0]), np.array([0.0, -0.08, 0.0]), self.sim.base_pose["pos"])

    def set_cmd_vel(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.command_vel = np.array([vx, vy, yaw_rate])

    def _check_fall(self):
        bz = self.sim.base_pose["pos"][2]
        r, p = abs(self.sim.base_pose["orn"][0]), abs(self.sim.base_pose["orn"][1])
        if bz < self.fall_threshold or r > 1.0 or p > 1.0:
            if not self.fallen:
                print(f"[Ctrl] FALL! Z={bz:.3f} R={r:.2f} P={p:.2f}")
                self.fallen = True
                self.mode = ControlMode.FALLEN
            return True
        return False

    def _recover(self):
        self.sim._set_default_pose()
        self.reset()

    def _stand(self):
        state = self.state_est.update(self.sim.imu_data, self.sim.joint_states,
                                      self.sim.foot_force, self.sim.base_pose["pos"], self.sim.base_pose["orn"])
        roll, pitch, yaw = state["base_rpy"]
        rv, pv, _ = self.sim.base_pose["omega"]
        tau_r, tau_p = self.simple_pd.compute(-roll, -pitch, rv, pv)
        targets = {
            "l_hip_yaw_joint": 0.0, "l_hip_roll_joint": 0.0, "l_hip_pitch_joint": -0.15,
            "l_knee_pitch_joint": 0.30, "l_ankle_pitch_joint": -0.15 + tau_p * 0.001,
            "l_ankle_roll_joint": 0.0 + tau_r * 0.001,
            "r_hip_yaw_joint": 0.0, "r_hip_roll_joint": 0.0, "r_hip_pitch_joint": -0.15,
            "r_knee_pitch_joint": 0.30, "r_ankle_pitch_joint": -0.15 + tau_p * 0.001,
            "r_ankle_roll_joint": 0.0 + tau_r * 0.001,
            "spine_pitch_joint": 0.0, "spine_yaw_joint": 0.0,
            "l_shoulder_pitch_joint": 0.0, "l_shoulder_roll_joint": 0.0, "l_shoulder_yaw_joint": 0.0,
            "l_elbow_pitch_joint": -0.5, "l_wrist_roll_joint": 0.0, "l_wrist_pitch_joint": 0.0, "l_wrist_yaw_joint": 0.0,
            "r_shoulder_pitch_joint": 0.0, "r_shoulder_roll_joint": 0.0, "r_shoulder_yaw_joint": 0.0,
            "r_elbow_pitch_joint": -0.5, "r_wrist_roll_joint": 0.0, "r_wrist_pitch_joint": 0.0, "r_wrist_yaw_joint": 0.0,
            "neck_yaw_joint": 0.0, "neck_pitch_joint": 0.0,
        }
        return targets

    def _walk(self):
        gait_out = self.gait.update(self.dt)
        state = self.state_est.update(self.sim.imu_data, self.sim.joint_states,
                                      self.sim.foot_force, self.sim.base_pose["pos"], self.sim.base_pose["orn"])
        targets = self.wbc.compute_targets(
            com_pos=gait_out["com_pos"],
            left_foot=gait_out["left_foot"],
            right_foot=gait_out["right_foot"],
            base_rpy=state["base_rpy"]
        )
        if gait_out["support_foot"] == "left":
            targets["r_shoulder_pitch_joint"] = 0.3
            targets["l_shoulder_pitch_joint"] = -0.3
        elif gait_out["support_foot"] == "right":
            targets["r_shoulder_pitch_joint"] = -0.3
            targets["l_shoulder_pitch_joint"] = 0.3
        return targets

    def _teleop(self):
        targets = self._stand()
        targets["l_ankle_pitch_joint"] += self.command_vel[0] * 0.05
        targets["r_ankle_pitch_joint"] += self.command_vel[0] * 0.05
        targets["l_ankle_roll_joint"] += self.command_vel[1] * 0.05
        targets["r_ankle_roll_joint"] += self.command_vel[1] * 0.05
        self.target_yaw += self.command_vel[2] * self.dt
        targets["neck_yaw_joint"] = self.target_yaw * 0.5
        return targets

    def update(self):
        if self.mode != ControlMode.FALLEN:
            self._check_fall()
        if self.mode == ControlMode.FALLEN:
            self._recover()
            return {}
        if self.mode == ControlMode.STAND:
            targets = self._stand()
        elif self.mode == ControlMode.WALK:
            targets = self._walk()
        elif self.mode == ControlMode.TELEOP:
            targets = self._teleop()
        else:
            targets = {}
        if targets:
            self.sim.set_joint_positions(targets, max_force=80.0)
        self.log.append({
            "time": self.sim._sim_time,
            "mode": self.mode,
            "base_z": self.sim.base_pose["pos"][2],
            "lf": self.sim.foot_force["left"],
            "rf": self.sim.foot_force["right"],
        })
        self.step_count += 1
        return targets

def main():
    cfg = SimConfig(urdf_path="../design/open_humanoid_v1.urdf", terrain_type="flat",
                    use_gui=False, use_real_time=False, sim_freq=240.0, control_freq=50.0)
    with OpenHumanoidSim(cfg) as sim:
        ctrl = RobotController(sim, mode=ControlMode.STAND)
        ctrl.reset()
        print("[Main] Running 5s STAND test...")
        for i in range(int(5.0 / cfg.time_step)):
            def cb(s): ctrl.update()
            sim.step(controller_callback=cb)
        print(f"[Main] STAND complete. BaseZ final: {sim.base_pose['pos'][2]:.3f}m")
        # Switch to walk
        ctrl.set_mode(ControlMode.WALK)
        print("[Main] Running 5s WALK test...")
        for i in range(int(5.0 / cfg.time_step)):
            def cb(s): ctrl.update()
            sim.step(controller_callback=cb)
        print(f"[Main] WALK complete. BaseZ final: {sim.base_pose['pos'][2]:.3f}m")
        # Save log
        import json
        with open("../tests/sim_log.json", "w") as f:
            json.dump(ctrl.log, f)
        print("[Main] Log saved to tests/sim_log.json")

if __name__ == "__main__":
    main()
