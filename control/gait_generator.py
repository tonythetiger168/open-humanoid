#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Gait Generator & Whole-Body Controller"""
import numpy as np
from typing import Tuple, List, Dict, Optional
from dataclasses import dataclass
from enum import Enum, auto
import math

class GaitPhase(Enum):
    LEFT_SUPPORT = auto()
    RIGHT_SUPPORT = auto()
    DOUBLE_SUPPORT = auto()

@dataclass
class GaitConfig:
    step_length: float = 0.15
    step_width: float = 0.16
    step_height: float = 0.05
    step_duration: float = 0.8
    double_support_ratio: float = 0.2
    num_steps: int = 10
    walk_direction: float = 0.0

class FootstepPlanner:
    def __init__(self, config: GaitConfig = None):
        self.cfg = config or GaitConfig()
    def plan_straight(self, start_l, start_r) -> List[Dict]:
        steps = []
        l_pos, r_pos = start_l.copy(), start_r.copy()
        d = np.array([math.cos(self.cfg.walk_direction), math.sin(self.cfg.walk_direction)])
        for i in range(self.cfg.num_steps):
            if i % 2 == 0:
                n = l_pos + d * self.cfg.step_length
                n[1] = self.cfg.step_width / 2
                steps.append({"foot": "left", "pos": n.copy(), "time": i * self.cfg.step_duration})
                l_pos = n
            else:
                n = r_pos + d * self.cfg.step_length
                n[1] = -self.cfg.step_width / 2
                steps.append({"foot": "right", "pos": n.copy(), "time": i * self.cfg.step_duration})
                r_pos = n
        return steps

class SwingTrajectory:
    def __init__(self, start, end, height, duration):
        self.start = np.array(start)
        self.end = np.array(end)
        self.height = height
        self.duration = duration
        self.delta = self.end - self.start
    def position(self, t):
        s = np.clip(t / self.duration, 0.0, 1.0)
        p0, p1 = self.start, self.start + self.delta / 3
        p2, p3 = self.end - self.delta / 3, self.end
        pos = (1-s)**3 * p0 + 3*s*(1-s)**2 * p1 + 3*s**2*(1-s) * p2 + s**3 * p3
        lift = 4 * self.height * s * (1 - s)
        pos[2] = self.start[2] + (self.end[2] - self.start[2]) * s + lift
        return pos
    def velocity(self, t):
        s = np.clip(t / self.duration, 0.0, 1.0)
        ds = 1.0 / self.duration
        p0, p1 = self.start, self.start + self.delta / 3
        p2, p3 = self.end - self.delta / 3, self.end
        vel = (3*(1-s)**2*(p1-p0) + 6*(1-s)*s*(p2-p1) + 3*s**2*(p3-p2)) * ds
        lift_vel = 4 * self.height * (1 - 2*s) * ds
        vel[2] = (self.end[2] - self.start[2]) * ds + lift_vel
        return vel

class GaitFSM:
    def __init__(self, step_duration=0.8, ds_ratio=0.2):
        self.step_dur = step_duration
        self.ds_time = step_duration * ds_ratio
        self.ss_time = step_duration * (1 - ds_ratio)
        self.phase = GaitPhase.DOUBLE_SUPPORT
        self.time_in_phase = 0.0
        self.next_foot = "left"
    def update(self, dt):
        self.time_in_phase += dt
        if self.phase == GaitPhase.DOUBLE_SUPPORT:
            if self.time_in_phase >= self.ds_time / 2:
                self.phase = GaitPhase.LEFT_SUPPORT if self.next_foot == "right" else GaitPhase.RIGHT_SUPPORT
                self.time_in_phase = 0.0
        else:
            if self.time_in_phase >= self.ss_time:
                self.phase = GaitPhase.DOUBLE_SUPPORT
                self.time_in_phase = 0.0
                self.next_foot = "right" if self.next_foot == "left" else "left"
        return self.phase, self.time_in_phase, self.next_foot
    def support_foot(self):
        if self.phase == GaitPhase.LEFT_SUPPORT: return "left"
        if self.phase == GaitPhase.RIGHT_SUPPORT: return "right"
        return "both"

@dataclass
class WalkConfig:
    com_height: float = 0.80
    step_duration: float = 0.8
    double_support_ratio: float = 0.2
    foot_lift_height: float = 0.05
    gravity: float = 9.81

class WalkingPatternGenerator:
    def __init__(self, config: WalkConfig = None):
        self.cfg = config or WalkConfig()
        self.omega = math.sqrt(self.cfg.gravity / self.cfg.com_height)
        self.fsm = GaitFSM(self.cfg.step_duration, self.cfg.double_support_ratio)
        self.planner = FootstepPlanner()
        self.footsteps = []
        self.com_pos = np.array([0.0, 0.0, self.cfg.com_height])
        self.com_vel = np.zeros(3)
        self.l_foot = np.array([0.0, 0.08, 0.0])
        self.r_foot = np.array([0.0, -0.08, 0.0])
        self.swing_traj = None
        self.swing_start = 0.0

    def init(self, start_l, start_r, com_pos):
        self.l_foot, self.r_foot = start_l.copy(), start_r.copy()
        self.com_pos = com_pos.copy()
        self.com_vel = np.zeros(3)
        self.footsteps = self.planner.plan_straight(start_l, start_r)
        self.fsm = GaitFSM(self.cfg.step_duration, self.cfg.double_support_ratio)

    def update(self, dt):
        phase, t_phase, next_f = self.fsm.update(dt)
        support = self.fsm.support_foot()
        if support == "left":
            zmp_d = self.l_foot[:2].copy()
        elif support == "right":
            zmp_d = self.r_foot[:2].copy()
        else:
            zmp_d = (self.l_foot[:2] + self.r_foot[:2]) / 2
        com_acc = self.cfg.gravity / self.cfg.com_height * (self.com_pos[:2] - zmp_d)
        self.com_vel[:2] += com_acc * dt
        self.com_pos[:2] += self.com_vel[:2] * dt
        swing_pos = swing_vel = None
        if phase in (GaitPhase.LEFT_SUPPORT, GaitPhase.RIGHT_SUPPORT) and self.footsteps:
            swing_f = "right" if support == "left" else "left"
            if self.swing_traj is None or getattr(self, "_last_p", None) != phase:
                for fs in self.footsteps:
                    if fs["foot"] == swing_f:
                        start = self.r_foot.copy() if swing_f == "right" else self.l_foot.copy()
                        self.swing_traj = SwingTrajectory(start, fs["pos"], self.cfg.foot_lift_height, self.fsm.ss_time)
                        self.swing_start = t_phase
                        break
            if self.swing_traj is not None:
                swing_pos = self.swing_traj.position(t_phase)
                swing_vel = self.swing_traj.velocity(t_phase)
                if swing_f == "left":
                    self.l_foot = swing_pos.copy()
                else:
                    self.r_foot = swing_pos.copy()
        self._last_p = phase
        return {
            "phase": phase, "support_foot": support,
            "com_pos": self.com_pos.copy(), "com_vel": self.com_vel.copy(),
            "left_foot": self.l_foot.copy(), "right_foot": self.r_foot.copy(),
            "swing_foot_pos": swing_pos, "swing_foot_vel": swing_vel, "zmp_des": zmp_d,
        }

class LegIK:
    def __init__(self, thigh=0.24, shin=0.20, foot=0.05, hip_y=0.08):
        self.L1, self.L2, self.L3, self.hip_y = thigh, shin, foot, hip_y
    def solve(self, foot_pos, foot_roll=0.0, foot_pitch=0.0, side="left"):
        x, y, z = foot_pos
        hip_yaw = 0.0
        y_s = 1.0 if side == "left" else -1.0
        y_eff = y - y_s * self.hip_y
        d = math.sqrt(x**2 + z**2)
        cos_k = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        cos_k = np.clip(cos_k, -1.0, 1.0)
        knee_pitch = math.pi - math.acos(cos_k)
        alpha = math.atan2(-z, x)
        cos_b = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        cos_b = np.clip(cos_b, -1.0, 1.0)
        beta = math.acos(cos_b)
        hip_pitch = alpha + beta
        ankle_pitch = foot_pitch - hip_pitch + knee_pitch
        ankle_roll = foot_roll
        hip_roll = -math.atan2(y_eff, math.sqrt(x**2 + z**2)) * 0.5
        return {
            f"{side[0]}_hip_yaw_joint": hip_yaw,
            f"{side[0]}_hip_roll_joint": hip_roll,
            f"{side[0]}_hip_pitch_joint": hip_pitch,
            f"{side[0]}_knee_pitch_joint": knee_pitch,
            f"{side[0]}_ankle_pitch_joint": ankle_pitch,
            f"{side[0]}_ankle_roll_joint": ankle_roll,
        }

class WholeBodyController:
    def __init__(self, leg_ik=None):
        self.leg_ik = leg_ik or LegIK()
        self.arm_default = {
            "l_shoulder_pitch_joint": 0.0, "l_shoulder_roll_joint": 0.0, "l_shoulder_yaw_joint": 0.0,
            "l_elbow_pitch_joint": -0.5, "l_wrist_roll_joint": 0.0, "l_wrist_pitch_joint": 0.0, "l_wrist_yaw_joint": 0.0,
            "r_shoulder_pitch_joint": 0.0, "r_shoulder_roll_joint": 0.0, "r_shoulder_yaw_joint": 0.0,
            "r_elbow_pitch_joint": -0.5, "r_wrist_roll_joint": 0.0, "r_wrist_pitch_joint": 0.0, "r_wrist_yaw_joint": 0.0,
        }
        self.torso_default = {"spine_pitch_joint": 0.0, "spine_yaw_joint": 0.0}
        self.head_default = {"neck_yaw_joint": 0.0, "neck_pitch_joint": 0.0}

    def compute_targets(self, com_pos, left_foot, right_foot, base_rpy=None) -> Dict[str, float]:
        pelvis = com_pos.copy()
        pelvis[2] -= 0.05
        l_rel = left_foot - pelvis
        r_rel = right_foot - pelvis
        left_j = self.leg_ik.solve(l_rel, side="left")
        right_j = self.leg_ik.solve(r_rel, side="right")
        targets = {}
        targets.update(left_j)
        targets.update(right_j)
        targets.update(self.arm_default)
        targets.update(self.torso_default)
        targets.update(self.head_default)
        return targets
