#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Balance Controller (ZMP Preview + LIPM)"""
import numpy as np
from typing import Tuple, Dict
from dataclasses import dataclass
import math

@dataclass
class ZMPConfig:
    gravity: float = 9.81
    com_height: float = 0.80
    preview_steps: int = 320
    dt: float = 0.02
    q_weight: float = 1.0
    r_weight: float = 1e-6

class ZMPPreviewController:
    def __init__(self, config: ZMPConfig = None):
        self.cfg = config or ZMPConfig()
        self._compute_gains()
        self.x = np.zeros(3)
        self.u = 0.0

    def _compute_gains(self):
        dt = self.cfg.dt
        z = self.cfg.com_height
        A = np.array([[1, dt, dt**2/2], [0, 1, dt], [0, 0, 1]], dtype=np.float64)
        B = np.array([[dt**3/6], [dt**2/2], [dt]], dtype=np.float64)
        C = np.array([[1, 0, -z/self.cfg.gravity]], dtype=np.float64)
        Q = self.cfg.q_weight * (C.T @ C)
        R = np.array([[self.cfg.r_weight]])
        P = self._dare(A, B, Q, R)
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
        Ac = A - B @ K
        self.Gi = []
        for i in range(self.cfg.preview_steps):
            Gi = -np.linalg.inv(R + B.T @ P @ B) @ (B.T @ np.linalg.matrix_power(Ac.T, i) @ C.T)
            self.Gi.append(Gi[0, 0])
        self.Gi = np.array(self.Gi)
        self.Gx = -K
        self.A, self.B, self.C = A, B, C

    def _dare(self, A, B, Q, R, max_iter=1000, tol=1e-10):
        P = Q.copy()
        for _ in range(max_iter):
            Pn = A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
            if np.max(np.abs(Pn - P)) < tol:
                return Pn
            P = Pn
        return P

    def reset(self, pos=0.0, vel=0.0, acc=0.0):
        self.x = np.array([pos, vel, acc])
        self.u = 0.0

    def update(self, zmp_ref: np.ndarray) -> Tuple[float, float, float]:
        if len(zmp_ref) < self.cfg.preview_steps:
            zmp_ref = np.pad(zmp_ref, (0, self.cfg.preview_steps - len(zmp_ref)), mode="edge")
        else:
            zmp_ref = zmp_ref[:self.cfg.preview_steps]
        e = self.C @ self.x - zmp_ref[0]
        preview = np.dot(self.Gi, zmp_ref - zmp_ref[0])
        self.u = (self.Gx @ self.x + preview)[0]
        self.x = self.A @ self.x + self.B.flatten() * self.u
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    def get_zmp(self) -> float:
        return float(self.C @ self.x)

class LIPM:
    def __init__(self, com_height=0.80, gravity=9.81):
        self.z, self.g = com_height, gravity
        self.omega = math.sqrt(self.g / self.z)

    def zmp(self, com_pos, com_acc):
        return com_pos - (self.z / self.g) * com_acc

    def com_acc(self, com_pos, zmp_pos):
        return (self.g / self.z) * (com_pos - zmp_pos)

@dataclass
class BalanceConfig:
    com_height: float = 0.80
    kp_com: float = 100.0
    kd_com: float = 20.0
    kp_orient: float = 50.0
    kd_orient: float = 10.0
    foot_spacing: float = 0.16

class BalanceController:
    def __init__(self, config: BalanceConfig = None):
        self.cfg = config or BalanceConfig()
        self.zmp_x = ZMPPreviewController(ZMPConfig(com_height=self.cfg.com_height))
        self.zmp_y = ZMPPreviewController(ZMPConfig(com_height=self.cfg.com_height))
        self.lipm = LIPM(self.cfg.com_height)
        self.com_target = np.array([0.0, 0.0, self.cfg.com_height])
        self.base_target_rpy = np.array([0.0, 0.0, 0.0])

    def reset(self, initial_com=None):
        if initial_com is None:
            initial_com = np.array([0.0, 0.0])
        self.zmp_x.reset(initial_com[0], 0.0, 0.0)
        self.zmp_y.reset(initial_com[1], 0.0, 0.0)

    def compute_zmp_ref(self, support_foot: str, foot_pos: Dict) -> np.ndarray:
        if support_foot == "both":
            return (foot_pos["left"][:2] + foot_pos["right"][:2]) / 2
        return foot_pos[support_foot][:2].copy()

    def update(self, current_com: np.ndarray, current_com_vel: np.ndarray,
               current_base_rpy: np.ndarray, current_base_omega: np.ndarray,
               support_foot: str, foot_pos: Dict) -> Dict:
        zmp_ref = self.compute_zmp_ref(support_foot, foot_pos)
        preview_x = np.full(self.zmp_x.cfg.preview_steps, zmp_ref[0])
        preview_y = np.full(self.zmp_y.cfg.preview_steps, zmp_ref[1])
        com_x, vel_x, acc_x = self.zmp_x.update(preview_x)
        com_y, vel_y, acc_y = self.zmp_y.update(preview_y)
        rpy_err = self.base_target_rpy - current_base_rpy
        rpy_err[2] = (rpy_err[2] + np.pi) % (2 * np.pi) - np.pi
        torque_rpy = self.cfg.kp_orient * rpy_err - self.cfg.kd_orient * current_base_omega
        return {
            "com_pos_des": np.array([com_x, com_y, self.cfg.com_height]),
            "com_vel_des": np.array([vel_x, vel_y, 0.0]),
            "com_acc_des": np.array([acc_x, acc_y, 0.0]),
            "zmp_des": np.array([self.zmp_x.get_zmp(), self.zmp_y.get_zmp()]),
            "torque_rpy_des": torque_rpy,
            "support_foot": support_foot
        }

class SimpleBalancePD:
    def __init__(self, kp=100.0, kd=8.0):
        self.kp, self.kd = kp, kd
    def compute(self, roll_err, pitch_err, roll_vel, pitch_vel):
        return self.kp * roll_err - self.kd * roll_vel, self.kp * pitch_err - self.kd * pitch_vel
