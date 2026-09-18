#!/usr/bin/env python3
"""OpenHumanoid v1.0 - State Estimator (EKF + Leg Odometry + Support Detection)"""
import numpy as np
from typing import Dict, Tuple
from dataclasses import dataclass
import math

@dataclass
class EKFConfig:
    dt: float = 0.02
    process_noise_pos: float = 0.001
    process_noise_vel: float = 0.01
    measurement_noise_imu: float = 0.05
    measurement_noise_odom: float = 0.02
    initial_cov_pos: float = 0.1
    initial_cov_vel: float = 0.5

class CoMEKF:
    def __init__(self, config: EKFConfig = None):
        self.cfg = config or EKFConfig()
        self.x = np.zeros(4)
        self.F = np.array([[1, 0, self.cfg.dt, 0], [0, 1, 0, self.cfg.dt],
                           [0, 0, 1, 0], [0, 0, 0, 1]])
        self.Q = np.diag([self.cfg.process_noise_pos, self.cfg.process_noise_pos,
                          self.cfg.process_noise_vel, self.cfg.process_noise_vel])
        self.H_imu = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.R_imu = np.eye(2) * self.cfg.measurement_noise_imu
        self.H_odom = np.array([[0, 0, 1, 0], [0, 0, 0, 1]])
        self.R_odom = np.eye(2) * self.cfg.measurement_noise_odom
        self.P = np.diag([self.cfg.initial_cov_pos, self.cfg.initial_cov_pos,
                          self.cfg.initial_cov_vel, self.cfg.initial_cov_vel])

    def reset(self, com_pos, com_vel=None):
        self.x[:2] = com_pos[:2]
        self.x[2:] = com_vel[:2] if com_vel is not None else 0.0
        self.P = np.diag([self.cfg.initial_cov_pos, self.cfg.initial_cov_pos,
                          self.cfg.initial_cov_vel, self.cfg.initial_cov_vel])

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_imu(self, zmp_com_pos):
        z = zmp_com_pos[:2]
        y = z - self.H_imu @ self.x
        S = self.H_imu @ self.P @ self.H_imu.T + self.R_imu
        K = self.P @ self.H_imu.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H_imu) @ self.P

    def update_odom(self, leg_odom_vel):
        z = leg_odom_vel[:2]
        y = z - self.H_odom @ self.x
        S = self.H_odom @ self.P @ self.H_odom.T + self.R_odom
        K = self.P @ self.H_odom.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H_odom) @ self.P

    def get_state(self):
        return self.x[:2].copy(), self.x[2:].copy()

class SupportPhaseDetector:
    def __init__(self, threshold=10.0, hysteresis=5.0):
        self.thresh = threshold
        self.hyst = hysteresis
        self.state = "both"
    def detect(self, left_f, right_f):
        lc = left_f > self.thresh
        rc = right_f > self.thresh
        if lc and rc: self.state = "both"
        elif lc and not rc: self.state = "left"
        elif rc and not lc: self.state = "right"
        else:
            if self.state == "both":
                if left_f < self.thresh - self.hyst and right_f > self.thresh: self.state = "right"
                elif right_f < self.thresh - self.hyst and left_f > self.thresh: self.state = "left"
        return self.state

class StateEstimator:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.ekf = CoMEKF(EKFConfig(dt=dt))
        self.support = SupportPhaseDetector()
        self.com_height = 0.80
        self.base_rpy = np.zeros(3)
        self.base_omega = np.zeros(3)

    def reset(self, initial_com):
        self.ekf.reset(initial_com[:2])
        self.support = SupportPhaseDetector()

    def _quat_to_rpy(self, q):
        x, y, z, w = q
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        sinp = 2*(w*y-z*x)
        pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        return np.array([roll, pitch, yaw])

    def update(self, imu_data, joint_states, foot_force, base_pos, base_orn):
        accel = imu_data.get("accel", np.zeros(3))
        gyro = imu_data.get("gyro", np.zeros(3))
        quat = imu_data.get("quat", np.array([0,0,0,1]))
        self.base_rpy = self._quat_to_rpy(quat)
        self.base_omega = gyro
        support_foot = self.support.detect(foot_force.get("left", 0.0), foot_force.get("right", 0.0))
        # ZMP-based CoM (simplified)
        zmp = base_pos[:2] - (self.com_height / 9.81) * accel[:2]
        com_from_zmp = zmp + (self.com_height / 9.81) * accel[:2]
        # Leg odometry (simplified)
        leg_vel = np.zeros(2)
        self.ekf.predict()
        self.ekf.update_imu(com_from_zmp)
        self.ekf.update_odom(leg_vel)
        com_pos, com_vel = self.ekf.get_state()
        return {
            "com_pos": np.array([com_pos[0], com_pos[1], self.com_height]),
            "com_vel": np.array([com_vel[0], com_vel[1], 0.0]),
            "base_rpy": self.base_rpy.copy(),
            "base_omega": self.base_omega.copy(),
            "support_foot": support_foot,
            "foot_force": foot_force,
        }
