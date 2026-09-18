#!/usr/bin/env python3
"""OpenHumanoid v1.0 - PyBullet Simulation Environment"""
import pybullet as p
import pybullet_data
import numpy as np
import time
import json
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

@dataclass
class SimConfig:
    urdf_path: str = "design/open_humanoid_v1.urdf"
    sim_freq: float = 240.0
    control_freq: float = 50.0
    gravity: Tuple[float, float, float] = (0, 0, -9.81)
    time_step: float = 1.0 / 240.0
    use_gui: bool = True
    use_real_time: bool = False
    initial_base_pos: Tuple[float, float, float] = (0.0, 0.0, 1.05)
    initial_base_orn: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    kp: float = 50.0
    kd: float = 2.0
    max_force: float = 100.0
    terrain_type: str = "flat"
    camera_distance: float = 2.5
    camera_yaw: float = 45.0
    camera_pitch: float = -30.0

class TerrainGenerator:
    TYPES = ["flat", "slope", "stairs", "uneven"]
    def __init__(self, client):
        self.client = client
    def generate(self, terrain_type: str, scale: float = 10.0) -> int:
        if terrain_type == "flat":
            shape = p.createCollisionShape(p.GEOM_PLANE)
            vis = p.createVisualShape(p.GEOM_PLANE, rgbaColor=[0.25, 0.25, 0.30, 1.0], planeNormal=[0, 0, 1])
            body = p.createMultiBody(0, shape, vis, [0, 0, 0], [0, 0, 0, 1])
            p.changeDynamics(body, -1, lateralFriction=0.8)
            return body
        elif terrain_type == "slope":
            he = [scale, scale, 0.05]
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=he)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=[0.35, 0.30, 0.25, 1.0])
            body = p.createMultiBody(0, shape, vis, [scale*0.5, 0, -0.05], p.getQuaternionFromEuler([0, -0.1745, 0]))
            p.changeDynamics(body, -1, lateralFriction=0.8)
            return body
        elif terrain_type == "stairs":
            base = self.generate("flat", scale)
            for i in range(8):
                he = [0.15, 0.15, 0.025]
                shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=he)
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=[0.4, 0.35, 0.3, 1.0])
                pos = [i*0.3+0.15, 0, i*0.05+0.025]
                body = p.createMultiBody(0, shape, vis, pos, [0,0,0,1])
                p.changeDynamics(body, -1, lateralFriction=0.9)
            return base
        elif terrain_type == "uneven":
            base = self.generate("flat", scale)
            for _ in range(20):
                r = np.random.uniform(0.3, 0.8)
                h = np.random.uniform(0.02, 0.08)
                shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
                vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=[0.3, 0.35, 0.25, 1.0])
                pos = [np.random.uniform(-scale/2, scale/2), np.random.uniform(-scale/2, scale/2), h/2]
                body = p.createMultiBody(0, shape, vis, pos, [0,0,0,1])
                p.changeDynamics(body, -1, lateralFriction=0.7)
            return base
        raise ValueError(f"Unknown terrain: {terrain_type}")

class OpenHumanoidSim:
    JOINT_NAMES = [
        'l_hip_yaw_joint', 'l_hip_roll_joint', 'l_hip_pitch_joint',
        'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint',
        'r_hip_yaw_joint', 'r_hip_roll_joint', 'r_hip_pitch_joint',
        'r_knee_pitch_joint', 'r_ankle_pitch_joint', 'r_ankle_roll_joint',
        'spine_pitch_joint', 'spine_yaw_joint',
        'l_shoulder_pitch_joint', 'l_shoulder_roll_joint', 'l_shoulder_yaw_joint',
        'l_elbow_pitch_joint', 'l_wrist_roll_joint', 'l_wrist_pitch_joint', 'l_wrist_yaw_joint',
        'r_shoulder_pitch_joint', 'r_shoulder_roll_joint', 'r_shoulder_yaw_joint',
        'r_elbow_pitch_joint', 'r_wrist_roll_joint', 'r_wrist_pitch_joint', 'r_wrist_yaw_joint',
        'neck_yaw_joint', 'neck_pitch_joint',
    ]

    def __init__(self, config: SimConfig = None):
        self.cfg = config or SimConfig()
        self.physics_client = None
        self.robot_id = None
        self.terrain_id = None
        self.joint_map: Dict[str, int] = {}
        self.joint_limits: Dict[str, Tuple[float, float]] = {}
        self.num_joints = 0
        self.control_dt = 1.0 / self.cfg.control_freq
        self._last_control_time = 0.0
        self._sim_time = 0.0
        self.imu_data = {"accel": np.zeros(3), "gyro": np.zeros(3), "quat": np.array([0,0,0,1])}
        self.joint_states = {}
        self.foot_force = {"left": 0.0, "right": 0.0}
        self.base_pose = {"pos": np.zeros(3), "orn": np.zeros(4), "vel": np.zeros(3), "omega": np.zeros(3)}
        self.log = []

    def init(self) -> bool:
        try:
            if self.cfg.use_gui:
                self.physics_client = p.connect(p.GUI)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
                p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            else:
                self.physics_client = p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(*self.cfg.gravity)
            p.setTimeStep(self.cfg.time_step)
            p.setPhysicsEngineParameter(numSolverIterations=50)
            # Terrain
            tg = TerrainGenerator(self.physics_client)
            self.terrain_id = tg.generate(self.cfg.terrain_type)
            # Robot
            start_pos = list(self.cfg.initial_base_pos)
            start_orn = p.getQuaternionFromEuler(self.cfg.initial_base_orn)
            urdf = self.cfg.urdf_path
            if not os.path.isabs(urdf):
                urdf = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", urdf)
            self.robot_id = p.loadURDF(urdf, start_pos, start_orn, useFixedBase=False,
                                       flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE)
            self._build_joint_map()
            self._set_default_pose()
            print(f"[Sim] Loaded robot ID={self.robot_id}, joints={self.num_joints}")
            return True
        except Exception as e:
            print(f"[Sim] Init failed: {e}")
            return False

    def _build_joint_map(self):
        self.num_joints = p.getNumJoints(self.robot_id)
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            name = info[1].decode("utf-8")
            self.joint_map[name] = i
            self.joint_limits[name] = (info[8], info[9])
            p.setJointMotorControl2(self.robot_id, i, p.VELOCITY_CONTROL, force=0)

    def _set_default_pose(self):
        defaults = {
            'l_hip_yaw_joint': 0.0, 'l_hip_roll_joint': 0.0, 'l_hip_pitch_joint': -0.15,
            'l_knee_pitch_joint': 0.30, 'l_ankle_pitch_joint': -0.15, 'l_ankle_roll_joint': 0.0,
            'r_hip_yaw_joint': 0.0, 'r_hip_roll_joint': 0.0, 'r_hip_pitch_joint': -0.15,
            'r_knee_pitch_joint': 0.30, 'r_ankle_pitch_joint': -0.15, 'r_ankle_roll_joint': 0.0,
            'spine_pitch_joint': 0.0, 'spine_yaw_joint': 0.0,
            'l_shoulder_pitch_joint': 0.0, 'l_shoulder_roll_joint': 0.0, 'l_shoulder_yaw_joint': 0.0,
            'l_elbow_pitch_joint': -0.5, 'l_wrist_roll_joint': 0.0, 'l_wrist_pitch_joint': 0.0, 'l_wrist_yaw_joint': 0.0,
            'r_shoulder_pitch_joint': 0.0, 'r_shoulder_roll_joint': 0.0, 'r_shoulder_yaw_joint': 0.0,
            'r_elbow_pitch_joint': -0.5, 'r_wrist_roll_joint': 0.0, 'r_wrist_pitch_joint': 0.0, 'r_wrist_yaw_joint': 0.0,
            'neck_yaw_joint': 0.0, 'neck_pitch_joint': 0.0,
        }
        for name, pos in defaults.items():
            if name in self.joint_map:
                p.resetJointState(self.robot_id, self.joint_map[name], pos)

    def read_sensors(self):
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        vel, omega = p.getBaseVelocity(self.robot_id)
        self.base_pose["pos"] = np.array(pos)
        self.base_pose["orn"] = np.array(orn)
        self.base_pose["vel"] = np.array(vel)
        self.base_pose["omega"] = np.array(omega)
        self.imu_data["gyro"] = np.array(omega)
        self.imu_data["quat"] = np.array(orn)
        # Approximate accel
        self.imu_data["accel"] = np.array([0, 0, 9.81]) + np.array(vel) * 0.1
        # Joint states
        self.joint_states = {}
        for name, idx in self.joint_map.items():
            jpos, jvel, _, _ = p.getJointState(self.robot_id, idx)
            self.joint_states[name] = {"position": jpos, "velocity": jvel, "effort": 0.0}
        # Foot force
        lf = rf = 0.0
        contacts = p.getContactPoints(self.robot_id, self.terrain_id)
        for c in contacts:
            link_idx = c[3]
            force = c[9]
            # Map to foot links
            if link_idx == self.joint_map.get("l_foot_joint", -1) - 1:
                lf += force
            elif link_idx == self.joint_map.get("r_foot_joint", -1) - 1:
                rf += force
        self.foot_force = {"left": lf, "right": rf}

    def set_joint_positions(self, positions: Dict[str, float], max_force: float = None):
        force = max_force or self.cfg.max_force
        for name, pos in positions.items():
            if name in self.joint_map:
                idx = self.joint_map[name]
                p.setJointMotorControl2(self.robot_id, idx, p.POSITION_CONTROL,
                                        targetPosition=pos, positionGain=self.cfg.kp,
                                        velocityGain=self.cfg.kd, force=force)

    def set_joint_torques(self, torques: Dict[str, float]):
        for name, tau in torques.items():
            if name in self.joint_map:
                p.setJointMotorControl2(self.robot_id, self.joint_map[name], p.TORQUE_CONTROL, force=tau)

    def get_joint_pos_array(self, names: List[str] = None) -> np.ndarray:
        names = names or self.JOINT_NAMES
        return np.array([self.joint_states[n]["position"] for n in names if n in self.joint_states])

    def step(self, controller_callback=None) -> dict:
        p.stepSimulation()
        self._sim_time += self.cfg.time_step
        if self._sim_time - self._last_control_time >= self.control_dt:
            self.read_sensors()
            if controller_callback:
                controller_callback(self)
            self._last_control_time = self._sim_time
        if self.cfg.use_gui:
            p.resetDebugVisualizerCamera(self.cfg.camera_distance, self.cfg.camera_yaw,
                                         self.cfg.camera_pitch, (0, 0, 0.5))
        if self.cfg.use_real_time:
            time.sleep(self.cfg.time_step)
        return {"time": self._sim_time, "base_z": self.base_pose["pos"][2],
                "foot_left": self.foot_force["left"], "foot_right": self.foot_force["right"]}

    def run(self, duration: float = 10.0, controller_callback=None):
        steps = int(duration / self.cfg.time_step)
        for i in range(steps):
            self.step(controller_callback)
        return self.log

    def close(self):
        if self.physics_client is not None:
            p.disconnect(self.physics_client)

    def __enter__(self):
        self.init()
        return self
    def __exit__(self, *args):
        self.close()
        return False
