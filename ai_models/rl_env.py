#!/usr/bin/env python3
"""OpenHumanoid v1.0 - RL Training Environment (Gymnasium Wrapper)

Compatible with: Stable-Baselines3, RLlib, CleanRL
Observation Space: (joint_pos, joint_vel, imu, foot_force, command)
Action Space: target joint positions (12 leg joints) or CoM offset
Reward: balance + forward_velocity - energy - fall_penalty
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from typing import Dict, Tuple, Optional
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from simulation.pybullet_env import OpenHumanoidSim, SimConfig
from control.main_controller import RobotController, ControlMode

class HumanoidRLEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode: Optional[str] = None, terrain: str = "flat",
                 target_velocity: float = 0.5, max_episode_steps: int = 1000):
        super().__init__()
        self.render_mode = render_mode
        self.target_velocity = target_velocity
        self.max_episode_steps = max_episode_steps

        # Simulation
        cfg = SimConfig(
            urdf_path="design/open_humanoid_v1.urdf",
            use_gui=(render_mode == "human"),
            terrain_type=terrain,
            sim_freq=240.0,
            control_freq=50.0
        )
        self.sim = OpenHumanoidSim(cfg)
        self.sim.init()
        self.controller = RobotController(self.sim, mode=ControlMode.STAND)

        # Observation space: 12 leg joints pos + vel + 6 IMU + 2 foot force + 3 command = 35
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(35,), dtype=np.float32
        )

        # Action space: 12 leg joint position offsets (delta from default stand)
        self.action_space = spaces.Box(
            low=-0.5, high=0.5, shape=(12,), dtype=np.float32
        )

        self.step_count = 0
        self.prev_base_pos = np.zeros(3)
        self.energy_accumulated = 0.0

    def _get_obs(self) -> np.ndarray:
        # Leg joint positions (12)
        leg_pos = self.sim.get_joint_pos_array(self.sim.JOINT_NAMES[:12])
        # Leg joint velocities (12)
        leg_vel = self.sim.get_joint_velocities_array(self.sim.JOINT_NAMES[:12])
        # IMU: roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel (6)
        imu = np.array([
            self.sim.base_pose["orn"][0], self.sim.base_pose["orn"][1], self.sim.base_pose["orn"][2],
            self.sim.base_pose["omega"][0], self.sim.base_pose["omega"][1], self.sim.base_pose["omega"][2]
        ])
        # Foot forces (2)
        foot = np.array([self.sim.foot_force["left"], self.sim.foot_force["right"]])
        # Command (3)
        cmd = np.array([self.target_velocity, 0.0, 0.0])

        obs = np.concatenate([leg_pos, leg_vel, imu, foot, cmd])
        return obs.astype(np.float32)

    def _compute_reward(self, action: np.ndarray) -> Tuple[float, Dict]:
        # 1. Forward velocity reward
        base_vel_x = (self.sim.base_pose["pos"][0] - self.prev_base_pos[0]) / 0.02
        vel_reward = -abs(base_vel_x - self.target_velocity) * 2.0

        # 2. Balance reward (keep upright)
        roll, pitch = self.sim.base_pose["orn"][0], self.sim.base_pose["orn"][1]
        balance_reward = - (roll**2 + pitch**2) * 10.0

        # 3. Height reward
        height_reward = -abs(self.sim.base_pose["pos"][2] - 0.95) * 5.0

        # 4. Energy penalty
        energy_penalty = -np.sum(action**2) * 0.1
        self.energy_accumulated += np.sum(action**2)

        # 5. Foot contact reward
        contact_reward = 0.0
        if self.sim.foot_force["left"] > 10 and self.sim.foot_force["right"] > 10:
            contact_reward = 0.5

        # 6. Fall penalty
        fall_penalty = 0.0
        if self.sim.base_pose["pos"][2] < 0.4 or abs(roll) > 1.0 or abs(pitch) > 1.0:
            fall_penalty = -100.0

        total_reward = vel_reward + balance_reward + height_reward + energy_penalty + contact_reward + fall_penalty

        info = {
            "vel_reward": vel_reward,
            "balance_reward": balance_reward,
            "height_reward": height_reward,
            "energy_penalty": energy_penalty,
            "contact_reward": contact_reward,
            "fall_penalty": fall_penalty,
            "base_z": self.sim.base_pose["pos"][2],
            "base_vel_x": base_vel_x,
        }
        return total_reward, info

    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None):
        super().reset(seed=seed)
        self.sim._set_default_pose()
        self.controller.reset()
        self.step_count = 0
        self.prev_base_pos = self.sim.base_pose["pos"].copy()
        self.energy_accumulated = 0.0

        # Domain randomization
        if options and options.get("domain_randomize", False):
            # Randomize CoM height slightly
            self.controller.balance.cfg.com_height = np.random.uniform(0.75, 0.85)
            # Randomize foot friction
            # (would need PyBullet API)

        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action: np.ndarray):
        # Apply action: add offsets to default standing pose
        default = {
            'l_hip_yaw_joint': 0.0, 'l_hip_roll_joint': 0.0, 'l_hip_pitch_joint': -0.15,
            'l_knee_pitch_joint': 0.30, 'l_ankle_pitch_joint': -0.15, 'l_ankle_roll_joint': 0.0,
            'r_hip_yaw_joint': 0.0, 'r_hip_roll_joint': 0.0, 'r_hip_pitch_joint': -0.15,
            'r_knee_pitch_joint': 0.30, 'r_ankle_pitch_joint': -0.15, 'r_ankle_roll_joint': 0.0,
        }
        leg_names = list(default.keys())
        targets = {}
        for i, name in enumerate(leg_names):
            targets[name] = default[name] + action[i]

        # Step simulation multiple times for control freq
        for _ in range(int(self.sim.cfg.sim_freq / self.sim.cfg.control_freq)):
            self.sim.step(controller_callback=lambda s: s.set_joint_positions(targets, max_force=80.0))

        obs = self._get_obs()
        reward, info = self._compute_reward(action)

        # Check termination
        terminated = False
        truncated = False
        if self.sim.base_pose["pos"][2] < 0.4 or abs(self.sim.base_pose["orn"][0]) > 1.0 or abs(self.sim.base_pose["orn"][1]) > 1.0:
            terminated = True
        self.step_count += 1
        if self.step_count >= self.max_episode_steps:
            truncated = True

        self.prev_base_pos = self.sim.base_pose["pos"].copy()
        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "rgb_array":
            rgb, _ = self.sim.get_camera_image()
            return rgb
        return None

    def close(self):
        self.sim.close()

# ============================================================
# Training Script (PPO with Stable-Baselines3)
# ============================================================
"""
Usage:
    python3 ai_models/rl_train.py --algo ppo --timesteps 1_000_000

Example:
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv
    from stable_baselines3.common.callbacks import CheckpointCallback

    def make_env():
        return HumanoidRLEnv(terrain="flat", target_velocity=0.5)

    env = DummyVecEnv([make_env] * 4)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./logs/")

    checkpoint = CheckpointCallback(save_freq=10000, save_path="./models/", name_prefix="ppo_humanoid")
    model.learn(total_timesteps=1_000_000, callback=checkpoint)
    model.save("models/ppo_humanoid_final")
"""
