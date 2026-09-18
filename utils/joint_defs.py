# Joint definitions for OpenHumanoid v1.0
# 28 DoF total

JOINT_NAMES = [
    # Left leg (6)
    'l_hip_yaw_joint', 'l_hip_roll_joint', 'l_hip_pitch_joint',
    'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint',
    # Right leg (6)
    'r_hip_yaw_joint', 'r_hip_roll_joint', 'r_hip_pitch_joint',
    'r_knee_pitch_joint', 'r_ankle_pitch_joint', 'r_ankle_roll_joint',
    # Torso (2)
    'spine_pitch_joint', 'spine_yaw_joint',
    # Left arm (7)
    'l_shoulder_pitch_joint', 'l_shoulder_roll_joint', 'l_shoulder_yaw_joint',
    'l_elbow_pitch_joint', 'l_wrist_roll_joint', 'l_wrist_pitch_joint', 'l_wrist_yaw_joint',
    # Right arm (7)
    'r_shoulder_pitch_joint', 'r_shoulder_roll_joint', 'r_shoulder_yaw_joint',
    'r_elbow_pitch_joint', 'r_wrist_roll_joint', 'r_wrist_pitch_joint', 'r_wrist_yaw_joint',
    # Head (2)
    'neck_yaw_joint', 'neck_pitch_joint',
]

LEG_JOINTS = JOINT_NAMES[0:12]
LEFT_LEG_JOINTS = JOINT_NAMES[0:6]
RIGHT_LEG_JOINTS = JOINT_NAMES[6:12]
TORSO_JOINTS = JOINT_NAMES[12:14]
LEFT_ARM_JOINTS = JOINT_NAMES[14:21]
RIGHT_ARM_JOINTS = JOINT_NAMES[21:28]
HEAD_JOINTS = JOINT_NAMES[28:30]

# Default standing pose
DEFAULT_STAND_POSE = {
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

# Joint limits (from URDF)
JOINT_LIMITS = {
    'l_hip_yaw_joint': (-0.78, 0.78), 'l_hip_roll_joint': (-0.52, 0.52), 'l_hip_pitch_joint': (-1.57, 1.0),
    'l_knee_pitch_joint': (-0.1, 2.0), 'l_ankle_pitch_joint': (-0.78, 0.52), 'l_ankle_roll_joint': (-0.35, 0.35),
    'r_hip_yaw_joint': (-0.78, 0.78), 'r_hip_roll_joint': (-0.52, 0.52), 'r_hip_pitch_joint': (-1.57, 1.0),
    'r_knee_pitch_joint': (-0.1, 2.0), 'r_ankle_pitch_joint': (-0.78, 0.52), 'r_ankle_roll_joint': (-0.35, 0.35),
    'spine_pitch_joint': (-0.35, 0.35), 'spine_yaw_joint': (-0.52, 0.52),
    'l_shoulder_pitch_joint': (-2.0, 2.0), 'l_shoulder_roll_joint': (-1.57, 0.5), 'l_shoulder_yaw_joint': (-1.57, 1.57),
    'l_elbow_pitch_joint': (-2.5, 0.0), 'l_wrist_roll_joint': (-1.57, 1.57), 'l_wrist_pitch_joint': (-0.78, 0.78), 'l_wrist_yaw_joint': (-1.0, 1.0),
    'r_shoulder_pitch_joint': (-2.0, 2.0), 'r_shoulder_roll_joint': (-0.5, 1.57), 'r_shoulder_yaw_joint': (-1.57, 1.57),
    'r_elbow_pitch_joint': (-2.5, 0.0), 'r_wrist_roll_joint': (-1.57, 1.57), 'r_wrist_pitch_joint': (-0.78, 0.78), 'r_wrist_yaw_joint': (-1.0, 1.0),
    'neck_yaw_joint': (-1.57, 1.57), 'neck_pitch_joint': (-0.78, 0.78),
}
