# OpenHumanoid v1.0 - TF Tree Specification

## Coordinate Frames

```
world (fixed)
  └── base_link (pelvis, floating base)
        ├── imu_link
        ├── neck_yaw_link
        │     └── neck_pitch_link
        │           └── head_camera_link
        ├── torso_link
        │     └── chest_link
        │           ├── l_shoulder_pitch_link
        │           │     └── l_shoulder_roll_link
        │           │           └── l_shoulder_yaw_link
        │           │                 └── l_elbow_pitch_link
        │           │                       └── l_wrist_roll_link
        │           │                             └── l_wrist_pitch_link
        │           │                                   └── l_hand_link
        │           ├── r_shoulder_pitch_link
        │           │     └── r_shoulder_roll_link
        │           │           └── r_shoulder_yaw_link
        │           │                 └── r_elbow_pitch_link
        │           │                       └── r_wrist_roll_link
        │           │                             └── r_wrist_pitch_link
        │           │                                   └── r_hand_link
        ├── l_hip_yaw_link
        │     └── l_hip_roll_link
        │           └── l_hip_pitch_link
        │                 └── l_knee_pitch_link
        │                       └── l_ankle_pitch_link
        │                             └── l_ankle_roll_link
        │                                   └── l_foot_link
        │                                         └── l_foot_force_link
        └── r_hip_yaw_link
              └── r_hip_roll_link
                    └── r_hip_pitch_link
                          └── r_knee_pitch_link
                                └── r_ankle_pitch_link
                                      └── r_ankle_roll_link
                                            └── r_foot_link
                                                  └── r_foot_force_link
```

## Frame Definitions

| Frame | Parent | Origin | Purpose |
|-------|--------|--------|---------|
| `world` | - | (0,0,0) | Global reference frame |
| `base_link` | `world` | CoM of pelvis | Robot base (6 DoF floating) |
| `imu_link` | `base_link` | (0,0,0) | IMU sensor location |
| `head_camera_link` | `neck_pitch_link` | Forward 6cm, up 2cm | RealSense D455 mount |
| `l_foot_force_link` | `l_foot_link` | Center of sole | Left foot force sensor |
| `r_foot_force_link` | `r_foot_link` | Center of sole | Right foot force sensor |

## Static Transforms (TF2)

Published by `robot_state_publisher` from URDF.

## Dynamic Transforms

| Transform | Publisher | Rate | Source |
|-----------|-----------|------|--------|
| `world -> base_link` | EKF | 50 Hz | IMU + leg odometry |
| `base_link -> imu_link` | Static | - | URDF |
| `base_link -> head_camera_link` | Static | - | URDF (via neck joints) |

## SLAM Integration

When visual SLAM is active:
- `world -> odom` published by SLAM node
- `odom -> base_link` published by EKF (fusing SLAM + IMU + odometry)
