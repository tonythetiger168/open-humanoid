# OpenHumanoid v1.0 - Sim-to-Real Migration Guide

## Overview

This guide documents the process of transferring controllers and policies from simulation to the physical robot.

## Phase 1: System Identification

### 1.1 Joint Dynamics Identification

For each joint, identify:
- **Motor torque constant (Kt)**: Apply known current, measure output torque
- **Friction model**: Coulomb + viscous friction vs. velocity
- **Backlash**: Measure hysteresis in position response
- **Gear ratio accuracy**: Verify actual vs. nominal ratio
- **Inertia**: Measure reflected inertia at output shaft

```python
# Identification procedure
for joint in all_joints:
    # Step 1: Apply sinusoidal torque, measure position response
    # Step 2: Fit second-order model: J*ddq + B*dq + tau_f = tau_cmd
    # Step 3: Validate with step response
```

### 1.2 Mass Properties Identification

- Weigh each link independently
- Measure CoM location using balance method
- Measure inertia using pendulum method or CAD
- Update URDF with measured values

### 1.3 Contact Model Identification

- Measure foot-ground friction coefficient (various surfaces)
- Measure foot stiffness/damping
- Identify ground reaction force model

## Phase 2: Domain Randomization

### 2.1 Parameter Ranges

| Parameter | Simulation Value | Randomization Range | Physical Check |
|-----------|---------------|---------------------|----------------|
| Mass | URDF value | +/- 10% | Weigh links |
| CoM | URDF value | +/- 2cm | Balance test |
| Inertia | URDF value | +/- 15% | Pendulum test |
| Friction | 0.8 | 0.5 - 1.2 | Incline test |
| Motor strength | 1.0 | 0.8 - 1.2 | Torque test |
| Joint damping | 0.05 | 0.02 - 0.10 | Step response |
| Sensor delay | 0ms | 0 - 20ms | Loopback test |
| Sensor noise | 0 | Add measured noise | Calibration |

### 2.2 Implementation

```python
# In RL training
if domain_randomize:
    env.randomize_mass(scale=np.random.uniform(0.9, 1.1))
    env.randomize_friction(np.random.uniform(0.5, 1.2))
    env.add_sensor_noise(measured_noise_profile)
```

## Phase 3: Controller Tuning

### 3.1 PD Gain Tuning

1. Start with simulation gains
2. Reduce by 30% for safety
3. Tune each joint individually:
   - Increase Kp until oscillation
   - Back off 30% from oscillation point
   - Add Kd to dampen overshoot
4. Test under load (robot standing)

### 3.2 Balance Controller Tuning

| Parameter | Sim Value | Real Robot Start | Tuning Method |
|-----------|-----------|------------------|---------------|
| Kp_com | 100.0 | 50.0 | Increase until stable |
| Kd_com | 20.0 | 15.0 | Reduce oscillation |
| Kp_orientation | 50.0 | 30.0 | Prevent tipping |
| Preview steps | 320 | 160 | Reduce for latency |
| CoM height | 0.80m | 0.75m | Measure actual |

### 3.3 Gait Parameter Tuning

| Parameter | Sim Value | Real Robot Start | Notes |
|-----------|-----------|------------------|-------|
| Step length | 0.15m | 0.05m | Start small |
| Step height | 0.05m | 0.03m | Reduce impact |
| Step duration | 0.8s | 1.2s | Slower initially |
| Double support | 20% | 30% | More stability |

## Phase 4: Validation Checklist

### Single Joint Tests
- [ ] Each joint moves smoothly through full range
- [ ] Position tracking error < 0.01 rad at steady state
- [ ] No overheating after 5 minutes continuous motion
- [ ] Encoder calibration verified

### Static Tests
- [ ] Robot stands stable for 60 seconds
- [ ] Can withstand 5N push from front/side/back
- [ ] Ankle PD correction works for small tilts
- [ ] Emergency stop responds within 100ms

### Dynamic Tests
- [ ] Weight shifting left/right without falling
- [ ] Single leg stand (3 seconds each)
- [ ] Squat motion (slow, controlled)
- [ ] Step in place (minimal forward motion)

### Walking Tests
- [ ] 3 steps forward without falling
- [ ] 10 steps forward (3 meters)
- [ ] Turn in place (90 degrees)
- [ ] Walk over 1cm obstacle
- [ ] Recovery from mild push during walking

## Phase 5: Policy Deployment (RL)

### 5.1 Action Space Mapping

Simulation action -> Real action:
```python
# Add action filter
action_real = alpha * action_sim + (1-alpha) * action_prev
# alpha = 0.3 for smooth transition

# Add action offset (sim-to-real gap)
action_real += action_bias  # Learned from real data
```

### 5.2 Observation Space Adaptation

```python
# Normalize observations using real robot statistics
obs_real = (obs_raw - obs_mean_real) / obs_std_real
# obs_mean_real, obs_std_real collected from 1000 steps on real robot
```

### 5.3 Safety Wrappers

```python
class RealRobotSafetyWrapper:
    def __init__(self, policy):
        self.policy = policy
        self.safety = SafetyMonitor()

    def act(self, obs):
        action = self.policy.predict(obs)
        # Clip action
        action = np.clip(action, -0.3, 0.3)
        # Check safety
        if self.safety.level != SafetyLevel.NORMAL:
            return np.zeros_like(action)  # Zero action
        return action
```

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Oscillation | Gains too high | Reduce Kp by 30% |
| Slow response | Gains too low / latency | Increase Kp, reduce preview steps |
| Drift during stand | IMU bias / friction | Calibrate IMU, add integral term |
| Hard landing | Swing trajectory too aggressive | Reduce step height, add landing phase |
| One leg weaker | Motor calibration error | Re-calibrate, add torque offset |
| Communication jitter | CAN bus load | Reduce message frequency, optimize IDs |
| Overheating | Current too high | Reduce acceleration limits, add cooling |

## Metrics for Success

| Metric | Simulation | Real Robot Target |
|--------|-----------|-------------------|
| Stand stability (CoM drift) | < 1cm | < 2cm |
| Walking speed | 0.5 m/s | 0.3 m/s (initial) |
| Energy per meter | 50 J/m | < 100 J/m |
| Fall rate | 0% | < 5% (100 trials) |
| Recovery time | N/A | < 3 seconds |
