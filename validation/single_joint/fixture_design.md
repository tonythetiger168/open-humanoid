# OpenHumanoid v1.0 - Single Joint Validation Fixture

## Test Fixture Design

### Purpose
Validate individual joint modules (motor + gearbox + encoder + driver) before full robot assembly.

### Fixture Components

```
                    [Load Cell] (optional)
                         |
    [Fixed Base] ---- [Test Joint] ---- [Counterweight Arm]
                         |
                    [Encoder Test]
```

| Component | Specification | Quantity |
|-----------|-------------|----------|
| Fixed base plate | 15mm aluminum 6061, 200x200mm | 1 |
| Motor mount bracket | 10mm aluminum, custom CNC | 1 |
| Output shaft fixture | Steel, keyed, H7 tolerance | 1 |
| Safety cage | 3mm steel wire mesh, 300x300x300mm | 1 |
| Emergency stop button | Red mushroom, NC contact | 2 |
| Limit switch | Mechanical, adjustable | 2 |
| Load cell (optional) | 50kg, strain gauge | 1 |
| Counterweight arm | 200mm aluminum tube | 1 |
| Counterweight | 1-5kg adjustable | 1 set |

### Assembly Steps

1. Mount motor to bracket using M4x12 screws (torque: 4.5 Nm)
2. Attach bracket to base plate
3. Install output shaft fixture on gearbox output
4. Mount limit switches at +/- 170 deg positions
5. Install safety cage around rotating parts
6. Connect E-stop in series with motor driver power
7. Wire encoder to driver
8. Connect CAN bus to test PC

### Safety Checklist

- [ ] E-stop tested: press -> motor power cut within 50ms
- [ ] Limit switches tested: trigger -> immediate stop
- [ ] Safety cage installed and secured
- [ ] No loose clothing/hair near rotating parts
- [ ] Counterweight secured with locking pin
- [ ] Work area clear of bystanders (2m radius)
- [ ] Fire extinguisher accessible
- [ ] First aid kit available

### Joint Under Test (JUT) Configuration

```yaml
# Test configuration per joint type
hip_yaw:
  motor: T-Motor U8 Lite KV100
  gearbox: 7.67:1 planetary
  encoder: AMT102-V 4096 CPR
  driver: ODrive Pro axis0
  test_range: [-0.78, 0.78] rad  # +/- 45 deg
  max_torque: 10.0 Nm
  max_velocity: 3.0 rad/s

hip_pitch:
  motor: T-Motor U8 Lite KV100
  gearbox: 7.67:1 planetary
  encoder: AMT102-V 4096 CPR
  driver: ODrive Pro axis0
  test_range: [-1.57, 1.0] rad
  max_torque: 15.0 Nm
  max_velocity: 3.0 rad/s

knee_pitch:
  motor: T-Motor U8 Lite KV100
  gearbox: 7.67:1 planetary
  encoder: AMT102-V 4096 CPR
  driver: ODrive Pro axis1
  test_range: [-0.1, 2.0] rad
  max_torque: 15.0 Nm
  max_velocity: 4.0 rad/s

ankle_pitch:
  motor: T-Motor U8 Lite KV100
  gearbox: 9.0:1 planetary
  encoder: AMT102-V 4096 CPR
  driver: ODrive Pro axis1
  test_range: [-0.78, 0.52] rad
  max_torque: 8.0 Nm
  max_velocity: 4.0 rad/s
```

## Test Sequence

1. **Pre-test**: Visual inspection, wiring check, E-stop test
2. **Encoder calibration**: Zero point, direction, resolution verification
3. **Friction identification**: Coulomb + viscous friction model
4. **Torque constant identification**: Kt measurement
5. **Step response**: Position/velocity step, measure rise time, overshoot
6. **Sinusoidal tracking**: 0.1-10 Hz frequency sweep
7. **Max performance**: Max velocity, max torque, thermal test
8. **Endurance**: 1000 cycles at rated load
9. **Post-test**: Visual inspection, temperature check, backlash measurement
