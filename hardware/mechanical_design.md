# OpenHumanoid v1.0 - Mechanical Design Deep Dive

## Joint Torque Budget

### Leg Joints (Peak Torque Requirements)

| Joint | Max Torque (Nm) | Safety Factor | Motor Torque (Nm) | Gear Ratio | Output Torque (Nm) | Margin |
|-------|-----------------|---------------|-------------------|------------|-------------------|--------|
| Hip Yaw | 10.0 | 1.5 | 1.8 | 7.67 | 13.8 | 38% |
| Hip Roll | 10.0 | 1.5 | 1.8 | 7.67 | 13.8 | 38% |
| Hip Pitch | 15.0 | 1.5 | 1.8 | 7.67 | 13.8 | -8% |
| Knee Pitch | 15.0 | 1.5 | 1.8 | 7.67 | 13.8 | -8% |
| Ankle Pitch | 8.0 | 1.5 | 1.2 | 9.0 | 10.8 | 35% |
| Ankle Roll | 5.0 | 1.5 | 1.2 | 9.0 | 10.8 | 116% |

**Note**: Hip Pitch and Knee Pitch require either:
- Higher torque motor (2.5 Nm @ 7.67:1 = 19.2 Nm)
- Or higher gear ratio (10:1 with 1.8 Nm = 18 Nm)

### Arm Joints

| Joint | Max Torque (Nm) | Safety Factor | Motor Torque (Nm) | Gear Ratio | Output Torque (Nm) | Margin |
|-------|-----------------|---------------|-------------------|------------|-------------------|--------|
| Shoulder Pitch | 5.0 | 1.5 | 0.8 | 7.67 | 6.1 | 22% |
| Shoulder Roll | 5.0 | 1.5 | 0.8 | 7.67 | 6.1 | 22% |
| Shoulder Yaw | 3.0 | 1.5 | 0.6 | 7.67 | 4.6 | 53% |
| Elbow Pitch | 3.0 | 1.5 | 0.5 | 7.67 | 3.8 | 27% |
| Wrist (all) | 1.5 | 1.5 | 0.3 | 6.0 | 1.8 | 20% |

## Frame Structural Analysis

### Pelvis (Base Link)
- **Load**: 4.5 kg + dynamic forces during walking
- **Max stress**: ~15 MPa (aluminum 6063-T5 yield: 215 MPa)
- **Safety factor**: 14.3
- **Deflection**: < 0.1 mm under max load

### Thigh Link
- **Load**: 2.5 kg + inertial forces
- **Length**: 0.24 m
- **Bending moment**: ~6 Nm at hip
- **Required section modulus**: > 28 mm³
- **2020 extrusion section modulus**: ~35 mm³
- **Safety factor**: 1.25 (use 3030 extrusion for margin)

### Shin Link
- **Load**: 2.0 kg
- **Length**: 0.20 m
- **Bending moment**: ~4 Nm at knee
- **2020 extrusion sufficient**

## Material Selection

| Component | Material | Reason | Alternative |
|-----------|----------|--------|-------------|
| Main frame | 6063-T5 aluminum | Light, stiff, easy to machine | 6061-T6 (stronger) |
| Joint brackets | 7075-T6 aluminum | High strength for stress concentration | Steel (heavier) |
| 3D printed parts | PETG-CF | Tough, impact resistant | PA12-CF (stronger) |
| Foot sole | TPU 95A | Grip, shock absorption | Rubber sheet |
| Motor mounts | Aluminum + steel insert | Heat dissipation + thread strength | Titanium |

## Fastener Specification

| Location | Size | Grade | Torque | Thread Lock |
|----------|------|-------|--------|-------------|
| Motor to bracket | M4x12 | 12.9 | 4.5 Nm | Loctite 243 |
| Bracket to frame | M5x16 | 12.9 | 9.0 Nm | Loctite 243 |
| Frame joints | M5x20 | 12.9 | 9.0 Nm | None (T-slot) |
| Foot attachment | M4x10 | 12.9 | 4.5 Nm | Loctite 243 |
| Camera mount | M3x8 | 12.9 | 2.0 Nm | Loctite 222 |

## Cable Management

### CAN Bus Wiring
- **Topology**: Daisy chain, left leg -> torso -> right leg -> arms -> head
- **Cable**: Twisted pair, 22 AWG, shielded
- **Max segment length**: 2m between nodes
- **Termination**: 120 Ohm at both ends

### Power Distribution
- **Main bus**: 12 AWG silicone wire, 24V
- **Branches**: 16 AWG to each ODrive (2 motors)
- **Fusing**: 30A per ODrive, 60A main fuse
- **Connector**: XT90-S (anti-spark) for battery

### Signal Cables
- **Encoders**: Shielded 4-wire, max 0.5m
- **IMU**: I2C/SPI, shielded, max 0.3m
- **Camera**: USB3, high quality cable, max 1m

## Assembly Tolerances

| Interface | Tolerance | Check Method |
|-----------|-----------|--------------|
| Motor shaft to gearbox | H7/g6 | Go/no-go gauge |
| Gearbox to bracket | 0.05mm | Feeler gauge |
| Bracket to frame | 0.1mm | Visual + feel |
| Foot to ground | 0.5mm | Level gauge |
| Joint axis alignment | 0.5 deg | Digital level |

## Maintenance Schedule

| Interval | Task |
|----------|------|
| Weekly | Check fasteners for loosening |
| Monthly | Inspect cables for wear |
| Quarterly | Re-grease gearboxes |
| Bi-annually | Calibrate encoders |
| Annually | Replace foot pads, inspect frame for cracks |
