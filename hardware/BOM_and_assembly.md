# OpenHumanoid v1.0 - Hardware BOM & Assembly Guide

## Bill of Materials

| Category | Part | Spec | Qty | Est. Cost (USD) | Supplier |
|----------|------|------|-----|-----------------|----------|
| **Actuators** | BLDC Motor | T-Motor U8 Lite KV100 | 28 | $140 x 28 = $3,920 | T-Motor |
| | Planetary Gearbox | 7.67:1, max 30Nm | 28 | $80 x 28 = $2,240 | Custom/AliExpress |
| | Motor Driver | ODrive Pro (2-axis) | 14 | $150 x 14 = $2,100 | ODrive Robotics |
| | Encoder | AMT102-V (4096 CPR) | 28 | $25 x 28 = $700 | CUI Devices |
| **Frame** | Aluminum Extrusion | 2020 / 3030, 6063-T5 | ~6m | $80 | Misumi |
| | 3D Printed Parts | PETG-CF / PLA-CF | ~2kg | $60 | Self-print |
| | Fasteners | M3/M4/M5 screws, nuts | 200+ | $40 | McMaster |
| **Electronics** | Compute | NVIDIA Jetson AGX Orin 64GB | 1 | $1,599 | NVIDIA |
| | Depth Camera | Intel RealSense D455 | 1 | $350 | Intel |
| | IMU | ICM-42688-P 6-axis | 1 | $15 | TDK |
| | CAN Transceiver | MCP2517FD x4 | 4 | $8 x 4 = $32 | Microchip |
| | Power Distribution | 24V PDB, 60A fuse | 1 | $45 | Custom |
| | BMS | 8S LiFePO4 BMS 40A | 1 | $60 | Overkill Solar |
| **Power** | Battery | 24V 20Ah LiFePO4 pack | 1 | $280 | BatteryHookup |
| | Charger | 29.2V 5A LiFePO4 | 1 | $45 | Amazon |
| **Cables** | CAN Bus | Twisted pair, 22AWG | 10m | $20 | DigiKey |
| | Power Cable | 12AWG silicone | 5m | $25 | Amazon |
| | Signal Cable | JST-XH, Molex | 50pcs | $30 | Amazon |
| **Misc** | Cooling | 40mm fan x4, heatsink | 1 set | $35 | Amazon |
| | Emergency Stop | Red mushroom E-stop | 1 | $15 | Amazon |
| | Foot Pads | Rubber, 3mm thick | 2 | $10 | Amazon |

**Total Estimated Cost: ~$11,700 USD**

## Mechanical Design Notes

### Joint Layout (28 DoF)
```
Head:     [Neck Yaw] -- [Neck Pitch] -- [Camera]
Torso:    [Spine Pitch] -- [Spine Yaw]
Arm:      [Shoulder Pitch] -- [Shoulder Roll] -- [Shoulder Yaw]
          -- [Elbow Pitch] -- [Wrist Roll] -- [Wrist Pitch] -- [Wrist Yaw] -- [Hand]
Leg:      [Hip Yaw] -- [Hip Roll] -- [Hip Pitch]
          -- [Knee Pitch] -- [Ankle Pitch] -- [Ankle Roll] -- [Foot]
```

### Key Dimensions
- Hip height (standing): ~0.75m
- Thigh length: 0.24m
- Shin length: 0.20m
- Foot length: 0.20m
- Shoulder width: 0.20m
- Arm reach: ~0.55m

### Assembly Sequence
1. **Legs**: Assemble hip yaw/roll/pitch modules → thigh link → knee → shin → ankle → foot
2. **Torso**: Mount pelvis frame → attach spine joints → chest frame
3. **Arms**: Shoulder assembly → upper arm → elbow → forearm → wrist → hand
4. **Head**: Neck yaw base → pitch joint → camera mount
5. **Integration**: Attach legs to pelvis → torso → arms → head
6. **Wiring**: CAN bus daisy chain → power distribution → sensor cables
7. **Calibration**: Zero all joints → verify encoder alignment → test single joint motion

## Safety Checklist
- [ ] Emergency stop functional
- [ ] Joint limits software + hardware enforced
- [ ] Battery temperature monitoring
- [ ] Current limiting on all motor drivers
- [ ] Mechanical interlocks on high-torque joints (hip/knee)
- [ ] Fall detection + automatic power cut
