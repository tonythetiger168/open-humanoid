# OpenHumanoid v1.0 - CAN-FD Communication Protocol
# Bus: CAN-FD @ 1 Mbps (nominal), 4 Mbps (data phase)
# Frame format: Extended ID (29-bit)

## ID Allocation (29-bit Extended Frame)

```
Bit 28-24: Device Class (5 bits)
  0x00 = Reserved
  0x01 = Left Leg
  0x02 = Right Leg
  0x03 = Torso
  0x04 = Left Arm
  0x05 = Right Arm
  0x06 = Head
  0x07 = Power/BMS
  0x08 = IMU/Sensors
  0x09 = Emergency/ Safety
  0x0A-0x1F = Reserved

Bit 23-16: Device ID (8 bits) - Joint index or sensor ID
Bit 15-8:  Message Type (8 bits)
  0x01 = Position Command
  0x02 = Velocity Command
  0x03 = Torque Command
  0x10 = Position Feedback
  0x11 = Velocity Feedback
  0x12 = Torque Feedback
  0x20 = Status/Error
  0x30 = Parameter Set
  0x31 = Parameter Get
  0x40 = Heartbeat
  0x50 = Emergency Stop
  0x51 = Emergency Reset
  0x60 = Bootloader

Bit 7-0:   Sequence Number (8 bits) - For multi-frame messages
```

## Message Payload (64 bytes max, CAN-FD)

### Position Command (0x01)
```
Byte 0-3:  Target Position (float32, rad)
Byte 4-7:  Position Gain Kp (float32)
Byte 8-11: Velocity Gain Kd (float32)
Byte 12:   Control Mode (uint8)
  0x00 = Position Control
  0x01 = Velocity Control
  0x02 = Torque Control
  0x03 = Impedance Control
Byte 13-15: Reserved
```

### Torque Command (0x03)
```
Byte 0-3:  Target Torque (float32, Nm)
Byte 4-7:  Feedforward Velocity (float32, rad/s)
Byte 8-11: Feedforward Acceleration (float32, rad/s^2)
Byte 12-15: Reserved
```

### Status/Error Feedback (0x20)
```
Byte 0:   Error Code (uint8)
  0x00 = OK
  0x01 = Overcurrent
  0x02 = Overtemperature
  0x03 = Position Limit Exceeded
  0x04 = Velocity Limit Exceeded
  0x05 = Torque Limit Exceeded
  0x06 = Encoder Error
  0x07 = Communication Timeout
  0x08 = Watchdog Timeout
  0xFF = Unknown Error

Byte 1:   Warning Flags (uint8, bitmask)
  Bit 0 = Temperature Warning (>80C)
  Bit 1 = Current Warning (>80% max)
  Bit 2 = Position Near Limit
  Bit 3 = Velocity Near Limit
  Bit 4 = Torque Near Limit
  Bit 5 = Communication Jitter High
  Bit 6 = Motor Not Calibrated
  Bit 7 = Reserved

Byte 2:   Temperature (uint8, degrees C)
Byte 3:   Motor State (uint8)
  0x00 = Idle
  0x01 = Ready
  0x02 = Running
  0x03 = Fault
  0x04 = Calibrating
  0x05 = Bootloader

Byte 4-7:  Current (float32, A)
Byte 8-11: DC Voltage (float32, V)
Byte 12-15: Reserved
```

### Heartbeat (0x40) - Broadcast @ 50Hz
```
Byte 0:   Master Status (uint8)
  Bit 0 = System Ready
  Bit 1 = Balance Active
  Bit 2 = Gait Active
  Bit 3 = AI Policy Active
  Bit 4 = Emergency Stop Active
  Bit 5 = Low Battery Warning
  Bit 6 = Overheating Warning
  Bit 7 = Communication Error

Byte 1:   Active Mode (uint8)
  0x00 = Standby
  0x01 = Stand
  0x02 = Walk
  0x03 = Teleop
  0x04 = AI Control
  0x05 = Calibration
  0x06 = Bootloader
  0xFF = Emergency

Byte 2-3:  Battery Voltage (uint16, 0.01V per LSB)
Byte 4-5:  Battery Current (int16, 0.01A per LSB)
Byte 6:   Battery SOC (uint8, %)
Byte 7:   System Temperature (uint8, degrees C)
Byte 8-11: Uptime (uint32, seconds)
Byte 12-15: Reserved
```

### Emergency Stop (0x50) - Broadcast
```
Byte 0:   Stop Source (uint8)
  0x01 = Hardware E-Stop Button
  0x02 = Software Command
  0x03 = Watchdog Timeout
  0x04 = Overcurrent Detected
  0x05 = Fall Detected
  0x06 = Communication Loss
  0x07 = User Triggered

Byte 1-3: Error Detail (uint24)
Byte 4-7: Timestamp (uint32, ms since boot)
Byte 8-15: Reserved
```

## Timing Requirements

| Message Type | Period | Priority | Deadline |
|-------------|--------|----------|----------|
| Position/Torque Cmd | 20ms (50Hz) | High | 5ms |
| Status/Error | 20ms (50Hz) | High | 10ms |
| Heartbeat | 20ms (50Hz) | Medium | 10ms |
| Parameter Get/Set | On-demand | Low | 100ms |
| Emergency | Immediate | Critical | 1ms |

## Bus Load Calculation

- 28 joints x 2 messages/cmd+fb x 64 bytes x 50Hz = ~7.2 Mbps
- Heartbeat: 64 bytes x 50Hz = 0.025 Mbps
- Safety margin: 2x
- **Required bus speed: 4 Mbps (CAN-FD data phase)**
