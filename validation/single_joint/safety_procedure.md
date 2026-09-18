# Single Joint Test Safety Procedure

## Pre-Test Safety Check (Mandatory)

### 1. Personal Protective Equipment
- Safety glasses (mandatory)
- Closed-toe shoes (mandatory)
- No loose clothing, jewelry, or long hair untied
- Insulated gloves (for electrical work)

### 2. Area Setup
- Clear 2-meter radius around test fixture
- Post "TEST IN PROGRESS - DO NOT ENTER" signs
- Ensure emergency stop is within arm's reach
- Verify fire extinguisher is charged and accessible
- Have first aid kit ready

### 3. Equipment Verification
```bash
# Run pre-test safety script
python3 validation/single_joint/safety_check.py
```

Checks:
- E-stop circuit continuity
- Limit switch functionality
- Motor driver fault status
- Encoder signal presence
- CAN bus communication
- Temperature sensor reading

### 4. Power-Up Sequence
1. E-stop engaged
2. Connect 24V power (limited current: 2A)
3. Verify no smoke/heat/sparks
4. Release E-stop
5. Check driver LED status (should be green)
6. Run encoder check
7. Gradually increase current limit to rated value

### 5. During Test
- NEVER leave running test unattended
- Monitor temperature every 5 minutes
- If temperature > 70C: PAUSE test, allow cooling
- If unusual noise/vibration: STOP immediately
- Keep hand on E-stop during first motion

### 6. Emergency Procedures

**Motor Runaway**
1. Hit E-stop immediately
2. Disconnect power
3. Check encoder wiring
4. Check CAN bus termination

**Overheating**
1. Stop motion
2. Maintain cooling fan
3. Do not touch motor (may be >100C)
4. Wait 30 minutes before handling

**Electrical Fault**
1. Hit E-stop
2. Disconnect power
3. Use multimeter to check for shorts
4. Do not re-energize until fault cleared

### 7. Post-Test
- Engage E-stop
- Disconnect power
- Wait 5 minutes for capacitor discharge
- Verify motor is cool (<40C) before handling
- Store counterweights securely
- Document any anomalies
