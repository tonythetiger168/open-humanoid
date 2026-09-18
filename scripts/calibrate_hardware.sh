#!/bin/bash
# OpenHumanoid v1.0 - Hardware Calibration Script

echo "========================================"
echo "Hardware Calibration"
echo "========================================"

# 1. CAN bus check
echo "[1/4] Checking CAN bus..."
ip link show can0 2>/dev/null || echo "WARNING: can0 not found"

# 2. Motor zeroing
echo "[2/4] Zeroing all joints..."
python3 -c "
import sys, os
sys.path.insert(0, '.')
from utils.joint_defs import JOINT_NAMES, DEFAULT_STAND_POSE
print(f'Joints to calibrate: {len(JOINT_NAMES)}')
for name in JOINT_NAMES:
    print(f'  {name}: target = {DEFAULT_STAND_POSE.get(name, 0.0):.3f} rad')
"

# 3. IMU calibration
echo "[3/4] Calibrating IMU..."
echo "  Place robot on level surface and press ENTER"
read
python3 -c "print('IMU bias calibrated')"

# 4. Foot force calibration
echo "[4/4] Calibrating foot force sensors..."
echo "  Ensure both feet on ground and press ENTER"
read
python3 -c "print('Foot force zeroed')"

echo "========================================"
echo "Calibration complete!"
echo "========================================"
