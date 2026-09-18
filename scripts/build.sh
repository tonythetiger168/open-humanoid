#!/bin/bash
# OpenHumanoid v1.0 - Build & Deploy Script

set -e

echo "========================================"
echo "OpenHumanoid v1.0 - Build Script"
echo "========================================"

# Check dependencies
echo "[1/5] Checking dependencies..."
python3 -c "import numpy, matplotlib" 2>/dev/null || { echo "ERROR: numpy/matplotlib not installed"; exit 1; }

# Install Python packages
echo "[2/5] Installing Python packages..."
pip3 install -r requirements.txt

# Build URDF (validate)
echo "[3/5] Validating URDF..."
python3 -c "
import xml.etree.ElementTree as ET
ET.parse('design/open_humanoid_v1.urdf')
print('URDF valid')
"

# Run unit tests
echo "[4/5] Running unit tests..."
python3 tests/integration_tests.py

# Build Docker image (optional)
echo "[5/5] Building Docker image..."
docker build -t open_humanoid:v1.0 .

echo "========================================"
echo "Build complete!"
echo "Run: python3 control/main_controller.py"
echo "========================================"
