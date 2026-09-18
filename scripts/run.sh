#!/bin/bash
# OpenHumanoid v1.0 - Run Script

MODE=${1:-stand}
DURATION=${2:-10}

echo "========================================"
echo "OpenHumanoid v1.0 - Run"
echo "Mode: $MODE, Duration: ${DURATION}s"
echo "========================================"

python3 control/main_controller.py --mode $MODE --duration $DURATION
