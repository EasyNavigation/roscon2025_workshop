#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Execute all setup scripts in sequence
echo "Running EasyNavigation setup..."
bash "$SCRIPT_DIR/easynavigation_setup.sh"

echo "Running Nav2 setup..."
bash "$SCRIPT_DIR/nav2_setup.sh"

echo "Running exercises setup..."
bash "$SCRIPT_DIR/exercises_setup.sh"

echo "====================================================="
echo "           ALL INSTALLATIONS COMPLETED               "
echo "====================================================="