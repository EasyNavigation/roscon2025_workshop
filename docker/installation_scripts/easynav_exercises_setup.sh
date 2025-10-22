#!/bin/bash

# File: nav2_setup.sh
# Description: Automated installation script for Navigation2 stack in ROS 2 Jazzy.
#              
# Authors:
#   - Jose Miguel Guerrero (josemiguel.guerrero@urjc.es)
#   - Esther Aguado (esther.aguado@urjc.es)
# Date: 22/10/25
# Institution: Universidad Rey Juan Carlos

# Update system packages to ensure we have the latest versions and security patches
echo "Updating system packages..."
sudo apt update
sudo apt full-upgrade -y

# Source ROS 2 environment
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update

# Install Navigation2 and its dependencies
echo "Installing Navigation2 and SLAM Toolbox..."
sudo apt install -y \
    ros-$ROS_DISTRO-nav2-* \
    ros-$ROS_DISTRO-slam-toolbox \


echo "====================================================="
echo "              NAVIGATION2 INSTALLED                  "
echo "====================================================="
