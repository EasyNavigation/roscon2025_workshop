#!/bin/bash

# File: easynavigation_setup.sh
# Description: Automated installation script for EasyNavigation robotics stack in ROS 2 Jazzy.
#              This script sets up a complete development environment with all necessary dependencies.
# Author: Jose Miguel Guerrero (josemiguel.guerrero@urjc.es)
# Date: 08/10/25
# Institution: Universidad Rey Juan Carlos

# Set the ROS 2 workspace directory where EasyNavigation will be installed
export pkg_dir=~/ros2_ws

# Update system packages to ensure we have the latest versions and security patches
echo "Updating system packages..."
sudo apt update
sudo apt full-upgrade -y

# Clone the EasyNavigation repositories (Jazzy branch for ROS 2 Jazzy compatibility)
echo "Cloning EasyNavigation repository..."
git clone -b jazzy https://github.com/EasyNavigation/EasyNavigation.git $pkg_dir/src/EasyNavigation
# --- include other necessary repositories if needed ---

echo "Copying example packages..."
cp -r ../examples/* $pkg_dir/src/

# Navigate to the ROS 2 workspace directory
cd $pkg_dir 

# Install all ROS 2 dependencies for the packages in the workspace
# This ensures all required packages are available before compilation
echo "Installing ROS 2 dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build all packages in the workspace using colcon
# --symlink-install creates symbolic links for faster development iteration
echo "Building EasyNavigation packages..."
colcon build --symlink-install

# Automatically source the workspace setup in future terminal sessions
# This makes EasyNavigation commands available in new terminals
echo "Configuring environment..."
echo "source $pkg_dir/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "====================================================="
echo "              EASYNAVIGATION INSTALLED               "
echo "====================================================="
