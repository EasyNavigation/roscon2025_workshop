#!/bin/bash

# File: easynavigation_setup.sh
# Description: Automated installation script for EasyNavigation robotics stack in ROS 2 Jazzy.
#              This script sets up a complete development environment with all necessary dependencies.
# Authors:
#   - Jose Miguel Guerrero (josemiguel.guerrero@urjc.es)
#   - Francisco Miguel Moreno (franciscom.moreno@urjc.es)
# Date: 08/10/25
# Institution: Universidad Rey Juan Carlos

# Set the ROS 2 workspace directory where EasyNavigation will be installed
export ws_dir=~/workshop_ws

# Update system packages to ensure we have the latest versions and security patches
echo "Updating system packages..."
sudo apt update
sudo apt full-upgrade -y

# Source ROS 2 environment
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update

# Clone the main EasyNavigation repositories
echo "Cloning EasyNavigation repositories..."
git clone -b jazzy https://github.com/EasyNavigation/EasyNavigation.git $ws_dir/src/easynav/EasyNavigation
git clone -b jazzy https://github.com/EasyNavigation/NavMap.git $ws_dir/src/easynav/NavMap
git clone -b jazzy https://github.com/EasyNavigation/easynav_plugins.git $ws_dir/src/easynav/easynav_plugins
# Clone yaets dependency
git clone -b jazzy-devel https://github.com/fmrico/yaets.git $ws_dir/src/easynav/yaets
# --- Extra repositories used in the examples ---
git clone -b rolling https://github.com/EasyNavigation/easynav_playground_kobuki.git $ws_dir/src/easynav/easynav_playground_kobuki
git clone -b main https://github.com/EasyNavigation/easynav_indoor_testcase.git $ws_dir/src/easynav/easynav_indoor_testcase
git clone -b rolling https://github.com/EasyNavigation/easynav_playground_summit.git $ws_dir/src/easynav/easynav_playground_summit
git clone -b jazzy --recursive https://github.com/EasyNavigation/easynav_lidarslam_ros2.git $ws_dir/src/easynav/easynav_lidarslam_ros2
git clone -b jazzy https://github.com/EasyNavigation/easynav_gridmap_stack.git $ws_dir/src/easynav/easynav_gridmap_stack

# Install third-party dependencies using vcs tool
cd $ws_dir/src
vcs import . < $ws_dir/src/easynav/easynav_playground_kobuki/thirdparty.repos
vcs import . < $ws_dir/src/easynav/easynav_playground_summit/thirdparty.repos

# Navigate to the ROS 2 workspace directory
cd $ws_dir

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
echo "source $ws_dir/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "====================================================="
echo "              EASYNAVIGATION INSTALLED               "
echo "====================================================="
