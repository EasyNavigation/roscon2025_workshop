#!/bin/bash

# File: exercises_setup.sh
# Description: Automated installation script for exercises 
# Authors:
#   - Jose Miguel Guerrero (josemiguel.guerrero@urjc.es)
# Date: 22/10/25
# Institution: Universidad Rey Juan Carlos


# Repository URL
REPO_URL="https://github.com/EasyNavigation/roscon2025_workshop.git"

# Destination folder in home directory
DESTINATION="$HOME/roscon2025_workshop"
WORKSHOP_WS="$DESTINATION/workshop_ws"

# Folder to download
FOLDER="exercises"


# Remove destination if it already exists
if [ -d "$DESTINATION" ]; then
    echo "Removing existing directory $DESTINATION"
    rm -rf "$DESTINATION"
fi


# Clone the repository without checking out files
echo "Cloning repository..."
git clone --no-checkout --depth 1 "$REPO_URL" "$DESTINATION"

# Navigate to the repository directory
cd "$DESTINATION" || exit 1

# Enable sparse-checkout
git sparse-checkout init --cone

# Specify the folder to download
git sparse-checkout set "$FOLDER"

# Checkout the main branch
git checkout main

echo "Exercises folder downloaded to $DESTINATION/$FOLDER"

# Set up workshop_ws
mkdir -p "$WORKSHOP_WS/src"

# Copy easynav_playground into src
PLAYGROUND_SRC="$DESTINATION/$FOLDER/easynav/easynav_playground"
PLAYGROUND_DEST="$WORKSHOP_WS/src"

if [ -d "$PLAYGROUND_SRC" ]; then
    echo "Copying easynav_playground to workspace"
    cp -r "$PLAYGROUND_SRC" "$PLAYGROUND_DEST"
    echo "Copied to $PLAYGROUND_DEST"
else
    echo "Error: Source folder not found at $PLAYGROUND_SRC"
    exit 1
fi

# Copy nav2_playground into src
NAV2_SRC="$DESTINATION/$FOLDER/nav2/nav2_playground"
NAV2_DEST="$WORKSHOP_WS/src"

if [ -d "$NAV2_SRC" ]; then
    echo "Copying nav2_playground to workspace"
    cp -r "$NAV2_SRC" "$NAV2_DEST"
    echo "Copied to $NAV2_DEST"
else
    echo "Error: Source folder not found at $NAV2_SRC"
    exit 1
fi

cd "$WORKSHOP_WS" || exit 1
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
echo "source $WORKSHOP_WS/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "====================================================="
echo "              EXERCISES INSTALLED               "
echo "====================================================="
