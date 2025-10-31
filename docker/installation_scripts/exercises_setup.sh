#!/bin/bash

# File: exercises_setup.sh
# Description: Automated installation script for exercises 
# Authors:
#   - Jose Miguel Guerrero (josemiguel.guerrero@urjc.es)
#   - Esther Aguado (esther.aguado@urjc.es) 
# Date: 22/10/25
# Institution: Universidad Rey Juan Carlos


# Exercises
# Set the ROS 2 workspace directory where EasyNavigation will be installed
export ws_dir=~/workshop_ws

# Set the branch you want to use
BRANCH="main"

# Repository URL
REPO_URL="https://github.com/EasyNavigation/roscon2025_workshop.git"

# Destination folder (change this to your preferred path)
TMP="/tmp/workshop"
DESTINATION="$ws_dir/src"
if [ -d "$TMP" ]; then
    echo "Removing existing $TMP"
    rm -rf "$TMP"
fi

# Folder to download
FOLDER="exercises"

# Clone the repository without checking out files
echo "Cloning repository..."DESTINATION
git clone --no-checkout --depth 1 -b "$BRANCH" "$REPO_URL" "$TMP"

# Navigate to the repository directory
cd "$TMP" || exit 1

# Enable sparse-checkout
git sparse-checkout init --cone

# Specify the folder to download
git sparse-checkout set "$FOLDER"

# Checkout the main branch
git checkout $BRANCH

# Move from TMP to DESTINATION
if [ -d "$DESTINATION/$FOLDER" ]; then
    echo "Removing existing $DESTINATION/$FOLDER"
    rm -rf "$DESTINATION/$FOLDER"
fi

mv "$TMP/$FOLDER" $DESTINATION

echo "Exercises folder downloaded to $DESTINATION/$FOLDER"

echo "Building exercises packages..."
cd $ws_dir
colcon build --symlink-install

echo "====================================================="
echo "                  EXERCISES INSTALLED                "
echo "====================================================="