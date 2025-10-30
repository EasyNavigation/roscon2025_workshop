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
# Repository URL
REPO_URL="https://github.com/EasyNavigation/roscon2025_workshop.git"

# Destination folder (change this to your preferred path)
TMP="/tmp/workshop"
DESTINATION="$ws_dir/src"

# Folder to download
FOLDER="exercises"

# Clone the repository without checking out files
echo "Cloning repository..."
git clone --no-checkout --depth 1 "$REPO_URL" "$TMP"

# Navigate to the repository directory
cd "$TMP" || exit 1

# Enable sparse-checkout
git sparse-checkout init --cone

# Specify the folder to download
git sparse-checkout set "$FOLDER"

# Checkout the main branch
git checkout main

# Move from TMP to DESTINATION
mv "$TMP/$FOLDER" "$DESTINATION"

echo "Exercises folder downloaded to $DESTINATION/$FOLDER"

echo "====================================================="
echo "                  EXERCISES INSTALLED                "
echo "====================================================="