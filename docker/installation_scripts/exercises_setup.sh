#!/bin/bash

# File: exercises_setup.sh
# Description: Automated installation script for exercises 
# Authors:
#   - Jose Miguel Guerrero (josemiguel.guerrero@urjc.es)
# Date: 22/10/25
# Institution: Universidad Rey Juan Carlos

#!/bin/bash

# Repository URL
REPO_URL="https://github.com/EasyNavigation/roscon2025_workshop.git"

# Destination folder in home directory
DESTINATION="$HOME/roscon2025_workshop"
WORKSHOP_WS="$HOME/workshop_ws"

# Folder to download
FOLDER="exercises"

# Remove destination if it already exists
if [ -d "$DESTINATION" ]; then
    echo "Removing existing directory $DESTINATION"
    rm -rf "$DESTINATION"
fi

# Clone the repository without checking out files
echo "Cloning repository..."
git clone --no-checkout --depth 1  --branch docker_test "$REPO_URL" "$DESTINATION"

# Navigate to the repository directory
cd "$DESTINATION" || exit 1

# Enable sparse-checkout
git sparse-checkout init --cone

# Specify the folder to download
git sparse-checkout set "$FOLDER"

# Checkout the main branch
git checkout docker_test

echo "Exercises folder downloaded to $DESTINATION/$FOLDER"

# Set up workshop_ws
mkdir -p "$WORKSHOP_WS/src"

# Copy easynav_playground into src
PLAYGROUND_SRC="$DESTINATION/$FOLDER/easynav/easynav_playground"
PLAYGROUND_DEST="$WORKSHOP_WS/src/easynav_playground"

if [ -d "$PLAYGROUND_SRC" ]; then
    echo "Copying easynav_playground to workspace"
    cp -r "$PLAYGROUND_SRC" "$PLAYGROUND_DEST"
    echo "Copied to $PLAYGROUND_DEST"
else
    echo "Error: Source folder not found at $PLAYGROUND_SRC"
    exit 1
fi

# ROS dependency installation
pip install "numpy<2.0" --break-system-packages
echo "Installing ROS dependencies"
cd "$WORKSHOP_WS_DIR"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "Building workspace"
colcon build --symlink-install

echo "Configuring environment"
echo "source $WORKSHOP_WS_DIR/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "====================================================="
echo "              WORKSHOP PKGS INSTALLED                "
echo "====================================================="
