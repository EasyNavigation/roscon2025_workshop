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
