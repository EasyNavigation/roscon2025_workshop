#!/bin/bash

# Check if the system has an NVIDIA graphics card
if lspci | grep -i nvidia; then
    echo "NVIDIA graphics card detected. Executing installation."

    # Download and configure the GPG key
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
      sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
      sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

    # Update packages and install nvidia-container-toolkit
    sudo apt-get update
    sudo apt-get install -y nvidia-container-toolkit

    # Configure nvidia-ctk runtime for Docker
    sudo nvidia-ctk runtime configure --runtime=docker

    # Restart the Docker service
    sudo systemctl restart docker

    echo "NVIDIA Container Toolkit installed successfully."
else
    echo "No NVIDIA graphics card detected."
fi
