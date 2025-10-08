#!/bin/bash

# Function to check if CUDA version is >= 12.9
check_cuda_version() {
    if ! command -v nvidia-smi &> /dev/null; then
        echo "nvidia-smi not found. Assuming no CUDA support."
        return 1
    fi

    version=$(nvidia-smi | grep -oP 'CUDA Version: \K[0-9]+\.[0-9]+')
    
    if [[ -z "$version" ]]; then
        echo "Could not detect CUDA version."
        return 1
    fi

    major=$(echo $version | cut -d. -f1)
    minor=$(echo $version | cut -d. -f2)

    if (( major > 12 )) || (( major == 12 && minor >= 9 )); then
        return 0
    else
        echo "CUDA version $version is lower than 12.9. Not using GPU support."
        return 1
    fi
}

if lspci | grep -i nvidia && check_cuda_version; then
    echo "Running Docker container with NVIDIA GPU support."
    docker run -p 6080:80 --privileged --env="DISPLAY=$DISPLAY" --env="NVIDIA_DRIVER_CAPABILITIES=all" --env="NVIDIA_VISIBLE_DEVICES=all" --runtime=nvidia --gpus all --name easynav -d jmguerreroh/easynav:ubuntu24
else
    echo "Running Docker container without NVIDIA GPU support."
    docker run -p 6080:80 --privileged --name easynav -d jmguerreroh/easynav:ubuntu24
fi