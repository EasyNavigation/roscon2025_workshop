# ROSCon 2025 Navigation Workshop
This guide helps developers and researchers quickly set up and test the EasyNavigation robotics platform using Docker and ROS 2. It provides a hassle-free way to try EasyNavigation without complex local installations, supporting both Linux and non-Linux systems through containerization. You can follow the slides at :
- [Nav2](https://github.com/EasyNavigation/roscon2025_workshop/blob/main/slides/nav2.pdf)
- [Easynav](https://github.com/EasyNavigation/roscon2025_workshop/blob/main/slides/easynav.pdf)

---

## Who is this for?
 - **Developers**: Quickly test and explore EasyNavigation features in a pre-configured environment.
 - **Researchers**: Evaluate the platform for your robotics projects without setup overhead.
 - **Students**: Learn robotics navigation concepts with ready-to-use tools and examples.
 - **Educators**: Use the containerized environment for teaching navigation algorithms.

---
# Surveys
  - Before workshop: https://forms.office.com/e/zHrLbQnEcW
  - After workshop: https://forms.office.com/e/zEHKFjjR2L
---
# Installation
- If you have an NVIDIA card in your system and it is configured with the proper drivers, you can execute the following command to switch between the integrated graphics card and the NVIDIA GPU:
```sh 
sudo prime-select nvidia
``` 

- After running prime-select, you will need to restart your system for the changes to take effect:
```sh 
sudo reboot
``` 
Now, choose your preferred setup method:
- **Option 1**: Local installation (Linux only, Ubuntu 24.04) - For advanced users who want direct system integration
- **Option 2**: Docker environment (Recommended) - For easy testing and cross-platform compatibility
---


## Option 1: Local installation (Linux only, Ubuntu 24.04)
If you're using Ubuntu 24.04 and prefer to install dependencies without Docker, you can install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html), create a directory for the EasyNavigation environment and clone the repository and run the provided setup script to configure the simulator locally.
```sh
git clone https://github.com/EasyNavigation/roscon2025_workshop.git
./roscon2025_workshop/docker/installation_scripts/workshop_all.sh
```

---
## Option 2: Docker environment

### A. For Windows/macOS users or Ubuntu with Docker Desktop:

#### 1. Install Docker Desktop
Install Docker Desktop from: [Docker Desktop](https://www.docker.com/products/docker-desktop/)

#### 2. Pull the Docker image
Pull the image by searching for `jmguerreroh/easynav:ubuntu24` in Docker Desktop's search interface or by running the following command in the Terminal:
```bash
docker pull jmguerreroh/easynav:ubuntu24
```

#### 3. Run the container
When running the image in Docker Desktop, set **6080** as the host port in the Docker settings. 

#### 4. Access the Virtual Desktop
Once it's running, open your browser and go to: http://localhost:6080/

You should see a full development desktop environment.

---

### B. For Ubuntu users without Docker Desktop:

#### 1. Install Docker Engine
Follow the official Docker installation guide for Ubuntu: [Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

#### 2. Clone the repository
Create a directory for the EasyNavigation environment and clone the repository:
```sh
mkdir ~/easynav_ws && cd ~/easynav_ws
git clone https://github.com/EasyNavigation/roscon2025_workshop.git
```

#### 3. (Optional) Install the NVIDIA Container Toolkit
To enable GPU Acceleration (Linux + NVIDIA) usage inside Docker:
- Install the [NVIDIA drivers](https://ubuntu.com/server/docs/nvidia-drivers-installation).
- Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). Execute the script:

```sh
cd ~/easynav_ws/roscon2025_workshop/docker/
./nvidia_install.sh
```

#### 4. Download or build the Docker image
There are two ways to get the Docker image:
- **Option 4.1: Download Prebuilt Image from Docker Hub (RECOMMENDED)**:
```sh
docker pull jmguerreroh/easynav:ubuntu24
```

- **Option 4.2: Build image locally**
Navigate to the Docker directory and build the Docker image:
```sh
cd ~/easynav_ws/roscon2025_workshop/docker/
sudo docker buildx build --platform=linux/amd64  -t jmguerreroh/easynav:ubuntu24 -f Dockerfile .
```

#### 5. Run Docker image
Run the Docker image using the provided script:
```sh
cd ~/easynav_ws/roscon2025_workshop/docker/
./run_docker.sh
```

**Note:** If you run this command without having the image, it will attempt to download it from Docker Hub as in step 4.1.

#### 6. Access the Virtual Desktop
Open your browser and go to: http://localhost:6080/

You should see a full development desktop environment:

![Environment](images/environment.png)

---

## Managing the Docker Container

### Stopping the container
If the container is currently running and you need to stop it, follow these steps:

1. First, log out of the Docker container environment in your browser:
   ![logout](images/logout.png)

2. Then, stop the Docker container using the following command:
```sh
docker stop easynav
```

**Note:** Your work is preserved unless you explicitly remove the container.

### Restarting the container
If the container is stopped and you want to start it again:
```sh
docker start easynav
```

Then access it again at: http://localhost:6080/

---
## Customization and Advanced Usage

### Custom Environment Setup
If you need to customize the environment with additional packages or configurations:
1. Create your own installation script and place it inside the `docker/installation_scripts/` folder.
2. Follow steps 4.2 and 5 from Option 2B to build a custom Docker image.
3. Execute your script inside the Docker container. Once running, open a terminal and execute:

```sh
source /installation_scripts/your_script.sh
```

### Using Pre-built Custom Images
If you have access to a custom EasyNavigation image:
1. Pull the specific image version you need:
```sh
docker pull your-custom-image:tag
```
2. Modify the `run_docker.sh` script to use your custom image name.
3. Follow the same steps as above to run the container.
4. Any custom scripts can be downloaded directly in the browser environment and executed in the terminal.

# Exercises index — Nav2 & EasyNav

This index lists the workshop exercises for the Nav2 (first part) and EasyNav (second part) tracks. Each entry includes a short summary and the README path inside the repo.

## Nav2 exercises

- nav2_playground — overview and entrypoint
  - Path: `exercises/nav2/nav2_playground/README.md`

- 1. Map generation (slam_toolbox)
  - Path: `exercises/nav2/1.map_generation/README.md`
  - Summary: Launch the Kobuki playground, run `slam_toolbox` to generate a map, use RViz to monitor mapping, and save the resulting `.pgm/.yaml` files.

- 2. Basic navigation
  - Path: `exercises/nav2/2.basic_navigation/README.md`
  - Summary: Launch Nav2 with a generated map, use RViz to set initial pose and send goals (2D Pose Estimate, Nav2 Goal), and observe the robot executing navigation tasks.

- 3. Parameters configuration
  - Path: `exercises/nav2/3.params_configuration/README.md`
  - Summary: Use `rqt` / `rqt_reconfigure` to change Nav2 parameters (e.g., inflation) interactively and observe costmap changes in RViz.

- 4. Patrolling (Nav2)
  - Path: `exercises/nav2/4.patrolling/README.md`
  - Summary: Final Nav2 exercise; implement a patrolling state machine using the provided C++ or Python templates (examples and testing instructions included).


# Exercises index — Nav2 & EasyNav

This index lists the workshop exercises for the Nav2 (first part) and EasyNav (second part) tracks. Each entry links directly to the exercise README.

## Nav2 exercises

- nav2_playground — [exercises/nav2/nav2_playground/README.md](exercises/nav2/nav2_playground/README.md)
- 1. Map generation (slam_toolbox) — [exercises/nav2/1.map_generation/README.md](exercises/nav2/1.map_generation/README.md)
- 2. Basic navigation — [exercises/nav2/2.basic_navigation/README.md](exercises/nav2/2.basic_navigation/README.md)
- 3. Parameters configuration — [exercises/nav2/3.params_configuration/README.md](exercises/nav2/3.params_configuration/README.md)
- 4. Patrolling (Nav2) — [exercises/nav2/4.patrolling/README.md](exercises/nav2/4.patrolling/README.md)

## EasyNav exercises

- easynav_workshop_testcase — [exercises/easynav/easynav_playground/easynav_workshop_testcase/README.md](exercises/easynav/easynav_playground/easynav_workshop_testcase/README.md)
- 1. Basic configuration (EasyNav) — [exercises/easynav/1.basic_config/README.md](exercises/easynav/1.basic_config/README.md)
- 2. Add custom plugin — [exercises/easynav/2.custom_plugin/README.md](exercises/easynav/2.custom_plugin/README.md)
- 3. Gridmap patrolling (EasyNav) — [exercises/easynav/3.gridmap_patrolling/README.md](exercises/easynav/3.gridmap_patrolling/README.md)
- 4. Using NavMap representation — [exercises/easynav/4.using_navmap/README.md](exercises/easynav/4.using_navmap/README.md)
