# easynav_workshop_testcase

This package contains the workshop testcase used in the second part of the navigation workshop. It demonstrates EasyNavigation ([EasyNav](https://easynavigation.github.io/)), an open-source navigation framework for ROS 2, and includes example configurations and launch files for the Kobuki playground.

**Workshop slides:** [TBD]

## Installation

### Prerequisites
- ROS 2 (Jazzy or Kilted)
- colcon

### Installation steps

1. Clone EasyNavigation and the workshop repositories into your workspace:
```bash
cd <your-workspace>/src/
git clone -b jazzy https://github.com/EasyNavigation/EasyNavigation.git
git clone -b jazzy https://github.com/EasyNavigation/easynav_plugins.git
git clone -b jazzy https://github.com/EasyNavigation/NavMap.git
git clone https://github.com/EasyNavigation/easynav_playground_kobuki.git
git clone https://github.com/EasyNavigation/roscon2025_workshop.git
```

2. Import third-party repos required by the playground:
```bash
cd <your-workspace>/src/
vcs import . < easynav_playground_kobuki/thirdparty.repos
```

3. Install system dependencies using rosdep:
```bash
cd <your-workspace>
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
colcon build --symlink-install
```

5. Source the workspace setup file:
```bash
source install/setup.bash
```

## Execution

1. Launch the Kobuki simulation environment:
```bash
ros2 launch easynav_playground_kobuki playground_kobuki.launch.py
```

2. In a separate terminal start the EasyNav main node with your chosen config file (this selects the plugins to load):
```bash
ros2 run easynav_system system_main --ros-args --params-file <desired_config_path.params.yaml>
```

3. Start RViz to visualize the robot and the costmap (use simulation time):
```bash
ros2 run rviz2 rviz2 -d src/easynav_workshop/easynav_workshop_testcase/rviz/costmap.rviz --ros-args -p use_sim_time:=true
```

### Exercises
All exercises are available in the `exercises/` folder. Each exercise includes a suggested solution in the same directory.
