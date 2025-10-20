# Nav2 Playground - Kobuki

## Description

This repository is designed for the first part of the navigation workshop using Nav2. It provides practical examples that guide through the complete autonomous navigation pipeline:

1. **Map generation**: Creating maps using SLAM
2. **Navigation**: Configuration and tuning of Nav2 parameters
3. **Practical application**: Implementation of a patrolling system

**Workshop slides:** [https://docs.google.com/presentation/d/1EolsYLnZf61nYQTu813rE4UZZ7SsVHEgQi_jr4Pu7Vo/edit?usp=sharing](https://docs.google.com/presentation/d/1EolsYLnZf61nYQTu813rE4UZZ7SsVHEgQi_jr4Pu7Vo/edit?usp=sharing)

## Installation

### Prerequisites
- ROS 2 (Jazzy/Kilted)
- Colcon

### Installation Steps

1. Clone the repository into your workspace:
```bash
cd <your-workspace>/src/
git clone https://github.com/EasyNavigation/easynav_playground_kobuki.git
```

2. Import dependencies:
```bash
cd <your-workspace>/src/
vcs import . < easynav_playground_kobuki/thirdparty.repos
```

3. Install dependencies with rosdep:
```bash
cd <your-workspace>
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the workspace:
```bash
colcon build --symlink-install
```

5. Source the workspace:
```bash
source install/setup.bash
```

---

## Workshop Exercises

This workshop includes several hands-on exercises to master Nav2:

| Exercise | Description | Location |
|----------|-------------|----------|
| **SLAM** | Learn how to generate maps using slam_toolbox | [exercises/nav2/slam.md](../exercises/nav2/slam.md) |
| **Navigation** | Configure and test autonomous navigation with Nav2 | [exercises/nav2/navigating.md](../exercises/nav2/navigating.md) |
| **Parameter Tuning** | Modify and optimize Nav2 parameters using rqt_reconfigure | [exercises/nav2/params.md](../exercises/nav2/params.md) |
| **Patrolling** | Implement a complete patrolling system | [exercises/nav2/patrolling_exercise.md](../exercises/nav2/patrolling_exercise.md) |

---
