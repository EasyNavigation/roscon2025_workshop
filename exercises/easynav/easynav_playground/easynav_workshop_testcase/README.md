# easynav_workshop_testcase

This package contains the workshop testcase used in the second part of the navigation workshop. It demonstrates EasyNavigation ([EasyNav](https://easynavigation.github.io/)), an open-source navigation framework for ROS 2, and includes example configurations and launch files for the Kobuki playground.

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
ros2 run rviz2 rviz2 -d ~/roscon2025_workshop/workshop_ws/src/easynav_playground/easynav_workshop_testcase/rviz/costmap.rviz 
```

### Exercises
All exercises are available in the `exercises/` folder. Each exercise includes a suggested solution in the same directory.
