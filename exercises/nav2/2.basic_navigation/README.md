
## Navigation

1. Launch the simulation environment with Kobuki:
```bash
ros2 launch easynav_playground_kobuki playground_kobuki.launch.py
```

2. In another terminal, launch the navigation stack with the generated map:
```bash
ros2 launch nav2_playground navigation_launch.py map:=<path-to-generated-yaml>
```

For example:
```bash
ros2 launch nav2_playground navigation_launch.py map:=/home/user/my_map.yaml
```

3. In RViz, use the "2D Pose Estimate" tool to set the robot's initial pose.

<p align="center">
  <img src="../../../images/2d_pose_estimate.png" alt="2D Pose Estimate" width="400"/>
  <img src="../../../images/click_to_locate_robot.png" alt="Click on the correct location" width="400"/>
</p>

4. Use the "Nav2 Goal" tool to send navigation goals to the robot.

<p align="center">
  <img src="../../../images/nav2_goal.png" alt="Nav2 Goal" width="400"/>
  <img src="../../../images/click_on_goal.png" alt="Click on the goal" width="400"/>
</p>