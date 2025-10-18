
# Patrolling


## Patrolling Exercise - Implementation Guide

The patrolling exercise is the culmination of this workshop. You will implement a state machine that makes the robot autonomously patrol through a set of waypoints.

###  Overview

The patrolling system uses a state machine to cycle through waypoints and perform actions at each location. The template files are already provided, and your task is to complete the missing implementation.

###  Your Task

You need to implement the `DO_SOMETHING_AT_WAYPOINT` state in the patrolling node template. This state is triggered when the robot successfully reaches a waypoint.

### Template Files

You can choose either the **C++** or **Python** implementation (or both!):

- **C++**: `src/nav2_playground/patrolling_node_template.cpp`
- **Python**: `scripts/patrolling_node_template.py`

### 🔍 What You Need to Do

#### Step 1: Locate the Template Code

In **C++** (`patrolling_node_template.cpp`), find line 203:
```cpp
case PatrolState::DO_SOMETHING_AT_WAYPOINT:
  {
  // Implement what the robot should do when it reaches a waypoint.
  // Ideas:
  // After completing the task, you should:
  //   1. Increment current_waypoint_index_
  //   2. Transition to PatrolState::SENDING_GOAL
  //
  // YOUR CODE HERE

    break;
  }
```

In **Python** (`patrolling_node_template.py`), find the similar section:
```python
elif self.state == PatrolState.DO_SOMETHING_AT_WAYPOINT:
    # Implement what the robot should do when it reaches a waypoint.
    #
    # After completing the task, you should:
    #   1. Increment self.current_waypoint_index
    #   2. Transition to PatrolState.SENDING_GOAL
    # YOUR CODE HERE
    pass
```

#### Step 2: Implement the Waypoint Action

When the robot reaches a waypoint, it should do something! Here are some ideas:

- Wait for a few seconds (simulate inspection, scanning, etc.)
- Log a message indicating arrival
- Rotate 360° to scan the environment
- Take a sensor reading
- Publish a message to another node
- Call a service

#### Step 3: Build Your Implementation

After implementing your solution, build the workspace:

```bash
cd <your-workspace>
colcon build --symlink-install --packages-select nav2_playground
source install/setup.bash
```

#### Step 4: Test Your Patrolling Node

1. Launch the simulation:
```bash
ros2 launch nav2_playground playground_kobuki.launch.py
```

2. Launch the navigation stack with your map:
```bash
ros2 launch nav2_playground navigation_launch.py map:=<path-to-your-map.yaml>
```

3. In RViz, set the initial pose using "2D Pose Estimate"

4. Launch your patrolling node:

**For C++:**
```bash
ros2 run nav2_playground patrolling_node_template --ros-args --params-file src/roscon2025_workshop/nav2_playground/config/patrolling_params.yaml
```

**For Python:**
```bash
ros2 run nav2_playground patrolling_node_template.py --ros-args --params-file src/roscon2025_workshop/nav2_playground/config/patrolling_params.yaml
```

### Waypoint Configuration

Waypoints are configured in `config/patrolling_params.yaml`:

```yaml
/**:
  ros__parameters:
    frame_id: "map"
    waypoints: ["wp1", "wp2", "wp3", "wp4"]
    wp1: [0.0, 0.0, 0.0]        # [x, y, yaw]
    wp2: [2.0, 0.0, 1.57]       # Move 2m in x, face north
    wp3: [2.0, 2.0, 3.14]       # Move to (2,2), face west
    wp4: [0.0, 2.0, -1.57]      # Move to (0,2), face south
```

Modify these values to match positions in your map!



## Patrolling existing examples:


### Using the NavigateToPose action

1. Launch the simulation (if not already running):
```bash
ros2 launch nav2_playground playground_kobuki.launch.py
```

2. Launch the navigation stack:
```bash
ros2 launch nav2_playground navigation_launch.py map:=<path-to-generated-yaml>
```

3. Launch the patrolling node with NavigateToPose:

**Using Python:**
```bash
ros2 launch nav2_playground patrol_launch.py use_cpp:=false use_poses:=false
```

**Using C++:**
```bash
ros2 launch nav2_playground patrol_launch.py use_cpp:=true use_poses:=false
```

### Using the NavigateThroughPoses action

1. Launch the simulation (if not already running):
```bash
ros2 launch nav2_playground playground_kobuki.launch.py
```

2. Launch the navigation stack:
```bash
ros2 launch nav2_playground navigation_launch.py map:=<path-to-generated-yaml>
```

3. Launch the patrolling node with NavigateThroughPoses:

**Using Python:**
```bash
ros2 launch nav2_playground patrol_launch.py use_cpp:=false use_poses:=true
```

**Using C++:**
```bash
ros2 launch nav2_playground patrol_launch.py use_cpp:=true use_poses:=true
```




