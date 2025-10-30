
# Patrolling


## Patrolling Exercise - Implementation Guide

The patrolling exercise is the culmination of this workshop. You will implement a state machine that makes the robot autonomously patrol through a set of waypoints.

###  Overview

The patrolling system uses a state machine to cycle through waypoints and perform actions at each location. The template files are already provided, and your task is to complete the missing implementation.

###  Your Task

You need to implement the `DO_SOMETHING_AT_WAYPOINT` state in the patrolling node template. This state is triggered when the robot successfully reaches a waypoint.

### Template Files

You can choose either the **C++** or **Python** implementation (or both!):

- **C++**: [exercises/nav2/nav2_playground/src/nav2_playground/patrolling_node_template.cpp](src/nav2_playground/patrolling_node_template.cpp)
- **Python**: [exercises/nav2/nav2_playground/scripts/patrolling_node_template.py](https://github.com/EasyNavigation/roscon2025_workshop/blob/main/exercises/nav2/nav2_playground/scripts/patrolling_node_template.py)


### What You Need to Do

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

**Example Solution 1: Print a Message**

**C++:**
```cpp
case PatrolState::DO_SOMETHING_AT_WAYPOINT:
  {
    RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu!", current_waypoint_index_);
    
    // Move to next waypoint
    current_waypoint_index_++;
    state_ = PatrolState::SENDING_GOAL;
    break;
  }
```

**Python:**
```python
elif self.state == PatrolState.DO_SOMETHING_AT_WAYPOINT:
    self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}!')
    
    # Move to next waypoint
    self.current_waypoint_index += 1
    self.state = PatrolState.SENDING_GOAL
```

**Example Solution 2: Wait at Waypoint**

**C++:**
```cpp
case PatrolState::DO_SOMETHING_AT_WAYPOINT:
  {
    if (!waiting_) {
      RCLCPP_INFO(this->get_logger(), "Waiting at waypoint %zu for 3 seconds...", current_waypoint_index_);
      wait_start_time_ = this->now();
      waiting_ = true;
    }
    
    // Check if 3 seconds have passed
    if ((this->now() - wait_start_time_).seconds() >= 3.0) {
      RCLCPP_INFO(this->get_logger(), "Wait complete! Moving to next waypoint.");
      waiting_ = false;
      current_waypoint_index_++;
      state_ = PatrolState::SENDING_GOAL;
    }
    break;
  }
```

**Python:**
```python
elif self.state == PatrolState.DO_SOMETHING_AT_WAYPOINT:
    if not self.waiting:
        self.get_logger().info(f'Waiting at waypoint {self.current_waypoint_index} for 3 seconds...')
        self.wait_start_time = self.get_clock().now()
        self.waiting = True
    
    # Check if 3 seconds have passed
    if (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9 >= 3.0:
        self.get_logger().info('Wait complete! Moving to next waypoint.')
        self.waiting = False
        self.current_waypoint_index += 1
        self.state = PatrolState.SENDING_GOAL
```

> **Note for Solution 2:** You'll need to add member variables to track the waiting state:
> - C++: Add `bool waiting_ = false;` and `rclcpp::Time wait_start_time_;` to your class
> - Python: Add `self.waiting = False` and `self.wait_start_time = None` in `__init__`

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
ros2 launch easynav_playground_kobuki playground_kobuki.launch.py
```

2. In another terminal, launch the navigation stack with your map:
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




