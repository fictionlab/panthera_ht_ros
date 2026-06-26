# panthera_ht_ros

This meta package contains the ROS 2 packages used to control the Panthera HT robot arm and gripper.

## What each package does

- `hightorque_robot` contains the low-level SDK wrapper.
- `panthera_description` provides the robot description used by ROS tooling and visualization.
- `panthera_interfaces` defines the custom messages and services used by the arm control node.
- `panthera_arm_control` provides direct SDK-based control without MoveIt.
- `panthera_moveit` provides the hardware interface plugin and MoveIt configuration.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage

### MoveIt

Launch the hardware stack and MoveIt from `panthera_moveit`:

```bash
source install/setup.bash
ros2 launch panthera_moveit hardware_moveit.launch.py
```

### Direct SDK-based arm control

```bash
source install/setup.bash
ros2 launch panthera_arm_control arm_control.launch.py
```
