# panthera_ht_ros

This meta package contains the ROS 2 packages used to control the Panthera HT robot arm and gripper.

This is a simplified version of the existing repository providing basic functionalities to Panthera-HT robotic arm.
For a full version visit the [HighTorque-Robotics' original repository](https://github.com/HighTorque-Robotics/Panthera-HT_ROS2).

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

Make sure that you are using the newest packages:

```bash
sudo apt update && sudo apt upgrade
```

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

To visualize MoveIt in RViz, you can build the package on your local machine and run:

```bash
source install/setup.bash
ros2 launch panthera_moveit rviz.launch.py
```

### Direct SDK-based arm control

```bash
source install/setup.bash
ros2 launch panthera_arm_control arm_control.launch.py
```

You can optionally launch joystick teleop together with arm control:

```bash
source install/setup.bash
ros2 launch panthera_arm_control arm_control.launch.py launch_joy_teleop:=true
```

By default the teleop node subscribes to the `/joy` topic, so you need to publish data to it
(for example with [joy_linux node](https://index.ros.org/p/joy_linux/)).

## ROS interfaces by node

### `panthera_arm_control/arm_control_node`

Publishers:
- `joint_states` (`sensor_msgs/msg/JointState`)
- `arm_status` (`panthera_interfaces/msg/ArmStatus`)
- `end_pose_euler` (`panthera_interfaces/msg/EndPoseEuler`)

Subscribers:
- `arm_joint_cmd` (`std_msgs/msg/Float64MultiArray`)
- `gripper_cmd` (`std_msgs/msg/Bool`)
- `enable_flag` (`std_msgs/msg/Bool`)
- `pos_cmd` (`panthera_interfaces/msg/PosCmd`)

Services (servers):
- `move_to_joint` (`panthera_interfaces/srv/MoveToJoint`)
- `move_to_pose` (`panthera_interfaces/srv/MoveToPose`)
- `gripper_control` (`panthera_interfaces/srv/GripperControl`)
- `enable_srv` (`panthera_interfaces/srv/Enable`)
- `go_zero_srv` (`panthera_interfaces/srv/GoZero`)
- `stop_srv` (`std_srvs/srv/Trigger`)
- `reset_srv` (`std_srvs/srv/Trigger`)
- `gripper_srv` (`panthera_interfaces/srv/GripperSrv`)

### `panthera_arm_control/arm_joy_teleop_node`

Publishers:
- `pos_cmd` (`panthera_interfaces/msg/PosCmd`)

Subscribers:
- `joy` (`sensor_msgs/msg/Joy`)
- `end_pose_euler` (`panthera_interfaces/msg/EndPoseEuler`)

### Nodes launched by `panthera_moveit/hardware_moveit.launch.py`

`hardware_moveit.launch.py` starts `move_group` and includes `hardware.launch.py`, which starts `ros2_control_node` (`/controller_manager`) plus controller spawners.

#### `moveit_ros_move_group/move_group`

Subscribers:
- `/joint_states` (`sensor_msgs/msg/JointState`)

Services:
- `/compute_ik` (`moveit_msgs/srv/GetPositionIK`)
- `/compute_fk` (`moveit_msgs/srv/GetPositionFK`)
- `/plan_kinematic_path` (`moveit_msgs/srv/GetMotionPlan`)
- `/get_planning_scene` (`moveit_msgs/srv/GetPlanningScene`)
- `/apply_planning_scene` (`moveit_msgs/srv/ApplyPlanningScene`)
- `/get_cartesian_path` (`moveit_msgs/srv/GetCartesianPath`)

Note:
- MoveIt also publishes/subscribes additional planning-scene and visualization topics, and uses controller actions configured in `panthera_moveit/config/moveit_controllers.yaml`.

#### `controller_manager/ros2_control_node` (`/controller_manager`)

Services:
- `/controller_manager/list_controllers`
- `/controller_manager/load_controller`
- `/controller_manager/configure_controller`
- `/controller_manager/switch_controller`
- `/controller_manager/unload_controller`
- `/controller_manager/list_hardware_interfaces`

Note:
- Loaded controllers expose their own interfaces:
	- `joint_state_broadcaster` publishes joint states.
	- `arm_controller` and `gripper_controller` expose FollowJointTrajectory action interfaces.
