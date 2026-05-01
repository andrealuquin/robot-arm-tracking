# 3D Robot Arm Tracking With Jacobian-Based Control Methods

This ROS 2 project compares two Jacobian-based control methods for a 3DOF robot arm tracking target positions in 3D space:

- Jacobian pseudo-inverse control
- Damped Least Squares (DLS) control

The controller uses forward kinematics, tracking error, the Jacobian matrix, joint velocity updates, and Yoshikawa manipulability to evaluate how the robot moves toward each target.

---


## Requirements

This project was developed for ROS 2 Humble on Ubuntu 22.04.

Install ROS 2 Humble first if it is not already installed:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

Install common Python/ROS build tools:

```bash
sudo apt install python3-colcon-common-extensions python3-pip
pip3 install numpy
```

---

## Workspace Setup

Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the repository:

```bash
git clone https://github.com/andrealuquin/robot-arm-tracking.git
```

Go back to the workspace root:

```bash
cd ~/ros2_ws
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Build the package:

```bash
colcon build
```

Source the workspace after building:

```bash
source install/setup.bash
```

---

## How to Run

The project can be run with two terminals: one for the controller and one for the target publisher.

### Terminal 1: Run the Controller

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_tracking_project controller_node
```

The controller node computes the robot's current end-effector position, tracking error, Jacobian, joint velocity updates, and performance metrics.

### Terminal 2: Run the Target Publisher

Open a second terminal and run:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_tracking_project target_publisher
```

The target publisher sends the target positions to the controller on the `/target_position` topic.

---

## Optional: Run Both Nodes With the Launch File

Instead of using two terminals, both nodes can be started with the launch file:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_tracking_project project_launch.launch.py
```

---
## Expected Output

The controller should print step-by-step tracking information and a summary for each target case, including values such as:

- final tracking error
- number of steps/iterations
- maximum joint velocity magnitude
- Yoshikawa manipulability
- final end-effector position

The main comparison is between pseudo-inverse and DLS:

- Pseudo-inverse usually reaches the target faster.
- DLS usually keeps joint velocities lower and produces smoother motion.

---

## Useful ROS 2 Checks

To verify that the target publisher is sending targets:

```bash
ros2 topic echo /target_position
```

To list active ROS 2 topics:

```bash
ros2 topic list
```

To confirm that both nodes are running:

```bash
ros2 node list
```

Expected nodes include:

```text
/controller_node
/target_publisher
```

---
## Tested Target Cases

The target positions are 3D coordinates in meters relative to the robot base frame:

| Case | Target Position `[x, y, z]` |
|---|---|
| Normal case | `[0.6, 0.2, 0.8]` |
| Shifted case | `[0.5, 0.1, 0.75]` |
| Stretched case | `[1.15, 0.0, 0.5]` |




## Project Parameters

Default values used by the controller:

```text
Initial joint configuration: q = [0.2, 0.3, -0.2]
Link lengths: [0.5, 0.7, 0.5]
Time step: dt = 0.05
Tracking tolerance: 0.01
DLS damping value: 0.1
Maximum steps: 200
```

---

