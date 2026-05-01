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



