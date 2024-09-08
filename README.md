# Manipulator Project

This repository contains the necessary files to control a mobile robot equipped with a manipulator. The manipulator can perform various tasks using ROS 2 and integrates the MoveIt motion planning framework along with the Behavior Tree framework.
***

## Installation Instructions
Follow the steps below to set up the project and run it:

### 0. Install necessary library
``` bash
sudo apt update
sudo apt install ros-humble-moveit
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros2-control
```

### 1. Create the Workspace and Clone the Repository
``` bash
mkdir -p ~/manipulator_ws && cd ~/manipulator_ws
git clone git@github.com:NoobiesDoobies/manipulator.git
mv manipulator src
```

### 2. Initialize and Update Submodules
```bash
git submodule update --init --recursive
```
### 3. Install BehaviorTree.ROS2
```bash
cd ~
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2

cd BehaviorTree.ROS2/btcpp_ros2_interfaces
mkdir -p build && cd build

cmake .. && make && sudo make install
```
Compile the behaviortree_ros2 library
```bash
cd ../behaviortree_ros2/
mkdir -p build && cd build

cmake .. && make && sudo make install
```

### 4. Build the Workspace
Navigate back to your workspace and build the necessary packages:

``` bash
cd ~/manipulator_ws

colcon build --packages-select manipulator_msgs
source install/setup.bash

colcon build --symlink-install
source install/setup.bash
```
### 5. Launch the Simulation
Once the workspace is built, launch the simulation:

``` bash
ros2 launch manipulator_bringup simulation.launch.py
```

### 6. Launch the Behavior Tree
To run the behavior tree controlling the manipulator, execute:
```bash
ros2 launch manipulator_bt behaviour_tree.launch.py
```
Additional Notes:
- Ensure you have ROS 2 Humble installed.
- The colcon build --symlink-install command allows easy editing and testing of code without recompiling the entire workspace.
- The BehaviorTree framework is required for high-level decision-making in this project.
