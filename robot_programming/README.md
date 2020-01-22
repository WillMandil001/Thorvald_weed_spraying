# Robot Programming Assignment
## Code Summary:
This repo is paired with the LCAS/CMP9767M forked repository found at: https://github.com/WillMandil001/CMP9767M
This repository controlls a Thorvald robot in a simulated environment, the robot will move about the environment autonomously and segment weeds into a pointcloud in the /map frame_id.

## Install and environment setup:
Using ROS-kinetic and Ubuntu 16.04.
2. Update and upgrade the current environment:
```bash
$ sudo apt-get update && sudo apt-get upgrade
```
3. Install the current uol-cmp9767m ros package
```bash
$ sudo apt-get install ros-kinetic-uol-cmp9767m-base
$ source /opt/ros/kinetic/setup.bash
```
If this latter resulted in error, there's a conflicting package installed, try sudo apt-get purge "*gazebo*" first, and then install again.

5. Clone the two required git repositories into your catkin workspace
```bash
$ git clone https://github.com/WillMandil001/CMP9767M
$ git clone https://github.com/WillMandil001/robot_programming.git
```
6. Install all ros dependencies:
```bash
$ rosdep install -- from-paths . -i -y
$ sudo apt-get install ros-kinetic-topological-navigation
$ sudo apt-get install ros-kinetic-image-geometry
$ sudo apt-get install ros-kinetic-topological-navigation
$ sudo apt-get install ros-kinetic-robot-pose-publisher
$ sudo apt-get install ros-kinetic-topological-utils
```
7. In you home directory create a directory which is used by the topological-navigation package.
```bash
$ mkdir mongodb
```

### To run this simulation:
1. Launch the thorvald and its relevant localisation and movement nodes:
```bash
$ roslaunch robot_programming thorvald_startup.launch
```
2. You need to initialise the topo_node_map (only the first time):
```bash
$ rosrun topological_utils load_yaml_map.py $(rospack find uol_cmp9767m_tutorial)/maps/farm_trial_map.yaml
```
3. Launch the weed segmentation nodes and the controller:
```bash
$ roslaunch robot_programming thorvald_controller.launch
```
