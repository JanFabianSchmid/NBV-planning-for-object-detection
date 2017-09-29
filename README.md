# Next best view planning for object detection

This is a ROS-based project that contains software to use a Pioneer robot to find objects in an environment.

## Installation
Some software packages have to be installed beforehand.
### Current dependencies:
- Ubuntu 14.04
- OpenCV 2.4.9
- PCL 1.7.2
- ros-indigo-ros-controllers (to install: sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers)

To use the simulated robot:
- https://github.com/avin2/SensorKinect (for instructions see http://learn.turtlebot.com/2015/02/01/5/)

To use a real robot:
- A mobile platform and a RGB-D camera, we used a Kinect v2 on top of a P2DX robot
- Rosaria
- A driver for the RGB-D camera, we used iai_kinect2

### Compiling
Navigate into catkin_ws and execute catkin_make (ROS has to be sourced in the terminal)

## Usage
- in frontier_exploration/launch/global_map.launch one of 5 NBV methods can be selected:
a) information_gain
b) information_gain_with_candidates
c) random
d) frontier
e) frontier_plus

 ### For the simulation:
- in catkin_ws: source devel/setup.bash
Two options
- roslaunch evaluation experiment.launch
- the parameters in evaluation/experiment_setup will be used for the simulation
- OR
- roslaunch p3dx_description everything.launch
- In Rviz the exploration boundaries have to be published using publish point

### For the real robot:
- git checkout real_robot
- in catkin_ws: source devel/setup.bash
- roslaunch p3dx_description everything.launch
- In Rviz the exploration boundaries have to be published using publish point

## Used and modified ROS packages
Our project contains code from multiple ROS packages.
### Used packages
- P3DX URDF and simulation setup: https://github.com/SD-Robot-Vision/PioneerModel and 
- P3DX navigation setup: https://github.com/larics/p3dx_config
- gmapping: http://wiki.ros.org/gmapping
- rosaria: http://wiki.ros.org/ROSARIA
- pointcloud_to_laserscan: http://wiki.ros.org/pointcloud_to_laserscan
- octomap: http://wiki.ros.org/octomap
- octomap_msgs: http://wiki.ros.org/octomap_msgs

Our project also contains some object models from: http://www.ycbbenchmarks.com/

### Modified packages
We included large parts of the code, altered and added parts of following packages:
- frontier_exploration: http://wiki.ros.org/frontier_exploration
- octomap_mapping: http://wiki.ros.org/octomap_mapping

Developed by Phil Bradfield and Jan Fabian Schmid
