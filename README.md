# pronto_anymal_example
[![CI](https://github.com/ori-drs/pronto_anymal_example/actions/workflows/industrial_ci_action.yml/badge.svg?branch=master)](https://github.com/ori-drs/pronto_anymal_example/actions/workflows/industrial_ci_action.yml)

Demo binary and related libraries to run the Pronto state estimator on the ANYmal B robot (simplified version).

## Environment
Tested on ROS Noetic and Ubuntu 20.04 Focal Fossa. Compatibility with any other operating system or ROS version is not guaranteed nor officially supported. Our build server ensures that the example can be built using Ubuntu 18.04/ROS Melodic and Ubuntu 20.04/ROS Noetic.

## Structure
This repository  includes three catkin packages:
- `anymal_b_robcogen` kinematics/dynamics libraries generated with RobCoGen from the public URDF of the ANYmal B robot. 
  For more info on how to generate the code for your quadruped robot, check out the [quadruped_robcogen](https://github.com/ori-drs/quadruped_robcogen) package. 
- `pronto_anymal_b_commons` implementation (using the package above) of the kinematics/dynamics interfaces to perform the estimation. The interfaces do not depend on RobCoGen, you can implement them with any other library like RBDL or Pinocchio. Here we use RobCoGen though.
- `pronto_anymal_b` Pronto ANYmal executable

## Dependencies
To run the demo program you need the catkin packages contained in the following repositories:
- [`fovis`](https://github.com/ori-drs/fovis.git) the FOVIS visual odometry algorithm
- [`fovis_ros`](https://github.com/ori-drs/fovis_ros/tree/pronto-fovis)  ROS wrapper for FOVIS (branch `pronto-fovis`)
- [`realsense2_description`](https://github.com/IntelRealSense/realsense-ros) description file of the realsense device

The `realsense2_description` can be installed automatically via `rosdep` or manually from APT:
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-description
```

You can automatically clone the other dependencies with the `clone_deps.sh` script under the `scripts` folder (see below). The cloned dependencies can be built in a catkin workspace 
**Note:** `colcon` as a build tool is currently not supported/does not work.

## How to Install
Clone compile the `pronto_anymal_b` package:
```
catkin build pronto_anymal_b
roscd
source setup.bash
```

## How to Run 
To run the Pronto estimator on your PC, download the example dataset with the provided script (if you want to download it manually, it is [here](https://drive.google.com/open?id=1a_BA7yyj4XdUcCXrxpz5o1PdCi0fJn5K)) and run it paused:
```
$(rospack find pronto_anymal_b)/scripts/download_sample_rosbag.sh
rosbag play --clock --pause fsc_minimal_joint_states_short.bag
```
in a separate terminal, launch the demo launcher:

```
roslaunch pronto_anymal_b demo.launch
```
Finally, press the space bar on the first terminal to start the rosbag replay.

This will run Pronto in inertial-kinematics + visual odometry mode and visualize it in RViz:

![pronto_anymal_rviz](./pronto_anymal_b/doc/rviz_output.png  "Pronto ANYmal RViz")

## Configuration
You can change the parameters of the Pronto estimator in the `state_estimator.yaml` configuration file  under the `pronto_anymal/config` folder. 
## Custom Visual and LIDAR Odometry Modules
To integrate your own Visual and LIDAR Odometry packages, just make them produce the same message type that the `fovis` and `scan_matcher` modules are expect to produce.

Even if the VO module is called `fovis`, Pronto does not depend on FOVIS and can accept any relative measurement of type `pronto_msgs/VisualOdometryUpdate`. In the same way, the LIDAR odometry module has to produce `pronto_msgs/LidarOdometryUpdate` messages. 

The message definitions are available [here](https://github.com/ori-drs/pronto/tree/master/pronto_msgs/msg).
