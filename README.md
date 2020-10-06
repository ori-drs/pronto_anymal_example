# pronto_anymal_example
Demo binary and related libraries to run the Pronto state estimator on the ANYmal B robot (simplified version).

## Structure
This repository  includes three catkin packages:
- `anymal_robcogen` kinematics/dynamics libraries generated with RobCoGen from the public URDF of the ANYmal B robot. 
  For more info on how to generate the code for your quadruped robot, check out the [quadruped_robcogen]() package. 
- `pronto_anymal_commons` implementation (using the package above) of the kinematics/dynamics interfaces to perform the estimation. The interfaces do not depend on RobCoGen, you can implement them with any other library like RBDL or Pinocchio. Here we use RobCoGen though.
- `pronto_anymal` Pronto ANYmal executable

## Dependencies
To run the demo program you need the catkin packages contained in the following repositories:
- [`pronto`](https://github.com/ori-drs/pronto) the Pronto estimator libraries
- [`common_utils`](https://github.com/ori-drs/common_utils) dependency of the above package
- [`fovis`](https://github.com/ori-drs/fovis_ros.git) the FOVIS visual odometry algorithm
- [`fovis_ros`](https://github.com/ori-drs/fovis_ros/tree/pronto-fovis)  ROS wrapper for FOVIS
- [`anymal_b_simple_description`](https://github.com/mcamurri/anymal_b_simple_description) public description of the ANYmal B (augmented with standard sensor components)
- [`realsense`](https://github.com/ori-drs/realsense/tree/development-fixes) description file of the realsense device

You can automatically clone them with the `clone_deps.sh` script under the `scripts` folder (see below).

## How to Install
Clone this repo, its dependencies, and compile the `pronto_anymal` package:
```
roscd
cd ../src
git clone https://github.com/ori-drs/pronto_anymal_example
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
pronto_anymal_example/pronto_anymal/scripts/clone_deps.bash
catkin build pronto_anymal
roscd
source setup.bash
```

## How to Run 
To run the Pronto estimator on your PC, download the example dataset with the provided script (if you want to download it manually, it is [here](https://drive.google.com/open?id=1a_BA7yyj4XdUcCXrxpz5o1PdCi0fJn5K)) and run it paused:
```
$(rospack find pronto_anymal)/scripts/download_sample_rosbag.sh
rosbag play --clock --pause fsc_minimal_joint_states_short.bag
```
in a separate terminal, launch the demo launcher:

```
roslaunch pronto_anymal pronto_anymal.launch
```
Finally, press the space bar on the first terminal to start the rosbag replay.

This will run Pronto in inertial-kinematics + visual odometry mode and visualize it in RViz:

![pronto_anymal_rviz](./pronto_anymal/doc/rviz_output.png  "Pronto ANYmal RViz")

## Configuration
You can change the parameters of the Pronto estimator in the `state_estimator.yaml` configuration file  under the `pronto_anymal/config` folder. 
## Custom Visual and LIDAR Odometry Modules
To integrate your own Visual and LIDAR Odometry packages, just make them produce the same message type that the `fovis` and `scan_matcher` modules are expect to produce.

Even if the VO module is called `fovis`, Pronto does not depend on FOVIS and can accept any relative measurement of type `pronto_msgs/VisualOdometryUpdate`. In the same way, the LIDAR odometry module has to produce `pronto_msgs/LidarOdometryUpdate` messages. 

The message definitions are available [here](https://github.com/ori-drs/pronto/tree/pronto-anymal-example/pronto_msgs/msg).
