#!/bin/bash

DEST=${CMAKE_PREFIX_PATH%%:*}/../src

# Pronto core libraries
git clone https://github.com/ori-drs/pronto.git ${DEST}/pronto

# Pronto dependencies
git clone https://github.com/ori-drs/common_utils.git ${DEST}/pronto_common_utils

git clone https://github.com/ori-drs/kinematic_utils.git ${DEST}/pronto_biped_kinematics

# FOVIS library, catkinized
git clone https://github.com/ori-drs/fovis.git ${DEST}/fovis

# ROS wrapper for FOVIS
git clone -b pronto-fovis https://github.com/ori-drs/fovis_ros.git ${DEST}/fovis_ros

# Tools to visualize path in RViz
git clone https://github.com/ori-drs/hector_slam.git ${DEST}/hector_slam

# RealSense Description
git clone https://github.com/IntelRealSense/realsense-ros ${DEST}/realsense

