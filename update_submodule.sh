#!/bin/bash

CUR_DIR=$(pwd)

git submodule update --init --recursive

# SLAM message
cd $CUR_DIR/ROS_Core/src/Utility/AprilTagSLAM_ROS
git checkout msg_only
git pull

# Interface
cd $CUR_DIR/ROS_Core/src/Utility/Interface
git checkout main
git pull

# # Planning
# cd $CUR_DIR/ROS_Core/src/Utility/Labs/Lab1
# git checkout main
# git pull


# Message
cd $CUR_DIR/ROS_Core/src/Utility/Custom_Msgs
git checkout main
git pull

# Routing
cd $CUR_DIR/ROS_Core/src/Utility/Routing
git checkout pylanelet
git pull

cd $CUR_DIR