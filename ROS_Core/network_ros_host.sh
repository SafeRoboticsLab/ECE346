#!/bin/bash

# use "source network_ros_host.sh HOST_IP"
export ROS_HOSTNAME=$1
export ROS_MASTER_URI=http://$1:11311
export ROS_IP=$1