#!/bin/bash

# use "source network_ros_client.sh HOST_IP CLIENT_IP"
export ROS_HOSTNAME=$2
export ROS_MASTER_URI=http://$1:11311
export ROS_IP=$2