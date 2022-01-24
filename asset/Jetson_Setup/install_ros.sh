#!/bin/bash
# install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full -y
apt search ros-melodic
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
pip3 install rospkg catkin_pkg

sudo rosdep init
rosdep update

# remove image transport plug in
# sudo apt-get remove -y ros-melodic-theora-image-transport ros-melodic-compressed-image-transport ros-melodic-compressed-depth-image-transport

# http://wiki.ros.org/UsingPython3/BuildUsingPython3