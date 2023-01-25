# ECE 346 - Intelligent Robotic Systems
**This repo hosts lab materials for *ECE 346: Intelligent Robotic Systems* at Princeton University.**

![image info](asset/Figures/robot.jpg)

<!-- To keep your forked repo updated, please fetch upstream every time we release a new lab assignment. If you are not familiar with fetch, please check out this [tutorial](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/syncing-a-fork). -->

# Getting Started

> **ECE 346 students:** please fork this repository and work within your own forked repo (see instructions below). 

## First Things First!
Create a clone of this repo locally. **Important**: `--recurse-submodules` option is very important!
```
git clone --recurse-submodules https://github.com/SafeRoboticsLab/ECE346.git 
```
## Setup your machine
One crucial component of ECE346 is ROS. Even though most of computation will be handled on our robots, it is still beneficial to set up ROS on your computer for development, testing, and visualization. Previously, ROS was only available to the Ubuntu system. However, recent developments on [RobotStack](https://robostack.github.io/) make it possible for Windows and Mac computers. Here, we provide detail [instructions](Host_Setup/RoboStack/robotstack.md) and a script to help you set it up. 

## Connect to the robot
The robot can be used as a standalone Ubuntu computer by plugging in a keyboard, a mouse, and a monitor through an HDMI or DP cable. 

Alternatively, you can remotely access the robot’s terminal through SSH, provided that both the robot and the computer are under the same network. If you are connected to the lab’s **ECE346** wifi, we provide detailed [instructions](asset/ssh.md) on how to SSH into the robot using one of the lab workstations or your own laptop.

We highly recommand you to use [Remote Development Functionality](https://code.visualstudio.com/docs/remote/ssh) from VS Code. 

## Create your own fork
You can simply clik **fork** button on the top of the page. However, we encourage each group to create a _private_ fork to host your code, and make a clone on your group's robot, by following these [instructions](asset/private_fork.md). Please include your group number in the name of your repo.

# [Lab 0 - Introduction](ROS_ws/src/lab0)
Lab 0 gives an introduction to ROS and the mobile robot platform used in this class.

# [Lab 2 - Forward Reachable Set](ROS_ws/src/lab2)
Lab 2 allows students to plan trajectory that follows given path and avoids obstacles.
