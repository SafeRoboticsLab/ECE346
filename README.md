# ECE 346 - Intelligent Robotic Systems
**This repo hosts lab materials for *ECE 346: Intelligent Robotic Systems* at Princeton University.**

![image info](asset/Figures/robot.jpg)

<!-- To keep your forked repo updated, please fetch upstream every time we release a new lab assignment. If you are not familiar with fetch, please check out this [tutorial](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/syncing-a-fork). -->

# Getting Started

> **ECE 346 students:** please fork this repository and work within your own forked repo (see instructions below). 

## First Things First!
[Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) if you have not done it before. Then create a clone of this repo locally. **Important**: `--recurse-submodules` option is neccessary to get all submodules!
```
git clone --recurse-submodules https://github.com/SafeRoboticsLab/ECE346.git 
```
## Setup your machine
One crucial component of ECE346 is ROS. Even though most of computation will be handled on our robots, it is still beneficial to set up ROS on your computer for development, testing, and visualization. Previously, ROS was only available to the Ubuntu system. However, recent developments on [RobotStack](https://robostack.github.io/) make it possible for Windows and Mac computers. Here, we provide detail [instructions](Host_Setup/RoboStack/robotstack.md) and a script to help you set it up. 

## Create your own fork
You can simply clik **fork** button on the top of the page. However, we encourage each group to create a _private_ fork to host your code, and make a clone on your group's robot, by following these [instructions](asset/private_fork.md). Please include your group number in the name of your repo.

## Still not comfortable with ROS?
We have a ROS cheat sheet for you! Check it out [here](ROScheatsheet.pdf).
## Frequently Asked Questions
Please check out our [FAQ](FAQ/readme.md) page for common questions.
