# ECE 346 - Intelligent Robotic Systems
**This repo hosts lab materials for *ECE 346: Intelligent Robotic Systems* at Princeton University.**

![image info](asset/Figures/robot.jpg)

<!-- To keep your forked repo updated, please fetch upstream every time we release a new lab assignment. If you are not familiar with fetch, please check out this [tutorial](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/syncing-a-fork). -->

# Getting Started

> **ECE 346 students:** please fork this repository and work within your own forked repo (see instructions below). 

## Windows Users:
You might want to setup the Windows Linux Subsystem first by following this [guide](Host_Setup/RoboStack/Windows/windows_robostack.md).

## First Things First!
[Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) on your computer if you haven't done this before.

Before cloning this repo, you will also need to setup your Github SSH key. Refer to [Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [Adding a new SSH key to your Github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) to generate and setup SSH key for your Github.

Once SSH key setup is done, create a clone of this repo locally. **Important**: `--recurse-submodules` option is neccessary to get all submodules!
```
git clone --recurse-submodules https://github.com/SafeRoboticsLab/ECE346.git 
```
## Set up your machine
One crucial component of ECE346 is ROS. Even though most of the computation will be handled on board our robots, it's still very useful to set up ROS on your computer for development, testing, and visualization. ROS used to only be available for Linux (at least painlessly). However, thanks to recent developments on [RoboStack](https://robostack.github.io/) it can now run on Windows and Mac too. Here, we provide detailed [instructions](Host_Setup/RoboStack/robotstack.md) and a script to help you set up ROS on your favorite operating system.

## Create your own fork
You can simply click the **fork** button on the top of the page. However, we encourage each group to create a _private_ fork to host your code, and make a local clone on your group's robot, by following these [instructions](Docs/private_fork.md). Please include your group number in the name of your repo.

## Still not comfortable with ROS?
We have a ROS cheat sheet for you! Check it out [here](Docs/ROScheatsheet.pdf).

## Frequently Asked Questions
Please check out our [FAQ](FAQ/readme.md) page for common questions.

# Lab Assignments
## [Pre-Lab 0: Introduction to ROS](Docs/Intro_ROS.pdf)
## [Pre-Lab 0: Introduction to Mini-Truck](Docs/Intro_Mini_Truck.pdf)
## [Lab 0: Introduction to ROS](ROS_Core/src/Labs/Lab0)
## [Lab 1: ILQR Trajectory Planning](ROS_Core/src/Labs/Lab1)
## [Lab 2: Collision Avodiance and Navigation in Dynamic Environment](ROS_Core/src/Labs/Lab2)
## [Lab 3: MDP and POMDP](ROS_Core/src/Labs/Lab3)
## [Lab 4: Imitation Learning](ROS_Core/src/Labs/Lab4)
