# ECE 346 - Intelligent Robotic Systems
**This repo hosts lab materials for *ECE 346: Intelligent Robotic Systems* at Princeton University.**

![image info](asset/Figures/robot.jpg)

<!-- To keep your forked repo updated, please fetch upstream every time we release a new lab assignment. If you are not familiar with fetch, please check out this [tutorial](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/syncing-a-fork). -->

# Getting Started
> **Note:** If you are following this repository outside of ECE 346 or are refreshing your ECE 346 laptop, please follow these [instructions](Host_Setup/robotstack.md) to set up a working ROS2 Humble environment using [RoboStack](https://robostack.github.io/) for Linux/MacOS. 

## Connect to Wi-Fi
First, you will need to connect your ECE 346 laptop to a Wi-Fi network. To connect to eduroam, open your terminal and run

```
python3 ~/Downloads/eduroam-linux-Princeton_University-Princeton_eduroam.py
```
If you don't see this file, you can temporarily connect to puvisitor using a non-Princeton email to [download it](https://cat.eduroam.org). Click 'Yes' and enter one group member's username, i.e., netid@princeton.edu and corresponding password. Then navigate to your Wi-Fi networks by clicking the top right of your screen, selecting the Wi-Fi logo followed by 'Select Network', 'eduroam', 'Connect'.

## Set up GitHub on your laptop
Next, you will connect your laptop to one group member's GitHub account using an SSH key. In your terminal, run
```
# Install packages to use GitHub and copy/paste
sudo apt install git xclip
```
```
# Replace with your GitHub email address
ssh-keygen -t ed25519 -C "your_email@example.com"
```
Press enter three times to skip requiring a password for each push/pull. Then run,
```
# Start the ssh-agent and add your private key to it
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

Open Google Chrome and log into [GitHub](http://github.com) using the same email from previous steps. In the upper-right corner of any page on GitHub, click your profile photo, then click 'Settings'. In the "Access" section of the sidebar, click  'SSH and GPG keys'. Click 'New SSH key'. In your terminal, run this command to copy your SSH key

```
# Copy public ssh key to your clipboard:
cat ~/.ssh/id_ed25519.pub | xclip -selection clipboard
```

Now in your browser, enter 'ece346-XX' for 'Title', where XX is your group number. For 'Key', simply paste the SSH key that you just copied.

Finally, complete your GitHub configuration in your terminal
```
# Replace with your GitHub email address and full name or a fun alias ;). Note this will appear on GitHub
git config –global user.email “your_email@example.com”
git config –global user.name “Your Name”
```

## Fork this repository

Set up GitHub 
[Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) on your computer if you haven't done this before.

Before cloning this repo, you will also need to setup your Github SSH key. Refer to [Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [Adding a new SSH key to your Github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) to generate and setup SSH key for your Github.

Once SSH key setup is done, create a clone of this repo locally. **Important**: `--recurse-submodules` option is neccessary to get all submodules!
```
git clone --recurse-submodules https://github.com/SafeRoboticsLab/ECE346.git 
```
## Set up your machine
One crucial component of ECE346 is ROS. Even though most of the computation will be handled on board our robots, it's still very useful to set up ROS on your computer for development, testing, and visualization. ROS used to only be available for Linux (at least painlessly). However, thanks to recent developments on [RoboStack](https://robostack.github.io/) it can now run on Windows and Mac too. Here, we provide detailed [instructions](Host_Setup/robotstack.md) and a script to help you set up ROS on your favorite operating system.

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
