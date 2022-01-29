# Lab 0 
In this lab, we will write a basic ROS package that controls the robot with a keyboard. In addition, we will also implement another ROS node that subscribes to the robot Pose from SLAM, and plots the robot trajectory.  

Please refer to the [lab0.pdf](lab0.pdf) for details. 

# Get Started
## How to build
Even if you have not filled in any code for this lab, you can still build the packages and check what functionalities we have provided for you.

- Open a terminal under to the ```ROS_ws``` and type.

    ```bash
    catkin_make 
    ```
- After the system build successfully, you will need to read and execute setups for your build by 
    ```bash
    source devel/setup.bash
    ```
## How to run your package
We have prepared [launchfiles](ROS_ws/src/lab0/launch) for this lab. Checkout this [tutorial](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch#Using_roslaunch) for more information. Under the same terminal that you build this package, use the command
```bash
roslaunch ece346_lab0 FILENAME.launch
```
For example, if you just want to launch the node for keyboard control that we have provided, you can 
```
roslaunch ece346_lab0 keyboard_only.launch
```
If everything is executed correctly, you will see the following ![keyboard](/asset/Figures/keyboard_node.png)

## Hint for checkpoint 4
We will work on the ```keyboard_listener``` node to accomplish the following tasks in this checkpoint.
1. The node will subscribe to ```geometric_msg/Twist``` messages sent through ```/cmd_vel``` topic
2. In the subscriber's callback function, we extract the ```linear.x``` as the throttle input and ```angular.z``` as the steering input
3. Construct a 

## Hint for checkpoint 5



