# Lab 2

## Set Jetson to Max Performance Mode
Your program will run much faster on Jetson by running the full performance mode. This can be easily achieved by 
```bash
sudo /usr/sbin/nvpmodel -m 8
```
More details regarding differnet power modes can be found [here](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0E0YO0HA).

## Setup the Environment
Your iLQR code requires Python3 to run, however the ROS Melodic is built on Python2. To overcome this issue, we will use *virtualenv* to create a virtual Python3 environment. We have provided the script to create such environment. Simply run
```bash
chmod +x setup_env.sh
source setup_env.sh
```

## Download the Map
The Zed camera will use a pre-build map to localize the truck on the racetrack. We also provide the script to download the map file and save it to `~/Documents/outerloop_map.area`

```bash
chmod +x download_map.sh
./download_map.sh
```

## Update your submodule
We have made changes to your zed_wrapper. Use the following command to make sure that you have the correct setup with Zed camera:
```
git submodule update --init --recursive
```

## Build the ROS Package
Navigate to folder `ROS_ws` within your repo and type ```catkin_make```.

Once everything is built, source the environment again with
```
source ROS_ws/devel/setup.bash
```

## Run the planner and tracker
You can see how iLQR online planning and tracking works on the real robot truc by running the following command
```
roslaunch traj_planning_ros ilqr_planning.launch
```

This example will show you the robot truck navigates around the Minicity. You will see from our real-life setup that there is a green truck put right onto the track at the entrance of the room. This is similar to your static obstacle in your Colab. If everything works well, you will see your robot truck does maneuver around this green truck to avoid collision.