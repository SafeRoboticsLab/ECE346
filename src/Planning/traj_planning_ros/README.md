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

## Build the ROS Package


