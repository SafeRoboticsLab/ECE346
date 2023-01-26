# Frequently Asked Questions
## 1. *"Unable to contact my own server at [http://xxxx]"*
You will typically see this error on Mac OS. This is because the default ROS master is not set to localhost. To fix this, you need to run following lines to in your terminal. 

```
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=localhost
```
We also provide a script to automate this process. Simply run ```source local_ros.sh``` in your terminal.

**Important**: You need to run these lines ***every time*** you open a new terminal. **Or**, you can export them to your shell profile by running 

```
proile= # choose from ~/.bash_profile, ~/.zshrc, and ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> $profile
echo "export ROS_MASTER_URI=http://localhost:11311" >> $profile
echo "export ROS_IP=localhost" >> $profile
```

## 2. *OMG! My ROS is totally broken*
Do not panic, let us just try to delete the ROS environment and reinstall it. 

First, go back to the base environment
```
conda activate base
```
Then, delete the ROS environment
```
conda env remove -n ros_base
```
Finally, reinstall ROS followin this [tutorial](Host_Setup/RoboStack/robotstack.md)
