# Install ROBOSTACK ROS-Noetic 
[RoboStack](https://robostack.github.io/index.html) is a bundling of the Robot Operating System (ROS) by Open Robotics for Linux, Mac and Windows using the Conda package manager. Traditionaly, ROS is only available to Linux and does not work well with Conda.

## Mac OS and Linux
If you are using Mac OS or Linux, we provide you a simple script to automate the process. Simply open a terminal, then run

```
chmod +x ros_conda_install_unix.sh
./ros_conda_install_unix.sh
```
If you already have conda (anaconda/miniconda/miniforge, etc) installed, it will install [**miniforge**](https://github.com/conda-forge/miniforge) in parallel with your current conda, and then create a new python3.9 environment with ROS Noetic installed.

If you do not have conda installed, the script will first install [**miniforge**](https://github.com/conda-forge/miniforge), and then create a new ROS Noetic environment. 

We will create an alias for the new environment called ```ros_env```. You can activate the environment by running ```conda activate ros_base```.
## Windows 10+
You have to use Windows Subsystem Linux. Please refer to this [tutorial](Windows/windows_robostack.md) for setup.

# Test it out
Open a new terminal, and run activate your ROS environment by ```ros_env```.

Then run ```roscore``` to start the ROS master. If everything works, you will seee
![](asset/ros_core_output.png)

# Common Issues
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




# Reference
```
@article{FischerRAM2021,
    title={A RoboStack Tutorial: Using the Robot Operating System Alongside the Conda and Jupyter Data Science Ecosystems},
    author={Tobias Fischer and Wolf Vollprecht and Silvio Traversaro and Sean Yen and Carlos Herrero and Michael Milford},
    journal={IEEE Robotics and Automation Magazine},
    year={2021},
    doi={10.1109/MRA.2021.3128367},
}
```
