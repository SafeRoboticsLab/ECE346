# Install ROBOSTACK ROS-Noetic 
[RoboStack](https://robostack.github.io/index.html) is a bundling of the Robot Operating System (ROS) by Open Robotics for Linux, Mac and Windows using the Conda package manager. Traditionaly, ROS is only available to Linux and does not work well with Conda.

# Update: 2023.2.5
RoboStack has released a new version of ROS Noetic. It uses a more recent version of Boost, which allows use to use newer version of Jax, PyTorch, and HPP-FCL. This means your algorithms will run (potentially) faster and more stable. If you want to try out, please follow the instructions below.
```
./ros_conda_install_unix_new.sh
```
It will create a new environment called ```ros_base2```. You can activate it by ```conda activate ros_base2```. Your old environment ```ros_base``` will still be there. Both will work well for this class.

## Mac OS and Linux
If you are using Mac OS or Linux, we provide you a simple script to automate the process. Simply open a terminal, then run

```
chmod +x ros_conda_install_unix.sh
./ros_conda_install_unix.sh
```
If you already have conda (anaconda/miniconda/miniforge, etc) installed, it will install [**mamba**](https://mamba.readthedocs.io/en/latest/installation.html) in your base environments, and then create a new python3.9 environment with ROS Noetic installed. If your original conda is very old, you might experience siginificantly slow installation and even errors. It might be a good idea to backup your old conda environments, uninstall your current conda, and fresh install mambaforge instead.

If you do not have conda installed, the script will first install [**mambaforge**](https://github.com/conda-forge/miniforge#mambaforge), and then create a new ROS Noetic environment. 

## Windows 10+
You have to use Windows Subsystem Linux. Please refer to this [tutorial](Windows/windows_robostack.md) for setup.

# Test it out
Open a new terminal, and run activate your ROS environment by ```conda activate ros_base```.

Then run ```roscore``` to start the ROS master. If everything works, you will seee
![](asset/ros_core_output.png)

# Common Issues
Check it out [here](/FAQ/readme.md)!



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
