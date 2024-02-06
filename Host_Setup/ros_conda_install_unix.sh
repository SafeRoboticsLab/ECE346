#!/bin/bash

set -e # exit on error

# define color
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

cur_dir=$(pwd)

OS=$(uname -s)
ARCH=$(uname -m)

# Check if conda installed
# if test -z "$(which $CONDA_EXE)" ; then
if [ -d "$HOME/miniforge3" ]; then
  echo -e "${BLUE}Found Miniforge3 in $HOME/miniforge3${NC}"
  echo -e "Install Mamba to base environment"
  eval "$($HOME/miniforge3/bin/conda shell.bash hook)"
  conda activate base
  conda config --add channels conda-forge
  conda install -n base mamba -c conda-forge --yes
else
  echo -e "${RED} Miniforge3 is not installed, install Miniforge3 ${NC}"
  # Mac OS
  if [[ $OS == 'Darwin' ]]; then
    if [[ $ARCH == 'arm64' ]]; then
      echo -e "${GREEN} You are using Mac OS with Apple Silicon${NC}"
      curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-arm64.sh > Miniforge3-Installer.sh
    elif [[ $ARCH == 'x86_64' ]]; then  
      echo -e "${GREEN} You are using Mac OS with X86${NC}"
      curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-x86_64.sh > Miniforge3-Installer.sh
    else
      echo -e "${RED} Unrecognized arch type, Mac OS with ${ARCH}${NC}"
      return 1 2>/dev/null
      exit 1
    fi

  elif [[ $OS == "Linux" ]]; then
    # Linux
    if [[ $ARCH == 'aarch64' ]]; then
      echo -e "${GREEN}You are using Linux with aarch64${NC}"
      curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh > Miniforge3-Installer.sh
    elif [[ $ARCH == 'x86_64' ]]; then  
      echo -e "${GREEN}You are using Linux with X86${NC}"
      curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh > Miniforge3-Installer.sh
    else
      echo -e "${RED} Unrecognized arch type, Linux with ${ARCH}${NC}"
      return 1 2>/dev/null
      exit 1
    fi

  else
    echo -e "${RED} OS: ${OS} not supported ${NC}"
    exit 1
  fi

  # Install Miniforge3
  chmod +x Miniforge3-Installer.sh
  ./Miniforge3-Installer.sh -b -f

  cd ~/miniforge3/bin
  ./conda config --set auto_activate_base false
  
  # manually enable conda in the current bash session
  eval "$(./conda shell.bash hook)"
  
  cd $cur_dir
  rm Miniforge3-Installer.sh

  echo -e "${Green} Finish install Miniforge3${NC}"  
fi

# manually enable mamba
cd $(conda info --base)/etc/profile.d/
. "mamba.sh"

cd $cur_dir
chmod +x start_ros.sh
cp start_ros.sh $HOME/miniforge3

echo -e "Current Conda is running on ${GREEN}$CONDA_EXE${NC}"

conda --version
# mamba --version

# make sure we are in base 
echo -e "${BLUE}Setup RoboStack Env${NC}"
for i in $(seq ${CONDA_SHLVL}); do
    conda deactivate
done
conda activate base

mamba create -n ros_base ros-noetic-desktop python=3.9 \
             -c robostack-staging -c conda-forge \
             --no-channel-priority --override-channels --yes


echo -e "${BLUE}Finish Settting up RoboStack Env${NC}"

mamba activate ros_base

mamba install -n ros_base compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools -c conda-forge -c robostack-staging --yes
# reload environment to activate required scripts before running anything
# on Windows, please restart the Anaconda Prompt / Command Prompt!
mamba deactivate
mamba activate ros_base

# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults || true

# if you want to use rosdep, also do:
mamba install -n ros_base rosdep -c conda-forge -c robostack-staging --yes
rosdep init  # note: do not use sudo!
rosdep update

echo -e "${BLUE}Install Dependency${NC}"
pip install --upgrade "jax[cpu]"
mamba install -n ros_base numpy scipy matplotlib jupyter notebook networkx shapely scikit-learn imageio -c conda-forge --yes
mamba install -n ros_base hpp-fcl -c conda-forge --yes

echo -e "${BLUE}Install PySpline${NC}"
# Mac OS
if [[ $OS == 'Darwin' ]]; then
  if [[ $ARCH == 'arm64' ]]; then
    echo -e "${GREEN} You are using Mac OS with Apple Silicon${NC}"
    # this avoid segfault when importing hppfcl 
    # mamba install -n ros_base eigenpy=2.7.10 --yes -c conda-forge 
    pip install osx_arm/pyspline-1.5.2-py3-none-any.whl
  elif [[ $ARCH == 'x86_64' ]]; then  
    echo -e "${GREEN} You are using Mac OS with X86${NC}"
    pip install osx/pyspline-1.5.2-py3-none-any.whl
  else
    echo -e "${RED} Unrecognized arch type, Mac OS with ${ARCH}. Cannot Install PySpline, Please compile from source${NC}"
    exit 1
  fi
  # create a command to start ros env in the terminal
  echo -e "alias ros_env='source $HOME/miniforge3/start_ros.sh'" >> $HOME/.zshrc

elif [[ $OS == "Linux" ]]; then
  # Linux
  if [[ $ARCH == 'aarch64' ]]; then
    echo -e "${GREEN}You are using Linux with aarch64${NC}"
    pip install linux_aarch64/pyspline-1.5.2-py3-none-linux_aarch64.whl
  elif [[ $ARCH == 'x86_64' ]]; then  
    echo -e "${GREEN}You are using Linux with X86${NC}"
    pip install linux/pyspline-1.5.2-py3-none-linux_x86_64.whl
  else
    echo -e "${RED} Unrecognized arch type, Linux with ${ARCH}. Cannot Install PySpline, Please compile from source${NC}"
    return 1 2>/dev/null
    exit 1
  fi
  # create a command to start ros env in the terminal
  echo -e "alias ros_env='source $HOME/miniforge3/start_ros.sh'" >> $HOME/.bashrc
else
  echo -e "${RED} OS: ${OS} not supported ${NC}"
  return 1 2>/dev/null
  exit 1
fi

# Add a line to .bashrc to activate the conda environment
echo -e "${Green}Finished! Reopen a new terminal to see if everything works. ${NC}"