#!/bin/bash

# define color
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

cur_dir=$(pwd)

OS=$(uname -s)
ARCH=$(uname -m)

# Check if conda installed
if test -z "$(which $CONDA_EXE)" ; then
  echo -e "${RED} Conda is not installed, install Mambaforge ${NC}"


  # Mac OS
  if [[ $OS == 'Darwin' ]]; then
    if [[ $ARCH == 'arm64' ]]; then
      echo -e "${GREEN} You are using Mac OS with Apple Silicon${NC}"
      curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-MacOSX-arm64.sh > Mambaforge-Installer.sh
    elif [[ $ARCH == 'x86_64' ]]; then  
      echo -e "${GREEN} You are using Mac OS with X86${NC}"
      curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-MacOSX-x86_64.sh > Mambaforge-Installer.sh
    else
      echo -e "${RED} Unrecognized arch type, Mac OS with ${ARCH}${NC}"
      return 1 2>/dev/null
      exit 1
    fi

  elif [[ $OS == "Linux" ]]; then
    # Linux
    if [[ $ARCH == 'aarch64' ]]; then
      echo -e "${GREEN}You are using Linux with aarch64${NC}"
      curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-Linux-aarch64.sh > Mambaforge-Installer.sh
    elif [[ $ARCH == 'x86_64' ]]; then  
      echo -e "${GREEN}You are using Linux with X86${NC}"
      curl -L https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-Linux-x86_64.sh > Mambaforge-Installer.sh
    else
      echo -e "${RED} Unrecognized arch type, Linux with ${ARCH}${NC}"
      return 1 2>/dev/null
      exit 1
    fi

  else
    echo -e "${RED} OS: ${OS} not supported ${NC}"
    exit 1
  fi

  # Install Mambaforge
  chmod +x Mambaforge-Installer.sh
  ./Mambaforge-Installer.sh -b -f

  cd ~/mambaforge/bin
  ./conda config --set auto_activate_base false
  ./conda init --all
  ./mamba init --all

  # manually enable conda and mamba
  eval "$(./conda shell.bash hook)"
  cd ~/mambaforge/etc/profile.d/
  . "mamba.sh"

  cd $cur_dir
  rm Mambaforge-Installer.sh

  echo -e "${Green} Finish install Mambaforge${NC}"

else
  echo -e "${BLUE}Found Conda in $CONDA_EXE${NC}, install Mamba to base environment"
  eval "$($CONDA_EXE shell.bash hook)"
  conda activate base
  conda install mamba -c conda-forge --yes
  mamba init --all

  # manually enable mamba
  cd $(conda info --base)/etc/profile.d/
  . "mamba.sh"
fi

cd $cur_dir

echo -e "Current Conda is running on ${GREEN}$CONDA_EXE${NC}"

conda --version
mamba --version

# make sure we are in base 
echo -e "${BLUE}Setup RoboStack Env${NC}"
conda activate base

mamba create -n ros_base ros-noetic-desktop python=3.9 -c robostack -c robostack-experimental \
                            -c conda-forge --no-channel-priority --override-channels --yes


echo -e "${BLUE}Finish Settting up RoboStack Env${NC}"

conda activate ros_base

mamba install compilers cmake pkg-config make ninja -c conda-forge --override-channels --yes

mamba install catkin_tools --yes

# reload environment to activate required scripts before running anything
# on Windows, please restart the Anaconda Prompt / Command Prompt!
conda deactivate
conda activate ros_base


# if you want to use rosdep, also do:
mamba install rosdep --yes
rosdep init  # note: do not use sudo!
rosdep update

echo -e "${BLUE}Install Dependency${NC}"

mamba install numpy scipy matplotlib --yes
mamba install jupyter notebook --yes

echo -e "${Green}Finished! Reopen a new terminal to see if everything works. ${NC}"