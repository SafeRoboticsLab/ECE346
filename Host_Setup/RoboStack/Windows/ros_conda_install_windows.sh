#! /bin/bash

# define color
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# make sure we are in base 
echo -e "${BLUE}Setup RoboStack Env${NC}"
conda activate base

mamba create -n ros ros-noetic-desktop python=3.9 -c robostack -c robostack-experimental -c conda-forge --no-channel-priority --override-channels --yes

echo -e "${BLUE}Finish Settting up RoboStack Env${NC}"

conda activate ros

echo -e "${BLUE}Install Dependency${NC}"

mamba install compilers cmake pkg-config make ninja --yes

mamba install vs2019_win-64 --yes

mamba install numpy scipy matplotlib --yes

# Instal Jax https://github.com/google/jax#conda-installation
# mamba install jax -c conda-forge --yes
pip install "jax[cpu]===0.3.14" -f https://whls.blob.core.windows.net/unstable/index.html --use-deprecated legacy-resolver

# reload environment to activate required scripts before running anything
# on Windows, please restart the Anaconda Prompt / Command Prompt!
conda deactivate
conda activate ros

# if you want to use rosdep, also do:
mamba install rosdep --yes
rosdep init  # note: do not use sudo!
rosdep update