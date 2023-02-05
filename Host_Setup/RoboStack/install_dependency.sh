#!/bin/bash

# define color
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

OS=$(uname -s)
ARCH=$(uname -m)

conda activate ros_base
mamba install networkx --yes

echo -e "${BLUE}Install PySpline${NC}"
# Mac OS
if [[ $OS == 'Darwin' ]]; then
  if [[ $ARCH == 'arm64' ]]; then
    echo -e "${GREEN} You are using Mac OS with Apple Silicon${NC}"
    # this avoid segfault when importing hppfcl 
    mamba install eigenpy=2.7.10 --yes -c conda-forge 
    pip install osx_arm/pyspline-1.5.2-py3-none-any.whl
  elif [[ $ARCH == 'x86_64' ]]; then  
    echo -e "${GREEN} You are using Mac OS with X86${NC}"
    pip install osx/pyspline-1.5.2-py3-none-any.whl
  else
    echo -e "${RED} Unrecognized arch type, Mac OS with ${ARCH}. Cannot Install PySpline, Please compile from source${NC}"
    exit 1
  fi

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
else
  echo -e "${RED} OS: ${OS} not supported ${NC}"
  return 1 2>/dev/null
  exit 1
fi

echo -e "${BLUE}Install Jax${NC}"
# Instal Jax https://github.com/google/jax#conda-installation
mamba install jax=0.3.22 'jaxlib=0.3.22=cpu*' -c conda-forge --yes

echo -e "${BLUE}Install Hpp-FCL${NC}"

mamba install -c conda-forge hpp-fcl --yes
conda deactivate 
conda activate ros_base

echo -e "${Green}Finished! Reopen a new terminal to see if everything works. ${NC}"