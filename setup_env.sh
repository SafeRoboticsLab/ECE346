#!/bin/bash

work_dir=~/Documents

# install gfortran
sudo apt-get install gfortran

# install pyspline
git clone https://github.com/mdolab/pyspline.git $work_dir/pyspline
cp asset/Jetson_Setup/config.mk $work_dir/pyspline/config/config.mk
cd $work_dir/pyspline
make

cd $work_dir

virtualenv lab2_env --python=/usr/bin/python3.8
source lab2_env/bin/activate

# set up env in virtualenv
pip install -e acados/interfaces/acados_template
pip install numpy matplotlib scipy 
pip install pyyaml rospkg empy sympy catkin_pkg

cd $work_dir/pyspline
pip install .
