#!/bin/bash

# need to fix the render 
# https://discourse.acados.org/t/problems-with-t-renderer/438

# sudo -H pip3 install --upgrade --no-deps --force-reinstall matplotlib scipy numpy 
work_dir=~/Documents
cd $work_dir

# install rust
curl https://sh.rustup.rs -sSf -o install_rust.sh
sh install_rust.sh -y
rm install_rust.sh

source ~/.bashrc
# get tera_renderer
git clone https://github.com/acados/tera_renderer.git
cd tera_renderer
cargo build --verbose --release
cd ..

git clone --recursive https://github.com/acados/acados.git
cd acados
mkdir -p build
cd build
cmake -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC -DACADOS_WITH_QPOASES=ON \
        -DACADOS_WITH_HPMPC=OFF -DACADOS_WITH_QORE=ON -DACADOS_WITH_OOQP=ON \
        -DACADOS_WITH_QPDUNES=ON -DACADOS_WITH_OSQP=ON ..
make install -j4

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$work_dir/acados/lib"" >> ~/.bashrc
echo "export ACADOS_SOURCE_DIR="$work_dir/acados"" >> ~/.bashrc

# copy the tera_renderer

cd ..
cp ../tera_renderer/target/release/t_renderer bin/

# create the virtualenv
cd $work_dir
virtualenv ACADOS_env --python=/usr/bin/python3.8
source ACADOS_env/bin/activate

# set up env in virtualenv
pip install -e acados/interfaces/acados_template
pip install pyyaml rospkg empy sympy spatialmath-python pyclothoids

# install gfortran
sudo apt-get install gfortran
# install pyspline
git clone https://github.com/mdolab/pyspline.git
cd pyspline
cp config/defaults/config.LINUX_GFORTRAN.mk config/config.mk
make
pip install .