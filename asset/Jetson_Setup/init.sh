#!/bin/bash



#cd $work_dir
sudo apt-get update 
sudo apt-get -y upgrade

# install git
sudo apt install -y git

# upgrade cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository -y "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt update
sudo apt install -y cmake

sudo apt-get install -y python3.8-dev
#update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1


# wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
# python get-pip.py --user
# rm get-pip*
sudo apt-get install -y python-pip python-numpy python-matplotlib python-scipy python-virtualenv

wget https://bootstrap.pypa.io/get-pip.py
python3.8 get-pip.py --user
rm get-pip*

# fix dependency
pip3 install testresources

pip3 install --upgrade matplotlib scipy numpy virtualenv

# install jetson stats
sudo -H pip install jetson-stats

sh install_ros.sh
sh install_zed.sh
sh install_acados.sh

# Install controller dependence
sudo apt-get install libusb-1.0-0-dev mono-runtime libmono-system-windows-forms4.0-cil -y





