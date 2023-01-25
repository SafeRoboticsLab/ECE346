#!/bin/bash

sudo apt-get update 

# install git
sudo apt-get install -y git

# upgrade cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository -y "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt update
sudo apt-get install -y cmake

# install jetson stats
sudo -H pip3 install --upgrade --force-reinstall jetson-stats

sudo apt-get install -y python3.8-dev
#update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# wget https://bootstrap.pypa.io/pip/2.7/get-pip.py
# python get-pip.py --user
# rm get-pip*
sudo apt-get install -y python-pip python-numpy python-matplotlib python-scipy python-virtualenv

wget https://bootstrap.pypa.io/get-pip.py
python3.8 get-pip.py --user
rm get-pip*

# add pip3.8 to path
echo "export PATH="$HOME/.local/bin:$PATH"" >> ~/.bashrc
export PATH="$HOME/.local/bin:$PATH"

# fix dependency
pip3 install testresources

pip3 install --upgrade matplotlib scipy numpy virtualenv

./install_ros.sh
./install_zed.sh
./install_acados.sh

# Install controller dependence
sudo apt-get install libusb-1.0-0-dev mono-runtime libmono-system-windows-forms4.0-cil -y
sudo cp 99-pololu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules

# install vs code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg

sudo apt install apt-transport-https
sudo apt update
sudo apt install code # or code-insiders



