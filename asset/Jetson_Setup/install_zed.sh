#!/bin/bash

# install ZED sdk

# install dependence
sudo apt-get install -y libjpeg-turbo8 libturbojpeg libusb-1.0-0 \
        libopenblas-dev libarchive-dev libv4l-0 curl unzip libpng16-16 \
        libpng-dev libturbojpeg0-dev qt5-default libqt5opengl5 libqt5svg5

pip3 install --upgrade cython 
pip3 install --upgrade numpy opencv-python pyopengl

wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/3.6/ZED_SDK_Tegra_JP46_v3.6.3.run
chmod +x ZED_SDK_Tegra_JP46_v3.6.3.run

# during installation, choose no for automatic dependency install, then choose python3.8 for API
./ZED_SDK_Tegra_JP46_v3.6.3.run

rm ZED_SDK_Tegra_JP46_v3.6.3.run
