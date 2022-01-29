#!/bin/bash

sudo ./ssd_extern.sh

#cd $work_dir
sudo apt-get update 
sudo apt-get -y upgrade

sudo apt -y autoremove 

./disable_BT.sh

sudo reboot

