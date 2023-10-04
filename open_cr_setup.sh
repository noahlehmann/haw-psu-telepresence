#!/bin/bash

#For further Information see: https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup

# install required pacḱages for OpenCR
sudo dpkg --add-architecture armhf && sudo apt-get update && sudo apt-get install libc6:armhf

#add OpenCR Model Name

export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger_noetic
rm -rf ./opencr_update.tar.bz2

#Download Firmware and loader

wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2

#Upload the opencr
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr

