#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

# get ROS version
distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

# rc file
if [ $SHELL = /usr/bin/zsh ]; then
SHELL_TYPE=zsh
elif [ $SHELL = /bin/bash ]; then
SHELL_TYPE=bash
fi

RCFILE=.${SHELL_TYPE}rc

pip install --upgrade rospkg

# install ROS dependencies using rosdep
cd $SCRIPT_PATH
# sudo sh -c 'echo "yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml" > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list'
# rosdep update
# sudo rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y

wget https://dnb-public-downloads-misc.s3.eu-central-1.amazonaws.com/pylon/pylon_5.2.0.13457-deb0_amd64.deb
sudo dpkg -i pylon_5.2.0.13457-deb0_amd64.deb


