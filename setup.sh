#!/bin/bash

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root or with sudo privileges."
  exit 1
fi

apt install software-properties-common lsb-release -y
add-apt-repository universe -y
apt update && apt install curl git -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
apt install /tmp/ros2-apt-source.deb -y
apt update && apt upgrade -y

apt install ros-humble-desktop -y
apt install ros-dev-tools libgazebo-dev gazebo terminator -y
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
apt install libignition-gazebo6 libignition-gazebo6-dev libignition-gazebo6-plugins ros-humble-ros-gz-bridge -y
apt-get update && apt install libgz-plugin2-dev ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* -y
