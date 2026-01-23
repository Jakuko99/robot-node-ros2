#!/bin/bash
set -e

if [[ $EUID -ne 0 ]]; then
  echo "This script must be run as root or with sudo privileges."
  exit 1
fi

echo "==> Installing base dependencies"
apt update
apt install -y software-properties-common lsb-release curl git wget gnupg

add-apt-repository universe -y

echo "==> Installing ROS 2 apt source"
ROS_APT_SOURCE_VERSION=$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F\" '{print $4}'
)

OS_CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${OS_CODENAME}_all.deb"

apt install -y /tmp/ros2-apt-source.deb

echo "==> Updating system"
apt update
apt upgrade -y

echo "==> Installing ROS 2 Humble"
apt install -y ros-humble-desktop ros-dev-tools terminator

echo "==> Setting up Gazebo repository (modern keyring method)"
mkdir -p /etc/apt/keyrings

curl -fsSL https://packages.osrfoundation.org/gazebo.key \
  | gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/gazebo.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable ${OS_CODENAME} main" \
> /etc/apt/sources.list.d/gazebo-stable.list

echo "==> Installing Gazebo & ROS-Gazebo integration"
apt update

apt install -y \
  gazebo \
  libgazebo-dev \
  libignition-gazebo6 \
  libignition-gazebo6-dev \
  libignition-gazebo6-plugins \
  libgz-plugin2-dev \
  ros-humble-ros-gz-bridge \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3*

echo "==> Done"
