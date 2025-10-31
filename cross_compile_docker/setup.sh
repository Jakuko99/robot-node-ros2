# !/bin/bash

apt update && apt install -y python3-colcon-common-extensions build-essential wget git openssh-client
source /opt/ros/humble/setup.bash
wget https://github.com/WiringPi/WiringPi/releases/download/3.16/wiringpi_3.16_arm64.deb
dpkg -i wiringpi_3.16_arm64.deb
rm wiringpi_3.16_arm64.deb