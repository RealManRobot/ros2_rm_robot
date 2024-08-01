#!/bin/bash
# Version: 1.4
# Date: 2023-06-19
# Author: Herman Ye @Realman Robotics
# Warning: This script is ONLY for ROS2 Humble in ubuntu 20.04
# set -x
set -e



# UBUNTU CONFIGURATION BEGINS HERE

# Check if script is run as root (sudo)
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo privileges. for example: sudo bash ros2_humble_install.sh"
    read -p "Press any key to exit..."
    exit 1
fi


if [ $(uname -m) = "x86_64" ]; then
    sudo rm libRM_Service.so*
    sudo tar -jxvf linux_x86_service_release*

    if [ -f "/usr/local/lib/libRM_Service.so" ];then
        echo "find file"
        sudo rm /usr/local/lib/libRM_Service.so*
        sudo rm /usr/local/lib/linux_x86_service_release*
    fi
    sudo cp linux_x86_service_release* /usr/local/lib
    cd /usr/local/lib
    sudo tar -jxvf /usr/local/lib/linux_x86_service_release*
else
    sudo rm libRM_Service.so*
    sudo tar -jxvf linux_arm_service_release*

    if [ -f "/usr/local/lib/libRM_Service.so" ];then
        sudo rm /usr/local/lib/libRM_Service.so*
        sudo rm /usr/local/lib/linux_arm_service_release*
    fi
    sudo cp linux_arm_service_release* /usr/local/lib
    cd /usr/local/lib
    sudo tar -jxvf /usr/local/lib/linux_arm_service_release*
fi 
if ! grep -q "/usr/local/lib" /etc/ld.so.conf; then
    echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf
else
    echo "/usr/local/lib is already set in /etc/ld.so.conf"
fi
sudo /sbin/ldconfig
cd ~
TEXT1="Lib installation completed!"
TEXT1_PADDING=$((($TERMINAL_WIDTH-${#TEXT1})/2))
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo -e "${GREEN}$(printf '%*s' $TEXT1_PADDING)${TEXT1} ${NC}"
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""

