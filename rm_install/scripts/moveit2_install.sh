#!/bin/bash
# Version: 1.3
# Date: 2023-07-19
# Author: Herman Ye @Realman Robotics
#
# Warning: This script is ONLY for ROS2 Jazzy in Ubuntu 24.04.
# If ROS2 has not been installed, please execute ros2_install.sh first.
#
# set -x
set -e

# Get script directory
SCRIPT_DIR=$(dirname "$0")
# Get the username of the non-root user
USERNAME=$SUDO_USER
echo "Current user is: $USERNAME"

# Check if script is run as root (sudo)
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo privileges. for example: sudo bash moveit2_install.sh"
    read -p "Press any key to exit..."
    exit 1
fi
echo "sudo privileges check passed"

UBUNTU_CODENAME=$(lsb_release -sc)
ROS_DISTRO="jazzy"

if [ "$UBUNTU_CODENAME" != "noble" ]; then
    echo "This script must be run in Ubuntu 24.04"
    read -p "Press any key to exit..."
    exit 1
fi

echo "Ubuntu version check passed"

# Check if the matching ROS2 desktop package has been installed.
if dpkg -l "ros-${ROS_DISTRO}-desktop" 2>/dev/null | grep -q "^ii"; then
    echo "ROS2 ${ROS_DISTRO} check passed"
else
    echo "This script must be run with ROS2 ${ROS_DISTRO}-desktop"
    read -p "Press any key to exit..."
    exit 1
fi

# Save logs to files
LOG_FILE="${SCRIPT_DIR}/moveit2_install.log"
ERR_FILE="${SCRIPT_DIR}/moveit2_install.err"
rm -f ${LOG_FILE}
rm -f ${ERR_FILE}

# Redirect output to console and log files
exec 1> >(tee -a ${LOG_FILE} )
exec 2> >(tee -a ${ERR_FILE} >&2)

# Add GitHub520 Host to host for GitHub access in China
# https://github.com/521xueweihan/GitHub520
sudo apt-get install curl -y
sudo sed -i "/# GitHub520 Host Start/Q" /etc/hosts && curl https://raw.hellogithub.com/hosts >> /etc/hosts
echo "GitHub520 Host added to host file"
# sudo sed -i 's/#DNS=/DNS=114.114.114.114/' /etc/systemd/resolved.conf
# echo "DNS server changed to 114.114.114.114"
sudo systemctl restart systemd-resolved.service
echo "Refreshed network settings, sleep 5 seconds"
sleep 5

# Install wstool
sudo apt-get install python3-wstool -y

# Install moveit
sudo apt-get install ros-${ROS_DISTRO}-moveit -y
# sudo apt-get install ros-${ROS_DISTRO}-moveit-visual-tools -y
# Warning: Installing all subpackages of moveit may cause dependency conflicts, please do so with caution.
sudo apt-get install ros-${ROS_DISTRO}-moveit-* -y
# Install ros2_control and Gazebo dependencies used by rm_gazebo demos.
sudo apt-get install \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-gz-ros2-control \
    -y


clear
sleep 1

# Define the variables to be printed
TEXT0=""
TEXT1="Moveit installation completed!"
TEXT2="Please open a new terminal and run roslaunch to verify the installation:"
TEXT3="roslaunch panda_moveit_config demo.launch"
TEXT4="1. Click 'Add' in the left panel, and add the following items:"
TEXT5="2. Add 'RobotModel', 'MotionPlanning' to the left panel"
TEXT6="3. Try to drag the end effector to see if the robot arm moves"
TEXT7="4. Click 'Plan & Execute' to see the robot arm move"
TEXT8="5. If you see the robot arm move, the installation is successful"
# Define the colors
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[1;32m'
NC='\033[0m'

# Calculate the center of the terminal window
TERMINAL_WIDTH=$(tput cols)
TEXT1_PADDING=$((($TERMINAL_WIDTH-${#TEXT1})/2))
TEXT2_PADDING=$((($TERMINAL_WIDTH-${#TEXT2})/2))
TEXT3_PADDING=$((($TERMINAL_WIDTH-${#TEXT3})/2))

# Finished
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${GREEN}$(printf '%*s' $TEXT1_PADDING)${TEXT1} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
read -p "Ok. The installation is complete. Press any key to exit..."
exit 0
