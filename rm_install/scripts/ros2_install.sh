#!/bin/bash
# Version: 1.4
# Date: 2023-06-19
# Author: Herman Ye @Realman Robotics
# Warning: This script is ONLY for ROS2 Jazzy in Ubuntu 24.04.
# set -x
set -e



# UBUNTU CONFIGURATION BEGINS HERE

# Check if script is run as root (sudo)
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo privileges. for example: sudo bash ros2_install.sh"
    read -p "Press any key to exit..."
    exit 1
fi
# Get script directory
SCRIPT_DIR=$(dirname "$0")
# Get the username of the non-root user
USERNAME=$SUDO_USER
Ubuntu_version=$(lsb_release -r --short)
UBUNTU_CODENAME="noble"
ROS_DISTRO="jazzy"

if [ "$Ubuntu_version" != "24.04" ]; then
    echo "This script must be run with Ubuntu 24.04"
    read -p "Press any key to exit..."
    exit 1
fi

echo "Current user is: $USERNAME"
# Save logs to files
LOG_FILE="${SCRIPT_DIR}/ros2_${ROS_DISTRO}_install.log"
ERR_FILE="${SCRIPT_DIR}/ros2_${ROS_DISTRO}_install.err"
rm -f ${LOG_FILE}
rm -f ${ERR_FILE}

# Redirect output to console and log files
exec 1> >(tee -a ${LOG_FILE} )
exec 2> >(tee -a ${ERR_FILE} >&2)

# Output log info to console
echo "ROS2 ${ROS_DISTRO} installation started!"  
echo "Installation logs will be saved to ${LOG_FILE}"
echo "Installation errors will be saved to ${ERR_FILE}"

# No Password sudo config
sudo sed -i 's/^%sudo.*/%sudo ALL=(ALL) NOPASSWD:ALL/g' /etc/sudoers

# Get architecture of the system
if [ $(uname -m) = "x86_64" ]; then
  MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/"
else
  MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/"
fi
echo "Current system architecture is: $(uname -m)"
echo "Current mirror is: $MIRROR"

# Backup original software sources
if [ -f /etc/apt/sources.list ]; then
  sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
else
  sudo touch /etc/apt/sources.list
fi
# Clear original software sources
sudo sh -c ': > /etc/apt/sources.list'
sudo sh -c ': > /etc/apt/sources.list.d/realman_ros2.list'

# Replace software sources
echo "deb $MIRROR ${UBUNTU_CODENAME} main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb $MIRROR ${UBUNTU_CODENAME}-updates main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb $MIRROR ${UBUNTU_CODENAME}-backports main restricted universe multiverse" >> /etc/apt/sources.list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros2/ubuntu ${UBUNTU_CODENAME} main" >> /etc/apt/sources.list.d/realman_ros2.list

if [ $(uname -m) = "x86_64" ]; then
  echo "deb http://security.ubuntu.com/ubuntu/ ${UBUNTU_CODENAME}-security main restricted universe multiverse" >> /etc/apt/sources.list
else
  echo "deb http://ports.ubuntu.com/ubuntu-ports/ ${UBUNTU_CODENAME}-security main restricted universe multiverse" >> /etc/apt/sources.list
fi

# Install Curl
sudo apt-get install curl -y # If you haven't already installed curl
sudo apt-get install software-properties-common -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# System update
sudo apt-get update
sudo apt-get upgrade -y


# Install pip
sudo apt-get install python3-dev -y
sudo apt-get install python3-pip -y # If you haven't already installed pip

# Install gnome-terminal
sudo apt-get install gnome-terminal -y # If you haven't already installed gnome-terminal

# Set default pip source
pip config set global.index-url http://pypi.tuna.tsinghua.edu.cn/simple
pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn

# Add the ROS key
# ros_key="${SCRIPT_DIR}/ros.key"
# rm -f "${ros_key}"
# wget http://packages.ros.org/ros.key
# sudo apt-key add ros.key

#First ensure that the Ubuntu Universe repository is enabled.
sudo apt-get install software-properties-common -y
sudo add-apt-repository universe

# Update the system packages index to the latest version

sudo apt-get update

# Install ROS2 Jazzy
sudo apt-get install ros-${ROS_DISTRO}-desktop -y
sudo apt-get install ros-${ROS_DISTRO}-ros-gz -y
sudo apt-get install \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-gz-ros2-control \
  -y

sudo apt-get install ros-dev-tools -y

# Environment setup
if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" /home/$USERNAME/.bashrc; then

    echo "# ROS2 ${ROS_DISTRO} Environment Setting" | sudo tee -a /home/$USERNAME/.bashrc
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" | sudo tee -a /home/$USERNAME/.bashrc
    echo "ROS2 ${ROS_DISTRO} environment setup added to /home/$USERNAME/.bashrc"
else
    echo "ROS2 ${ROS_DISTRO} environment is already set in /home/$USERNAME/.bashrc"
fi

source /home/$USERNAME/.bashrc

# Initialize rosdepc by fishros under BSD License
# https://pypi.org/project/rosdepc/#files
if python3 -m pip install --help | grep -q -- "--break-system-packages"; then
  sudo python3 -m pip install --break-system-packages rosdep
  sudo python3 -m pip install --break-system-packages rosdepc
else
  sudo python3 -m pip install rosdep
  sudo python3 -m pip install rosdepc
fi
# sudo pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -U rosdep
# Init & update rosdep 
sudo rosdepc init > /dev/null
#sudo rosdep fix-permissions
# su -l $USERNAME -c 'rosdepc update' > /dev/null
echo "rosdepc init completed!"

# System update again
sudo apt-get update
sudo apt-get dist-upgrade -y

# Verifying ROS2 installation
clear

source /home/$USERNAME/.bashrc
# Define the variables to be printed
TEXT1="ROS2 installation completed!"
TEXT2="Please open a new terminal and run demo to verify the installation:"

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

# Print the text in the center of the screen in the desired colors
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
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT2} ${NC}"
echo -e "${RED}$(printf '%*s' $TEXT3_PADDING)${TEXT3} ${NC}"
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
