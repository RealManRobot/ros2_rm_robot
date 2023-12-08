#!/bin/bash
# Version: 1.3
# Date: 2023-07-19
# Author: Herman Ye @Realman Robotics
#
# Warning: This script assumes that the ubuntu20.04 system and ROS2 Foxy have been installed correctly
# If not, please execute ROS2_Foxy_install.sh first.
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

# Check if script is run in ubuntu20.04
if [ "$(lsb_release -sc)" != "focal" ]; then
    if [ "$(lsb_release -sc)" != "jammy" ]; then
        echo "This script must be run in ubuntu20.04 or 22.04"
        read -p "Press any key to exit..."
        exit 1
    fi
fi

echo "Ubuntu version check passed"

# Check if script is run in ROS2 Foxy
if [[ "$(sudo -u $USERNAME dpkg -l ros-foxy-desktop)" == *ii* ]]; then
    echo "ROS2 foxy check passed"
elif [[ "$(sudo -u $USERNAME dpkg -l ros-humble-desktop)" == *ii* ]]; then
    echo "ROS2 humble check passed"
else
    echo "This script must be run with ROS2 HumbleorFoxy-desktop-full"
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

if [ "$(lsb_release -sc)" = "focal" ]; then
    # Install moveit
    sudo apt-get install ros-foxy-moveit -y
    # sudo apt-get install ros-foxy-moveit-visual-tools -y
    # Warning: Installing all subpackages of moveit may cause dependency conflicts, please do so with caution.
    sudo apt-get install ros-foxy-moveit-* -y
    # Install ros_control
    sudo apt-get install ros-foxy-controller-interface ros-foxy-controller-manager-msgs ros-foxy-controller-manager
elif [ "$(lsb_release -sc)" = "jammy" ]; then
    # Install moveit
    sudo apt-get install ros-humble-moveit -y
    # sudo apt-get install ros-foxy-moveit-visual-tools -y
    # Warning: Installing all subpackages of moveit may cause dependency conflicts, please do so with caution.
    sudo apt-get install ros-humble-moveit-* -y
    # Install ros_control
    sudo apt-get install ros-humble-controller-interface ros-humble-controller-manager-msgs ros-humble-controller-manager
fi
# Create A Catkin Workspace and Download MoveIt Source


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
# read -rp "Do you want to download the tutorial code? (y/n)  " confirm
#   if [[ "$confirm" =~ ^[Yy]$ ]]; then
#     # Download Example Code(already in the moveit.rosinstall)
#     # cd /home/$USERNAME/ws_moveit/src  
#     sudo rm -rf /home/$USERNAME/ws_moveit
#     mkdir -p /home/$USERNAME/ws_moveit/src
#     cd  /home/$USERNAME/ws_moveit/src
#     clear
#     echo "Connecting to GitHub, please wait..."
#     echo "If the download stuck here for a long time"
#     echo "please check your network connection and rerun this script"
#     # install franka robot as demo
#     sudo apt-get install ros-foxy-franka-* -y
#     git clone https://github.com/ros-planning/moveit_tutorials.git -b master
#     git clone https://github.com/ros-planning/panda_moveit_config.git -b Foxy-devel

#     # git clone https://github.com/ros-controls/ros_control.git -b Foxy-devel

#     # Clone MoveIt packages from source
#     # git clone https://github.com/ros-planning/moveit_msgs.git
#     # git clone https://github.com/ros-planning/moveit_resources.git
#     # git clone https://github.com/ros-planning/geometric_shapes.git --branch Foxy-devel
#     # git clone https://github.com/ros-planning/srdfdom.git --branch Foxy-devel
#     # git clone https://github.com/ros-planning/moveit.git
#     # git clone https://github.com/PickNikRobotics/rviz_visual_tools.git
#     # git clone https://github.com/ros-planning/moveit_visual_tools.git
#     # git clone https://github.com/ros-planning/moveit_tutorials.git
#     # git clone https://github.com/ros-planning/panda_moveit_config.git --branch Foxy-devel
#         # Rosdepc install
#     cd /home/$USERNAME/ws_moveit/src
#     rosdepc install -y --from-paths . --ignore-src --rosdistro Foxy > /dev/null
#     echo "Rosdep install finished"

#     # Build the Workspace
#     cd /home/$USERNAME/ws_moveit
#     catkin config --extend /opt/ros/Foxy --cmake-args -DCMAKE_BUILD_TYPE=Release
#     catkin init
#     catkin build

#     # Environment setup
#     if ! grep -q "/home/$USERNAME/ws_moveit/devel/setup.bash" /home/$USERNAME/.bashrc; then

#         echo "# ws_moveit Environment Setting" | sudo tee -a /home/$USERNAME/.bashrc
#         echo "source /home/$USERNAME/ws_moveit/devel/setup.bash" >> /home/$USERNAME/.bashrc
#         echo "ws_moveit environment setup added to /home/$USERNAME/.bashrc"
#     else
#         echo "ws_moveit environment is already set in /home/$USERNAME/.bashrc"
#     fi
#     source /home/$USERNAME/.bashrc

#     # Verifying Moveit1 installation
#     clear


#     # Print the text in the center of the screen in the desired colors
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
#     echo -e "${GREEN}$(printf '%*s' $TEXT1_PADDING)${TEXT1} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT2} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
#     echo -e "${RED}$(printf '%*s' $TEXT3_PADDING)${TEXT3} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT4} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT5} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT6} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT7} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT8} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT0} ${NC}"
#     echo -e "${NC}$(printf '%*s' $TEXT1_PADDING)${TEXT0} ${NC}"
#   else
read -p "Ok. The installation is complete. Press any key to exit..."
exit 0
#   fi
