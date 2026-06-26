#!/bin/bash
# Version: 1.6-fixed
# Ubuntu 20.04 -> ROS2 Foxy + Gazebo11
# Ubuntu 22.04 -> ROS2 Humble + Gazebo Classic packages
# Fixed:
#   1. ROS key binary keyring handling
#   2. Docker/root user .bashrc path
#   3. Foxy/Humble auto selection
#   4. Avoid risky dist-upgrade
#   5. Use python3-pip instead of pip

set -e

export DEBIAN_FRONTEND=noninteractive

# =========================
# Check root
# =========================
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo/root privileges."
    echo "Example: sudo bash ros2_install.sh"
    exit 1
fi

# =========================
# Basic variables
# =========================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

USERNAME="${SUDO_USER:-root}"

if [ "$USERNAME" = "root" ]; then
    USER_HOME="/root"
else
    USER_HOME="$(eval echo "~$USERNAME")"
fi

if [ ! -f /etc/os-release ]; then
    echo "Cannot find /etc/os-release"
    exit 1
fi

. /etc/os-release

Ubuntu_version="${VERSION_ID}"
UBUNTU_CODENAME="${VERSION_CODENAME}"

if [ "$Ubuntu_version" = "20.04" ]; then
    ROS_DISTRO="foxy"
    UBUNTU_SERIES="focal"
elif [ "$Ubuntu_version" = "22.04" ]; then
    ROS_DISTRO="humble"
    UBUNTU_SERIES="jammy"
else
    echo "This script only supports Ubuntu 20.04 or Ubuntu 22.04."
    echo "Current system: $PRETTY_NAME"
    exit 1
fi

LOG_FILE="${SCRIPT_DIR}/ros2_${ROS_DISTRO}_install.log"
ERR_FILE="${SCRIPT_DIR}/ros2_${ROS_DISTRO}_install.err"

rm -f "$LOG_FILE" "$ERR_FILE"

exec 1> >(tee -a "$LOG_FILE")
exec 2> >(tee -a "$ERR_FILE" >&2)

echo "============================================"
echo "ROS2 ${ROS_DISTRO} installation started!"
echo "Current user: $USERNAME"
echo "User home: $USER_HOME"
echo "Ubuntu version: $PRETTY_NAME"
echo "Ubuntu codename: $UBUNTU_SERIES"
echo "Log file: $LOG_FILE"
echo "Error file: $ERR_FILE"
echo "============================================"

# =========================
# Mirror configuration
# =========================
if [ "$(uname -m)" = "x86_64" ]; then
    UBUNTU_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/"
    SECURITY_MIRROR="http://security.ubuntu.com/ubuntu/"
else
    UBUNTU_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/"
    SECURITY_MIRROR="http://ports.ubuntu.com/ubuntu-ports/"
fi

echo "System architecture: $(uname -m)"
echo "Ubuntu mirror: $UBUNTU_MIRROR"

# =========================
# Clean old ROS source files
# =========================
echo "Removing old ROS source list files..."
rm -f /etc/apt/sources.list.d/*ros*.list
rm -f /etc/apt/sources.list.d/*ros2*.list
rm -f /etc/apt/sources.list.d/realman_ros2.list

# =========================
# Backup and rewrite Ubuntu sources
# =========================
echo "Backing up /etc/apt/sources.list..."
cp /etc/apt/sources.list "/etc/apt/sources.list.backup.$(date +%Y%m%d_%H%M%S)" || true

echo "Writing Ubuntu ${UBUNTU_SERIES} sources..."
cat > /etc/apt/sources.list <<EOF
deb ${UBUNTU_MIRROR} ${UBUNTU_SERIES} main restricted universe multiverse
deb ${UBUNTU_MIRROR} ${UBUNTU_SERIES}-updates main restricted universe multiverse
deb ${UBUNTU_MIRROR} ${UBUNTU_SERIES}-backports main restricted universe multiverse
deb ${SECURITY_MIRROR} ${UBUNTU_SERIES}-security main restricted universe multiverse
EOF

# =========================
# Update package index
# =========================
echo "Cleaning apt cache..."
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "Fixing possible broken dpkg state..."
dpkg --configure -a || true
apt-get -f install -y || true

echo "Updating apt index..."
apt-get update

# =========================
# Install basic tools
# =========================
echo "Installing basic tools..."
apt-get install -y \
    curl \
    ca-certificates \
    gnupg2 \
    lsb-release \
    software-properties-common

# =========================
# Add ROS key
# =========================
echo "Downloading ROS key..."

rm -f /usr/share/keyrings/ros-archive-keyring.gpg

curl -fsSL --connect-timeout 10 --max-time 60 \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "Checking ROS key..."

if ! gpg --show-keys /usr/share/keyrings/ros-archive-keyring.gpg | grep -q "Open Robotics"; then
    echo "ROS key download failed or key is invalid."
    echo "Please check network/proxy/GitHub raw access."
    echo ""
    echo "You can manually check with:"
    echo "  gpg --show-keys /usr/share/keyrings/ros-archive-keyring.gpg"
    exit 1
fi

echo "ROS key installed successfully."

# =========================
# Add ROS2 source
# =========================
echo "Adding ROS2 source..."

cat > /etc/apt/sources.list.d/realman_ros2.list <<EOF
deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu ${UBUNTU_SERIES} main
EOF

echo "Updating apt index after adding ROS2 source..."
apt-get update

# =========================
# Install Python tools
# =========================
echo "Installing Python tools..."
apt-get install -y \
    python3-dev \
    python3-pip \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# =========================
# Install terminal
# =========================
echo "Installing gnome-terminal..."
apt-get install -y gnome-terminal || true

# =========================
# Configure pip mirror
# =========================
echo "Configuring pip mirror..."
python3 -m pip config set global.index-url http://pypi.tuna.tsinghua.edu.cn/simple || true
python3 -m pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn || true

# =========================
# Install ROS2 and Gazebo
# =========================
if [ "$ROS_DISTRO" = "foxy" ]; then
    echo "Installing ROS2 Foxy Desktop..."
    apt-get install -y ros-foxy-desktop

    echo "Installing Gazebo Classic 11..."
    apt-get install -y gazebo11 libgazebo11-dev

    echo "Installing ROS2 Foxy Gazebo packages..."
    apt-get install -y ros-foxy-gazebo-ros-pkgs

    echo "Installing Foxy gazebo_ros2_control..."
    apt-get install -y ros-foxy-gazebo-ros2-control || true

elif [ "$ROS_DISTRO" = "humble" ]; then
    echo "Installing ROS2 Humble Desktop..."
    apt-get install -y ros-humble-desktop

    echo "Installing Gazebo Classic packages..."
    apt-get install -y gazebo

    echo "Installing ROS2 Humble Gazebo packages..."
    apt-get install -y ros-humble-gazebo-ros-pkgs

    echo "Installing Humble gazebo_ros2_control..."
    apt-get install -y ros-humble-gazebo-ros2-control || true
fi

# =========================
# ROS dev tools
# =========================
echo "Installing ROS development tools..."
apt-get install -y ros-dev-tools || true

# =========================
# Environment setup
# =========================
BASHRC_FILE="${USER_HOME}/.bashrc"

echo "Setting ROS environment in ${BASHRC_FILE}..."

touch "$BASHRC_FILE"

if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" "$BASHRC_FILE"; then
    {
        echo ""
        echo "# ROS2 ${ROS_DISTRO} Environment Setting"
        echo "source /opt/ros/${ROS_DISTRO}/setup.bash"
    } >> "$BASHRC_FILE"

    echo "ROS2 ${ROS_DISTRO} environment added to ${BASHRC_FILE}"
else
    echo "ROS2 ${ROS_DISTRO} environment already exists in ${BASHRC_FILE}"
fi

# shellcheck disable=SC1090
source "$BASHRC_FILE" || true

# =========================
# rosdep / rosdepc
# =========================
echo "Installing rosdep and rosdepc..."

python3 -m pip install -U pip || true
python3 -m pip install rosdep rosdepc || true

echo "Initializing rosdepc..."

rosdepc init || true

echo "rosdepc init completed."

# =========================
# Final apt update
# =========================
echo "Final apt update..."
apt-get update

# Do not run dist-upgrade automatically.
# It may change too many system packages.
# apt-get dist-upgrade -y

# =========================
# Verification
# =========================
echo ""
echo "============================================"
echo "ROS2 ${ROS_DISTRO} installation completed!"
echo "============================================"
echo ""

echo "Please run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  ros2 --version"
echo "  ros2 pkg list | grep gazebo"
echo ""

if [ "$ROS_DISTRO" = "foxy" ]; then
    echo "Gazebo Classic check:"
    echo "  gazebo --version"
    echo "  gzserver --version"
    echo ""
fi

echo "Log file:"
echo "  ${LOG_FILE}"
echo "Error file:"
echo "  ${ERR_FILE}"
echo ""