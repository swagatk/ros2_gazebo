#!/usr/bin/env bash
# ==============================================================================
# Setup Script for ROS 2 Jazzy, Gazebo Harmonic, and TurtleBot3 Simulation
# Optimized for Ubuntu 24.04 on WSL2 (Sanitized Windows PATH)
# ==============================================================================

set -e # Exit immediately if a command exits with a non-zero status

echo "=== [0/6] Sanitizing PATH for WSL Environment ==="
# Strip Windows /mnt/ paths from PATH to prevent CMake from discovering Windows libraries (e.g., Anaconda)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '^/mnt/' | tr '\n' ':' | sed 's/:$//')

echo "=== [1/6] Updating System & Installing Base Utilities ==="
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl wget gnupg lsb-release xterm git
sudo add-apt-repository universe -y

echo "=== [2/6] Installing ROS 2 Jazzy Desktop ==="
# Setup ROS 2 apt source repository using official ros-apt-source package
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb

# Install development tools and ROS 2 Desktop
sudo apt update
sudo apt install -y ros-dev-tools ros-jazzy-desktop

echo "=== [3/6] Installing Gazebo Harmonic & ROS-GZ Bridge ==="
# Add Open Robotics / Gazebo repository keyring and source list
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic ros-jazzy-ros-gz

echo "=== [4/6] Installing Nav2, Cartographer, and Colcon Tools ==="
sudo apt install -y \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-route \
    python3-colcon-common-extensions \
    libprotobuf-dev \
    protobuf-compiler \
    libabsl-dev

echo "=== [5/6] Downloading & Building TurtleBot3 Workspace ==="
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Clone the Jazzy branches of TurtleBot3 packages
if [ ! -d "DynamixelSDK" ]; then
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
fi
if [ ! -d "turtlebot3_msgs" ]; then
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
fi
if [ ! -d "turtlebot3" ]; then
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
fi
if [ ! -d "turtlebot3_simulations" ]; then
    git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
fi

# Clean prior contaminated build/install directories if they exist
cd ~/turtlebot3_ws
rm -rf build/ install/ log/

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build workspace while instructing CMake to completely ignore /mnt/c
colcon build --symlink-install --cmake-args -DCMAKE_IGNORE_PREFIX_PATH="/mnt/c"

echo "=== [6/6] Configuring Shell Environment (~/.bashrc) ==="
append_bashrc() {
    local line="$1"
    if ! grep -Fxq "$line" ~/.bashrc; then
        echo "$line" >> ~/.bashrc
    fi
}

append_bashrc "source /opt/ros/jazzy/setup.bash"
append_bashrc "export ROS_DOMAIN_ID=23"
append_bashrc "source ~/turtlebot3_ws/install/setup.bash"
append_bashrc "export TURTLEBOT3_MODEL=burger"

echo "=========================================================="
echo "Installation complete!"
echo "Run 'source ~/.bashrc' to start using ROS 2 and TurtleBot3."
echo "=========================================================="