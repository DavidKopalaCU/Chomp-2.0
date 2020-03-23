#!/usr/bin/env bash

# Install script for Chomp 2.0

FALSE=0
TRUE=1

cat docs/ascii_banner.txt
cat docs/intro.txt

INSTALL_DIR=$PWD

function check_raspberry_pi() {
    return $FALSE
}

function full_install() {
    echo "FULL INSTALL"

    echo "Installing system dependencies"
    # REALSENSE INSTALL
}

# Follows the instructions for installing ROS Melodic on Ubuntu
# Stolen from: http://wiki.ros.org/melodic/Installation/Ubuntu
function install_ros() {
    echo "\nChecking for ROS..."
    if ! which roscore > /dev/null; then
        echo "ROS not found! Installing..."

        # Start ROS Wiki Installation Tutorial
        # 1.2 Setup sources.list
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        # 1.3 Setup keys for new souce
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        # 1.4 Installation - Load new available packages
        sudo apt update
        # Modify the below package to install different configuations (desktop, ros-base)
        sudo apt install ros-melodic-desktop-full

        # 1.5 Initialize rosdep
        sudo rosdep init
        rosdep update
        # 1.6 Environment setup
        echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        # 1.7 Dependencies for building packages
        sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

        echo "Finished installing ROS!"
    else
        echo "ROS found! Not installing."
    fi
}

function check_platformio() {
    if ! which platformio > /dev/null; then
        return $FALSE
    fi
    return $TRUE
}
# Following the installation instructions at: https://docs.platformio.org/en/latest/installation.html#python-package-manager
function install_platformio() {
    if check_platformio; then 
        echo "Found platformio installation. Not installing"
        return
    fi

    pip install -U platformio
}

function setup_catkin_ws() {
    echo 'Linking to ~/catkin_ws'
    
    # Make the workspace directory if it already exists
    if [ ! -d "~/catkin_ws/src" ]; then
        echo "Making catkin workspace for ROS project"
        mkdir -p ~/catkin_ws/src

        echo "Building empty cakin workspace"
        cd ~/catkin_ws && catkin_make
    fi

    if [ ! -d "~/catkin_ws/src/chomp2" ]; then
        echo "Creating new ROS package"
        cd ~/catkin_ws && catkin_create_pkg chomp2 std_msgs rospy roscpp
        cp $INSTALL_DIR/ros_package.xml ~/catkin_ws/src/chomp2/package.xml

        echo "Attemption to build new ROS package"
        cd ~/catkin_ws && caktin_make
    fi

    if [ ! -d "~/catkin_ws/src/chomp2/scripts" ]; then
        echo "Linking ROS Package scripts"
        ln -s $INSTALL_DIR/scripts ~/catkin_ws/src/chomp2

        echo "Attempting to build ROS package"
        cd ~/catkin_ws && caktin_make
    else
        echo "Catkin Workspace already configured and linked! Not repeating."
    fi
}

function build_ros_package() {
    echo 'Building ROS package'
    if [ ! -d "~/catkin_ws/src/chomp2/scripts" ]; then
        echo "The catkin workspace does not seem to be configured. Set up the workspace before building the package"
        return $FALSE
    fi

    cd ~/catkin_ws && caktin_make
}

function install_service() {
    echo 'Installing system service'

    echo "Copying service file"
    sudo cp $INSTALL_DIR/RaspberryPi/chomp2.service /etc/systemd/system/

    echo "Setting up systemctl stuff"
    sudo systemctl start chomp2
    sudo systemctl enable chomp2
}

function flash_arduino() {
    echo 'Building and Flashing Ardino Firmware'
    cd $INSTALL_DIR/Arduino && platformio run -t upload
}

function main() {
    echo ""
    echo "Main Menu - Please select feature to run"
    echo "1. Install Full System"
    echo "2. Install ROS"
    echo "3. Install PlatformIO"
    echo "4. Setup ROS workspace"
    echo "5. Build/Flash Arduino"
    echo "E. Exit"
    
    read
    if [ "$REPLY" == "1" ]; then
        full_install
    elif [ "$REPLY" == "2" ]; then
        install_ros
    elif [ "$REPLY" == "3" ]; then
        install_platformio
    elif [ "$REPLY" == "4" ]; then
        setup_catkin_ws
    elif [ "$REPLY" == "5" ]; then
        flash_arduino
    elif [ "$REPLY" == "E" ]; then
        exit
    else
        echo "Option not recognized, please try again."
    fi
}

echo 'Checking if we are running on a Raspberry Pi'
if check_raspberry_pi; then
    echo 'This does not seem to be a Raspberry Pi! Would you like to continue? (y/N)'
    read
    if [ "$REPLY" != "y" ] && [ "$REPLY" != "Y" ]; then
        exit
    fi
fi

while true
do
    main
done