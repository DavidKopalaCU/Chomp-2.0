#!/bin/bash

# Install script for Chomp 2.0

FALSE=0
TRUE=1
                                                          
cat docs/ascii_banner.txt
cat docs/intro.txt

function check_raspberry_pi() {
    return $FALSE
}

function full_install() {
    echo "FULL INSTALL"
}

function install_ros() {
    echo "\nChecking for ROS..."
    if ! which roscore > /dev/null; then
        echo "ROS not found! Installing..."
    else
        echo "ROS found!"
    fi
}

function check_platformio() {
    if ! which platformio > /dev/null; then
        return $FALSE
    fi
    return $TRUE
}
function install_platformio() {
    echo '\nChecking for PlatformIO installation...'
    if ! which platformio > /dev/null; then
        echo "PlatformIO not found! Installing..."
    else
        echo "PlatformIO found!"
    fi
}

function setup_catkin_ws() {
    echo 'Linking to ~/catkin_ws'
    # mkdir -p ~/catkin_ws/src/
}

function build_ros_package() {
    echo 'Building ROS package'
}

function install_service() {
    echo 'Installing system service'
}

function flash_arduino() {
    echo 'Building Ardino Firmware'
    echo 'Flashing Arduino'
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