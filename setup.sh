#!/bin/bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

sudo apt update

sudo apt install ros-melodic-desktop-full

apt search ros-melodic

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y

sudo apt install python-rosdep -y
sudo rosdep init
rosdep update

sudo apt-get install astyle build-essential ccache clang clang-tidy cmake cppcheck doxygen file g++ gcc gdb git lcov make ninja-build python3 python3-dev python3-pip python3-setuptools python3-wheel rsync shellcheck unzip xsltproc zip libeigen3-dev libopencv-dev libroscpp-dev protobuf-compiler python-pip python3-pip ninja-build gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev xvfb catkin python-catkin-tools -y
pip install --user argparse cerberus empy jinja2 numpy packaging pandas psutil pygments pyros-genmsg pyserial pyulog pyyaml setuptools six toml wheel rosdep
pip3 install --user --upgrade empy jinja2 numpy packaging pyros-genmsg toml pyyaml pymavlink

if [ -d ~/Firmware]; then
    echo "PX4 Firmware is installed"
else
    pushd ~
        git clone https://github.com/PX4/Firmware.git
    popd
    pushd ~/Firmware
        git submodule update --init --recursive
    popd
    pushd ~/Firmware
        DONT_RUN=1 make px4_sitl_default gazebo
    popd
fi

sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y

pushd ./catkin_ws
    chmod +x ./src/hole_detector/src/offb_control.py
    catkin build || catkin_make
popd

pwd

pushd ~
    curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash install_geographiclib_datasets.sh 
popd

if [ -d ~/eit_ws/src ]; then
    echo "eit_ws already exists"
else
    source /opt/ros/melodic/setup.bash
    mkdir -p ~/eit_ws/src
    pwd
    pushd ~/eit_ws/src/
        catkin_init_workspace
    popd
    pushd ~/eit_ws/
        catkin build
        sudo apt-get install unzip
    popd
    pushd ~/eit_ws/src
        wget https://nextcloud.sdu.dk/index.php/s/oycQ3pgQPXASHEY/download
        unzip download
        rm download
    popd
fi

cp -rf ./sdu_drone_mono_cam.sdf ~/eit_ws/src/eit_playground/models/sdu_drone_mono_cam

if [ -f ~/eit_ws/src/eit_playground/worlds/hca_airport_fence_complex.world ]; then
    echo "World file already exists"
else
    pushd ~/eit_ws/src/eit_playground/worlds/
        wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1b8MNieB_JLFnwehBVa12ll2kzuR6cK6_' -O hca_airport_fence_complex.world
    popd
fi

bash start.sh