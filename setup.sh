#!/bin/bash

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