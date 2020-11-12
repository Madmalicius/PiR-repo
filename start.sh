#!/bin/bash

source /home/$USER/Firmware/Tools/setup_gazebo.bash /home/$USER/Firmware /home/$USER/Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware/Tools/sitl_gazebo
export SVGA_VGPU10=0

gnome-terminal -- bash -c "roslaunch mavros px4.launch fcu_url:='udp://:14540@127.0.0.1:14557'"

gnome-terminal -- bash -c "source ~/eit_ws/devel/setup.bash && source ~/eit_ws/src/eit_playground/setup_gazebo.bash && cd ~/eit_ws/ && (catkin build || catkin_make) ; roslaunch --wait eit_playground posix.launch vehicle:=sdu_drone_mono_cam env:=hca_airport_fence_complex"

gnome-terminal -- bash -c "cd ./catkin_ws && source devel/setup.bash && rosrun hole_detector offb_control.py"
